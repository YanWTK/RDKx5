"""ASR node for the XVF3800 audio topic.

This node subscribes to the existing microphone audio topic published by the
XVF3800 driver, keeps a short rolling buffer, and exposes a Trigger service
that returns the ASR text and parsed command as JSON.
"""

from __future__ import annotations

import json
import time
import tempfile
import threading
import wave
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from audio_msg.msg import AudioFrame
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .asr_client import ASRClientError, ASRResponseError, BailianASRClient
from .command_parser import parse_robot_command
from .speech_utils import should_end_recording


def _pcm_to_s16le_bytes(pcm: bytes, audio_format: str) -> bytes:
    """Convert PCM bytes to signed 16-bit little-endian bytes."""

    if audio_format == "S16_LE":
        return pcm
    if audio_format == "S32_LE":
        samples = np.frombuffer(pcm, dtype="<i4")
        samples = np.clip(samples >> 16, -32768, 32767).astype("<i2")
        return samples.tobytes()
    raise ValueError(f"Unsupported audio format: {audio_format}")


class XVF3800ASRNode(Node):
    """Buffer audio frames and call Bailian ASR on demand."""

    def __init__(self) -> None:
        super().__init__("respeaker_xvf3800_asr_node")

        self.declare_parameter("audio_topic", "/xvf3800/audio/asr")
        self.declare_parameter("vad_topic", "/xvf3800/vad")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("audio_format", "S16_LE")
        self.declare_parameter("input_channel_count", 1)
        self.declare_parameter("asr_channel_candidates", [0])
        self.declare_parameter("normalize_audio", True)
        self.declare_parameter("normalize_peak", 24000)
        self.declare_parameter("record_duration", 15.0)
        self.declare_parameter("silence_timeout", 1.0)
        self.declare_parameter("buffer_seconds", 10.0)
        self.declare_parameter("language", "zh")
        self.declare_parameter("base_url", "")
        self.declare_parameter("model", "qwen3-asr-flash")

        self.audio_topic = str(self.get_parameter("audio_topic").value)
        self.vad_topic = str(self.get_parameter("vad_topic").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.audio_format = str(self.get_parameter("audio_format").value)
        self.input_channel_count = max(int(self.get_parameter("input_channel_count").value), 1)
        self.asr_channel_candidates = [int(value) for value in self.get_parameter("asr_channel_candidates").value]
        if not self.asr_channel_candidates:
            self.asr_channel_candidates = [1 if self.input_channel_count > 1 else 0]
        if self.input_channel_count == 1:
            self.asr_channel_candidates = [0]
        self.normalize_audio = bool(self.get_parameter("normalize_audio").value)
        self.normalize_peak = int(self.get_parameter("normalize_peak").value)
        self.record_duration = float(self.get_parameter("record_duration").value)
        self.silence_timeout = float(self.get_parameter("silence_timeout").value)
        self.buffer_seconds = float(self.get_parameter("buffer_seconds").value)
        self.language = str(self.get_parameter("language").value)
        self.base_url = str(self.get_parameter("base_url").value).strip() or None
        self.model = str(self.get_parameter("model").value)

        self._audio_lock = threading.Lock()
        self._audio_ready = threading.Condition(self._audio_lock)
        self._audio_chunks: deque[bytes] = deque()
        self._audio_bytes = 0
        self._total_audio_bytes = 0
        self._vad_active = False
        self._last_voice_time: float | None = None
        self._bytes_per_second = self.sample_rate * self.input_channel_count * self._bytes_per_sample()
        self._buffer_seconds = max(self.buffer_seconds, self.record_duration + self.silence_timeout + 1.0)
        self._max_buffer_bytes = int(self._buffer_seconds * self._bytes_per_second)
        self._record_timeout = max(self.record_duration + 2.0, 12.0)

        self._asr_client: BailianASRClient | None = None

        self._callback_group = ReentrantCallbackGroup()
        self.result_pub = self.create_publisher(String, "/xvf3800/asr/result", 10)
        self.recording_pub = self.create_publisher(
            Bool, "/xvf3800/asr/recording", 10
        )
        self.audio_sub = self.create_subscription(
            AudioFrame,
            self.audio_topic,
            self._on_audio_frame,
            10,
            callback_group=self._callback_group,
        )
        self.vad_sub = self.create_subscription(
            Bool,
            self.vad_topic,
            self._on_vad,
            10,
            callback_group=self._callback_group,
        )
        self.record_asr_srv = self.create_service(
            Trigger,
            "/xvf3800/record_asr",
            self._handle_record_asr,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "ASR node ready. audio_topic=%s vad_topic=%s max_duration=%.1fs silence_timeout=%.1fs buffer=%.1fs format=%s channels=%d candidates=%s"
            % (
                self.audio_topic,
                self.vad_topic,
                self.record_duration,
                self.silence_timeout,
                self.buffer_seconds,
                self.audio_format,
                self.input_channel_count,
                self.asr_channel_candidates,
            )
        )

    def _bytes_per_sample(self) -> int:
        if self.audio_format == "S16_LE":
            return 2
        if self.audio_format == "S32_LE":
            return 4
        raise ValueError(f"Unsupported audio format: {self.audio_format}")

    def _get_asr_client(self) -> BailianASRClient:
        if self._asr_client is None:
            self._asr_client = BailianASRClient(
                base_url=self.base_url,
                model=self.model,
                log_callback=self.get_logger().info,
            )
        return self._asr_client

    def _on_audio_frame(self, msg: AudioFrame) -> None:
        """Store the latest mono audio bytes from the microphone pipeline."""

        payload = bytes(msg.data)
        if not payload:
            return

        with self._audio_lock:
            self._audio_chunks.append(payload)
            self._audio_bytes += len(payload)
            self._total_audio_bytes += len(payload)

            while self._audio_bytes > self._max_buffer_bytes and self._audio_chunks:
                removed = self._audio_chunks.popleft()
                self._audio_bytes -= len(removed)
            self._audio_ready.notify_all()

    def _on_vad(self, msg: Bool) -> None:
        with self._audio_ready:
            now = time.monotonic()
            self._vad_active = bool(msg.data)
            if self._vad_active:
                self._last_voice_time = now
            self._audio_ready.notify_all()

    def _record_audio(self, max_duration: float, silence_timeout: float) -> bytes:
        """Record audio until silence or the maximum duration is reached."""

        if max_duration <= 0:
            raise RuntimeError("Invalid ASR duration")

        deadline = time.monotonic() + self._record_timeout
        start_time = time.monotonic()
        with self._audio_ready:
            start_total_bytes = self._total_audio_bytes
            # Don't inherit VAD activity from BEFORE the recording opened —
            # speaker leakage from "在呢" / wake-tail can pollute these and
            # cause silence_timeout to fire before the user speaks.
            voice_seen = False
            last_voice_time = None
            while True:
                now = time.monotonic()
                if should_end_recording(
                    now=now,
                    start_time=start_time,
                    voice_seen=voice_seen,
                    vad_active=self._vad_active,
                    last_voice_time=last_voice_time,
                    max_duration=max_duration,
                    silence_timeout=silence_timeout,
                ):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        "Timed out while waiting for microphone audio. "
                        f"Recording window did not finish within {self._record_timeout:.1f}s."
                    )
                self.get_logger().info(
                    f"Recording ASR audio (max {max_duration:.1f}s, silence {silence_timeout:.1f}s) "
                    f"({self._total_audio_bytes - start_total_bytes} bytes captured)"
                )
                self._audio_ready.wait(timeout=min(remaining, 0.1))
                if self._vad_active:
                    voice_seen = True
                    last_voice_time = self._last_voice_time or time.monotonic()
                elif self._last_voice_time is not None and self._last_voice_time >= start_time:
                    voice_seen = True
                    last_voice_time = self._last_voice_time
            data = b"".join(self._audio_chunks)
            captured_bytes = self._total_audio_bytes - start_total_bytes

        if captured_bytes <= 0:
            raise RuntimeError("No fresh microphone audio captured")
        if not voice_seen:
            raise RuntimeError("No speech detected during recording window")
        return data[-captured_bytes:]

    def _extract_channel_pcm(self, pcm_bytes: bytes, channel_index: int) -> bytes:
        """Extract one channel from interleaved PCM bytes."""

        if self.input_channel_count == 1:
            return pcm_bytes

        dtype = np.dtype("<i2") if self.audio_format == "S16_LE" else np.dtype("<i4")
        samples = np.frombuffer(pcm_bytes, dtype=dtype)
        if samples.size % self.input_channel_count != 0:
            raise ValueError("Audio buffer is not aligned to complete frames")
        frames = samples.reshape((-1, self.input_channel_count))
        if channel_index < 0 or channel_index >= self.input_channel_count:
            raise ValueError(
                f"ASR channel index {channel_index} is out of range for {self.input_channel_count} channels"
            )
        mono_pcm = frames[:, channel_index].tobytes()
        return _pcm_to_s16le_bytes(mono_pcm, self.audio_format)

    def _normalize_pcm(self, pcm_bytes: bytes) -> bytes:
        """Boost quiet mono PCM to a usable level without changing the sample rate."""

        if not self.normalize_audio or not pcm_bytes:
            return pcm_bytes

        samples = np.frombuffer(pcm_bytes, dtype="<i2").astype(np.float32)
        if samples.size == 0:
            return pcm_bytes
        peak = float(np.max(np.abs(samples)))
        if peak <= 0.0:
            return pcm_bytes
        gain = min(float(self.normalize_peak) / peak, 8.0)
        if gain <= 1.0:
            return pcm_bytes
        normalized = np.clip(samples * gain, -32768, 32767).astype("<i2")
        return normalized.tobytes()

    def _build_wav_file(self, pcm_bytes: bytes) -> str:
        """Write PCM bytes to a temporary WAV file."""

        temp_file = tempfile.NamedTemporaryFile(prefix="xvf3800_asr_", suffix=".wav", delete=False)
        temp_path = Path(temp_file.name)
        temp_file.close()

        with wave.open(str(temp_path), "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(pcm_bytes)

        return str(temp_path)

    def _handle_record_asr(self, request, response):
        """Record the buffered microphone audio and return text + command JSON."""

        temp_path: str | None = None
        recording_active = False
        try:
            self.recording_pub.publish(Bool(data=True))
            recording_active = True
            try:
                pcm_bytes = self._record_audio(
                    self.record_duration, self.silence_timeout
                )
            finally:
                self.recording_pub.publish(Bool(data=False))
                recording_active = False
            client = self._get_asr_client()
            last_error: ASRResponseError | None = None
            for channel_index in self.asr_channel_candidates:
                try:
                    mono_pcm = self._extract_channel_pcm(pcm_bytes, channel_index)
                    mono_pcm = self._normalize_pcm(mono_pcm)
                    temp_path = self._build_wav_file(mono_pcm)
                    text = client.transcribe_wav(temp_path, language=self.language)
                    command = parse_robot_command(text)
                    payload = {
                        "text": text,
                        "command": command,
                        "channel_index": channel_index,
                    }
                    message = json.dumps(payload, ensure_ascii=False)

                    result_msg = String()
                    result_msg.data = message
                    self.result_pub.publish(result_msg)

                    response.success = True
                    response.message = message
                    return response
                except ASRResponseError as exc:
                    last_error = exc
                    self.get_logger().warning(
                        f"ASR channel {channel_index} returned empty text, trying next candidate"
                    )
                    continue
                finally:
                    if temp_path:
                        Path(temp_path).unlink(missing_ok=True)
                        temp_path = None

            if last_error is not None:
                raise last_error
            raise ASRResponseError("ASR returned an empty text result.")
        except (ASRClientError, RuntimeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"ASR failed: {exc}")
            return response
        except Exception as exc:
            response.success = False
            response.message = f"Unexpected ASR failure: {exc}"
            self.get_logger().error(response.message)
            return response
        finally:
            if recording_active:
                self.recording_pub.publish(Bool(data=False))
            if temp_path:
                Path(temp_path).unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = XVF3800ASRNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
        raise SystemExit(1)
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
