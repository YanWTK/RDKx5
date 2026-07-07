"""Record an XVF3800 AudioFrame topic to a WAV file."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import time
import wave
from collections import deque
from pathlib import Path
from threading import Lock

import numpy as np
import rclpy
from audio_msg.msg import AudioFrame
from rclpy.node import Node


class TopicRecorder(Node):
    """Subscribe to an AudioFrame topic and buffer PCM bytes."""

    def __init__(
        self,
        topic: str,
        sample_rate: int,
        channels: int,
        sample_width: int,
        duration: float,
        timeout: float,
    ) -> None:
        super().__init__("respeaker_xvf3800_audio_recorder")
        self._topic = topic
        self._sample_rate = sample_rate
        self._channels = channels
        self._sample_width = sample_width
        self._duration = duration
        self._timeout = timeout
        self._target_bytes = int(sample_rate * channels * sample_width * duration)
        self._lock = Lock()
        self._chunks: deque[bytes] = deque()
        self._captured_bytes = 0
        self._message_count = 0

        self._subscription = self.create_subscription(
            AudioFrame,
            topic,
            self._on_audio_frame,
            10,
        )

        self.get_logger().info(
            "recording topic=%s duration=%.1fs sample_rate=%d channels=%d sample_width=%d"
            % (topic, duration, sample_rate, channels, sample_width)
        )

    def _on_audio_frame(self, msg: AudioFrame) -> None:
        payload = bytes(msg.data)
        if not payload:
            return

        with self._lock:
            self._chunks.append(payload)
            self._captured_bytes += len(payload)
            self._message_count += 1

    def capture(self) -> bytes:
        """Collect enough bytes for the requested duration."""

        deadline = time.monotonic() + self._timeout
        last_log = 0.0
        while True:
            with self._lock:
                captured_bytes = self._captured_bytes
                chunks = list(self._chunks)
                message_count = self._message_count

            if captured_bytes >= self._target_bytes:
                data = b"".join(chunks)
                return data[-self._target_bytes :]

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    "Timed out while waiting for audio frames. "
                    f"Received {captured_bytes}/{self._target_bytes} bytes from {message_count} messages."
                )

            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    "capturing... %d/%d bytes from %d messages"
                    % (captured_bytes, self._target_bytes, message_count)
                )
                last_log = now

            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))


def _normalize_pcm(pcm_bytes: bytes, peak: int) -> bytes:
    """Boost a mono 16-bit stream to a practical listening level."""

    if not pcm_bytes:
        return pcm_bytes

    samples = np.frombuffer(pcm_bytes, dtype="<i2").astype(np.float32)
    if samples.size == 0:
        return pcm_bytes

    current_peak = float(np.max(np.abs(samples)))
    if current_peak <= 0.0:
        return pcm_bytes

    gain = min(float(peak) / current_peak, 8.0)
    if gain <= 1.0:
        return pcm_bytes

    normalized = np.clip(samples * gain, -32768, 32767).astype("<i2")
    return normalized.tobytes()


def _write_wav(path: Path, pcm_bytes: bytes, sample_rate: int, channels: int, sample_width: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm_bytes)


def _play_wav(path: Path) -> None:
    aplay = shutil.which("aplay")
    if not aplay:
        raise RuntimeError("aplay not found. Install alsa-utils or play the file manually.")
    subprocess.run([aplay, str(path)], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Record an XVF3800 audio topic to WAV")
    parser.add_argument(
        "--topic",
        default="/xvf3800/audio/conference",
        help="AudioFrame topic to record",
    )
    parser.add_argument(
        "--output",
        default="/tmp/xvf3800_audio.wav",
        help="Output WAV path",
    )
    parser.add_argument("--duration", type=float, default=5.0, help="Recording duration in seconds")
    parser.add_argument("--timeout", type=float, default=None, help="Capture timeout in seconds")
    parser.add_argument("--sample-rate", type=int, default=16000, help="WAV sample rate")
    parser.add_argument("--channels", type=int, default=1, help="WAV channel count")
    parser.add_argument("--sample-width", type=int, default=2, help="Bytes per sample")
    parser.add_argument("--normalize", dest="normalize", action="store_true", default=True, help="Normalize mono audio before saving")
    parser.add_argument("--no-normalize", dest="normalize", action="store_false", help="Save the captured samples without gain adjustment")
    parser.add_argument("--normalize-peak", type=int, default=24000, help="Peak target used by normalization")
    parser.add_argument("--play", action="store_true", help="Play the WAV after recording")
    args = parser.parse_args()

    timeout = args.timeout if args.timeout is not None else max(args.duration + 5.0, 8.0)
    output_path = Path(args.output).expanduser()

    rclpy.init()
    node = TopicRecorder(
        topic=args.topic,
        sample_rate=args.sample_rate,
        channels=args.channels,
        sample_width=args.sample_width,
        duration=args.duration,
        timeout=timeout,
    )

    try:
        pcm_bytes = node.capture()
        if args.normalize and args.channels == 1 and args.sample_width == 2:
            pcm_bytes = _normalize_pcm(pcm_bytes, args.normalize_peak)
        _write_wav(output_path, pcm_bytes, args.sample_rate, args.channels, args.sample_width)
        seconds = len(pcm_bytes) / float(args.sample_rate * args.channels * args.sample_width)
        print(f"saved {output_path} ({seconds:.2f}s, {len(pcm_bytes)} bytes)")
        if args.play:
            _play_wav(output_path)
        return 0
    except Exception as exc:
        print(f"error: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
