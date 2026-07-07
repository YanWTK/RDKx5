"""Wake-word detection node for XVF3800.

Uses on-device sherpa-onnx KeywordSpotter instead of cloud ASR for wake-word
detection. Eliminates the 2-4 s wake delay from the buffered audio scan +
Bailian ASR roundtrip; the command-recognition path (asr_node) is unchanged.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import sherpa_onnx
from audio_msg.msg import AudioFrame
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .speech_utils import normalize_pcm


DEFAULT_MODEL_DIR = "/opt/xiaogua/models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
DEFAULT_ENCODER = "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
DEFAULT_DECODER = "decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
DEFAULT_JOINER = "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
DEFAULT_KEYWORDS = "xiaogua_keywords.txt"


class XVF3800WakeWordNode(Node):
    """Detect a wake phrase from the ASR audio stream using on-device KWS."""

    def __init__(self) -> None:
        super().__init__("respeaker_xvf3800_wake_word_node")

        # KWS engine parameters
        self.declare_parameter("kws_model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("kws_encoder", DEFAULT_ENCODER)
        self.declare_parameter("kws_decoder", DEFAULT_DECODER)
        self.declare_parameter("kws_joiner", DEFAULT_JOINER)
        self.declare_parameter("kws_keywords_file", DEFAULT_KEYWORDS)
        self.declare_parameter("kws_threshold", 0.15)
        self.declare_parameter("kws_num_threads", 2)
        self.declare_parameter("kws_provider", "cpu")

        # Audio source parameters
        self.declare_parameter("audio_topic", "/xvf3800/audio/asr")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("audio_format", "S16_LE")
        self.declare_parameter("input_channel_count", 1)
        self.declare_parameter("normalize_audio", True)
        self.declare_parameter("normalize_peak", 24000)
        self.declare_parameter("max_normalize_gain", 16.0)

        # Wake-event behavior
        self.declare_parameter("wake_phrases", ["小瓜"])
        self.declare_parameter("cooldown_seconds", 3.0)
        self.declare_parameter("auto_trigger_asr", True)
        self.declare_parameter("skip_asr", False)
        self.declare_parameter("record_service", "/xvf3800/record_asr")
        self.declare_parameter("record_timeout", 20.0)
        self.declare_parameter("pause_after_detection", True)
        self.declare_parameter("enable_topic", "/xvf3800/kws/enabled")
        self.declare_parameter("active_topic", "/xvf3800/kws/active")
        self.declare_parameter("max_disabled_seconds", 600.0)

        # Legacy parameters from the cloud-ASR era. Kept for compatibility.
        for name, default in [
            ("vad_topic", "/xvf3800/vad"),
            ("scan_interval", 0.5),
            ("window_seconds", 2.2),
            ("buffer_seconds", 8.0),
            ("vad_stable_ms", 120),
            ("wake_vad_hold_ms", 1400),
            ("language", "zh"),
            ("base_url", ""),
            ("model", "qwen3-asr-flash"),
        ]:
            self.declare_parameter(name, default)

        # Resolve KWS file paths (relative paths are resolved against model_dir)
        model_dir = Path(str(self.get_parameter("kws_model_dir").value))

        def _resolve(name: str) -> Path:
            v = str(self.get_parameter(name).value)
            p = Path(v)
            return p if p.is_absolute() else model_dir / v

        encoder_path = _resolve("kws_encoder")
        decoder_path = _resolve("kws_decoder")
        joiner_path = _resolve("kws_joiner")
        keywords_path = _resolve("kws_keywords_file")
        tokens_path = model_dir / "tokens.txt"

        for label, path in [
            ("model dir", model_dir),
            ("encoder", encoder_path),
            ("decoder", decoder_path),
            ("joiner", joiner_path),
            ("keywords", keywords_path),
            ("tokens", tokens_path),
        ]:
            if not path.exists():
                raise FileNotFoundError(f"KWS {label} not found: {path}")

        self.kws_threshold = float(self.get_parameter("kws_threshold").value)
        self.kws_num_threads = int(self.get_parameter("kws_num_threads").value)
        self.kws_provider = str(self.get_parameter("kws_provider").value)

        self.audio_topic = str(self.get_parameter("audio_topic").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.audio_format = str(self.get_parameter("audio_format").value)
        self.input_channel_count = max(int(self.get_parameter("input_channel_count").value), 1)
        self.normalize_audio = bool(self.get_parameter("normalize_audio").value)
        self.normalize_peak = max(int(self.get_parameter("normalize_peak").value), 1)
        self.max_normalize_gain = max(float(self.get_parameter("max_normalize_gain").value), 1.0)

        self.wake_phrases = [str(v) for v in self.get_parameter("wake_phrases").value]
        self.cooldown_seconds = max(float(self.get_parameter("cooldown_seconds").value), 0.0)
        self.auto_trigger_asr = bool(self.get_parameter("auto_trigger_asr").value)
        self.skip_asr = bool(self.get_parameter("skip_asr").value)
        self.record_service = str(self.get_parameter("record_service").value)
        self.record_timeout = max(float(self.get_parameter("record_timeout").value), 1.0)
        self.pause_after_detection = bool(self.get_parameter("pause_after_detection").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.active_topic = str(self.get_parameter("active_topic").value)
        self.max_disabled_seconds = max(
            float(self.get_parameter("max_disabled_seconds").value), 0.0
        )

        # Load the KWS engine (one-shot, ~6 s on X5)
        self.get_logger().info(
            f"loading KWS model from {model_dir} "
            f"(threshold={self.kws_threshold}, threads={self.kws_num_threads})..."
        )
        t0 = time.monotonic()
        self.spotter = sherpa_onnx.KeywordSpotter(
            tokens=str(tokens_path),
            encoder=str(encoder_path),
            decoder=str(decoder_path),
            joiner=str(joiner_path),
            keywords_file=str(keywords_path),
            num_threads=self.kws_num_threads,
            keywords_threshold=self.kws_threshold,
            provider=self.kws_provider,
        )
        self.kws_stream = self.spotter.create_stream()
        self.get_logger().info(
            f"KWS loaded in {time.monotonic() - t0:.1f}s, keywords={keywords_path.name}"
        )

        # State
        self._state_lock = threading.Lock()
        self._cooldown_until = 0.0
        self._detection_in_flight = False
        self._enabled = True
        self._disabled_since: float | None = None

        # ROS interfaces
        self._record_client = (
            self.create_client(Trigger, self.record_service) if self.auto_trigger_asr else None
        )
        self.wake_detected_pub = self.create_publisher(Bool, "/xvf3800/wake_detected", 10)
        self.wake_event_pub = self.create_publisher(String, "/xvf3800/wake_event", 10)
        self.wake_asr_result_pub = self.create_publisher(String, "/xvf3800/wake_asr_result", 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_pub = self.create_publisher(Bool, self.active_topic, state_qos)
        self.enable_sub = self.create_subscription(
            Bool, self.enable_topic, self._on_enable, state_qos
        )

        audio_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.audio_sub = self.create_subscription(
            AudioFrame, self.audio_topic, self._on_audio_frame, audio_qos
        )
        self.create_timer(1.0, self._disabled_watchdog)
        self._publish_active_state()

        self.get_logger().info(
            "wake node (KWS) ready. topic=%s phrases=%s cooldown=%.1fs "
            "auto_trigger_asr=%s skip_asr=%s pause_after_detection=%s "
            "normalize=%s peak=%d max_gain=%.1f"
            % (
                self.audio_topic,
                self.wake_phrases,
                self.cooldown_seconds,
                self.auto_trigger_asr,
                self.skip_asr,
                self.pause_after_detection,
                self.normalize_audio,
                self.normalize_peak,
                self.max_normalize_gain,
            )
        )

    # ------------------------------------------------------------------
    # Audio in: feed every frame to KWS, fire on detection.
    # ------------------------------------------------------------------
    def _on_audio_frame(self, msg: AudioFrame) -> None:
        payload = bytes(msg.data)
        if not payload:
            return

        with self._state_lock:
            now = time.monotonic()
            if not self._enabled or self._detection_in_flight or now < self._cooldown_until:
                return  # drop audio while a wake is being handled / in cooldown

        pcm = self._to_mono_s16le(payload)
        if not pcm:
            return
        if self.normalize_audio:
            pcm = normalize_pcm(
                pcm,
                peak=self.normalize_peak,
                max_gain=self.max_normalize_gain,
            )
        samples = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
        if samples.size == 0:
            return

        self.kws_stream.accept_waveform(self.sample_rate, samples)
        while self.spotter.is_ready(self.kws_stream):
            self.spotter.decode_stream(self.kws_stream)
        kw = self.spotter.get_result(self.kws_stream)
        if not kw:
            return

        with self._state_lock:
            now = time.monotonic()
            if self._detection_in_flight or now < self._cooldown_until:
                self.spotter.reset_stream(self.kws_stream)
                return
            self._cooldown_until = now + self.cooldown_seconds
            self._detection_in_flight = True
            if self.pause_after_detection:
                self._enabled = False
                self._disabled_since = now
            self.spotter.reset_stream(self.kws_stream)

        if self.pause_after_detection:
            self._publish_active_state()
        threading.Thread(target=self._handle_wake, args=(kw,), daemon=True).start()

    def _on_enable(self, msg: Bool) -> None:
        self._set_enabled(bool(msg.data), "control topic")

    def _set_enabled(self, enabled: bool, reason: str) -> None:
        with self._state_lock:
            if self._enabled == enabled:
                return
            self._enabled = enabled
            self._disabled_since = None if enabled else time.monotonic()
            if enabled:
                self._cooldown_until = 0.0
        # Control callbacks and audio callbacks run on the same executor thread,
        # so resetting the stream here cannot race with decode_stream().
        self.spotter.reset_stream(self.kws_stream)
        self._publish_active_state()
        self.get_logger().info(
            f"KWS {'enabled' if enabled else 'paused'} ({reason})"
        )

    def _disabled_watchdog(self) -> None:
        if self.max_disabled_seconds <= 0.0:
            return
        with self._state_lock:
            disabled_since = self._disabled_since
            should_enable = (
                not self._enabled
                and disabled_since is not None
                and time.monotonic() - disabled_since >= self.max_disabled_seconds
            )
        if should_enable:
            self.get_logger().warning(
                "KWS pause watchdog expired; enabling wake detection as a fail-safe"
            )
            self._set_enabled(True, "pause watchdog")

    def _publish_active_state(self) -> None:
        with self._state_lock:
            enabled = self._enabled
        self.active_pub.publish(Bool(data=enabled))

    # ------------------------------------------------------------------
    # PCM normalisation: model wants 16 kHz mono int16
    # ------------------------------------------------------------------
    def _to_mono_s16le(self, pcm_bytes: bytes) -> bytes:
        if self.audio_format == "S16_LE":
            if self.input_channel_count == 1:
                return pcm_bytes
            samples = np.frombuffer(pcm_bytes, dtype="<i2")
            if samples.size == 0 or samples.size % self.input_channel_count != 0:
                return b""
            frames = samples.reshape((-1, self.input_channel_count))
            return frames[:, 0].tobytes()
        if self.audio_format == "S32_LE":
            samples = np.frombuffer(pcm_bytes, dtype="<i4")
            if samples.size == 0:
                return b""
            if self.input_channel_count > 1:
                if samples.size % self.input_channel_count != 0:
                    return b""
                frames = samples.reshape((-1, self.input_channel_count))
                samples = frames[:, 0]
            samples = np.clip(samples >> 16, -32768, 32767).astype("<i2")
            return samples.tobytes()
        raise ValueError(f"Unsupported audio format: {self.audio_format}")

    # ------------------------------------------------------------------
    # Wake handling: publish events and trigger the command ASR service.
    # ------------------------------------------------------------------
    def _handle_wake(self, matched_phrase: str) -> None:
        try:
            payload = {
                "wake_phrase": matched_phrase,
                "text": matched_phrase,
                "auto_trigger_asr": self.auto_trigger_asr,
            }

            if self.skip_asr:
                self._publish_wake_event(payload)
                self.get_logger().info(f"wake detected: {matched_phrase} (skip_asr)")
                return

            response_message = ""
            if self.auto_trigger_asr and self._record_client is not None:
                response_message = self._call_record_service()

            if response_message:
                payload["record_asr_response"] = response_message

            self._publish_wake_event(payload)

            if response_message:
                self._publish_wake_asr_result(response_message)
                try:
                    asr_payload = json.loads(response_message)
                    asr_text = str(asr_payload.get("text", "")).strip()
                except Exception:
                    asr_text = response_message.strip()
                self.get_logger().info(f"ASR结果: {asr_text or response_message}")
            else:
                self.get_logger().info(f"wake detected: {matched_phrase}")
        except Exception as exc:
            self.get_logger().error(f"wake handling failed: {exc}")
        finally:
            with self._state_lock:
                self._detection_in_flight = False

    def _call_record_service(self) -> str:
        if self._record_client is None:
            return ""
        if not self._record_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"record service not available: {self.record_service}")
            return ""

        request = Trigger.Request()
        future = self._record_client.call_async(request)
        deadline = time.monotonic() + self.record_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().warning(f"record service timed out: {self.record_service}")
            return ""

        response = future.result()
        if response is None:
            return ""
        if not response.success:
            self.get_logger().warning(f"record service failed: {response.message}")
        return response.message

    def _publish_wake_event(self, payload: dict[str, object]) -> None:
        message = json.dumps(payload, ensure_ascii=False)
        bool_msg = Bool()
        bool_msg.data = True
        self.wake_detected_pub.publish(bool_msg)

        text_msg = String()
        text_msg.data = message
        self.wake_event_pub.publish(text_msg)

    def _publish_wake_asr_result(self, response_message: str) -> None:
        text_msg = String()
        text_msg.data = response_message
        self.wake_asr_result_pub.publish(text_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XVF3800WakeWordNode()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
