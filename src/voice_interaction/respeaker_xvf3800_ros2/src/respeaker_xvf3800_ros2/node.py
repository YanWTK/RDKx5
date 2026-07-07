"""ROS 2 node for the Seeed reSpeaker XVF3800."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Any

import rclpy
from audio_msg.msg import AudioFrame, AudioFrameType
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger

from .audio import (
    ArecordStream,
    AudioIntegrityError,
    AudioIntegrityMonitor,
    chunk_to_arrays,
    find_usb_sysfs_device,
    reset_usb_authorization,
    run_audio_loop,
    select_profile,
)
from .backend import XVF3800Error, XVF3800USBBackend


def _parameter_value(value: Any) -> ParameterValue:
    msg = ParameterValue()
    if isinstance(value, bool):
        msg.type = ParameterType.PARAMETER_BOOL
        msg.bool_value = value
    elif isinstance(value, int):
        msg.type = ParameterType.PARAMETER_INTEGER
        msg.integer_value = value
    elif isinstance(value, float):
        msg.type = ParameterType.PARAMETER_DOUBLE
        msg.double_value = value
    elif isinstance(value, str):
        msg.type = ParameterType.PARAMETER_STRING
        msg.string_value = value
    elif isinstance(value, list):
        if not value:
            msg.type = ParameterType.PARAMETER_INTEGER_ARRAY
            msg.integer_array_value = []
        elif all(isinstance(item, int) for item in value):
            msg.type = ParameterType.PARAMETER_INTEGER_ARRAY
            msg.integer_array_value = value
        elif all(isinstance(item, float) for item in value):
            msg.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            msg.double_array_value = value
        else:
            msg.type = ParameterType.PARAMETER_STRING_ARRAY
            msg.string_array_value = [str(item) for item in value]
    else:
        msg.type = ParameterType.PARAMETER_STRING
        msg.string_value = str(value)
    return msg


class XVF3800Node(Node):
    def __init__(self) -> None:
        super().__init__("respeaker_xvf3800_node")

        self.declare_parameter("usb_vid", 0x2886)
        self.declare_parameter("usb_pid", 0x001E)
        self.declare_parameter("device_index", 0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channel_count", 6)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("doa_rate", 50.0)
        self.declare_parameter("doa_smoothing", 0.4)
        self.declare_parameter("track_only_when_vad", True)
        self.declare_parameter("doa_when_vad_lost", "hold")
        self.declare_parameter("vad_hold_ms", 200)
        self.declare_parameter("min_speech_energy", 0.0)
        self.declare_parameter("voice_switch_margin", 1.5)
        self.declare_parameter("voice_switch_min_hold_ms", 250)
        self.declare_parameter("snap_on_voice_onset", True)
        self.declare_parameter("enable_audio_publish", True)
        self.declare_parameter("enable_led_control", True)
        self.declare_parameter("enable_vad_publish", True)
        self.declare_parameter("apply_audio_tuning", False)
        self.declare_parameter("mic_gain", 90.0)
        self.declare_parameter("ref_gain", 8.0)
        self.declare_parameter("agc_max_gain", 64.0)
        self.declare_parameter("agc_gain", 2.0)
        self.declare_parameter("asr_output_gain", 1.0)
        self.declare_parameter("audio_output_left", [6, 3])
        self.declare_parameter("audio_output_right", [7, 3])
        self.declare_parameter("audio_device", "hw:C16K6Ch,0")
        self.declare_parameter("audio_format", "S16_LE")
        self.declare_parameter("audio_chunk_frames", 320)
        self.declare_parameter("audio_profile", "auto")
        self.declare_parameter("audio_watchdog_enabled", True)
        self.declare_parameter("audio_watchdog_zero_fraction", 0.80)
        self.declare_parameter("audio_watchdog_bad_seconds", 2.0)
        self.declare_parameter("audio_watchdog_usb_reset_after", 3)
        self.declare_parameter("audio_watchdog_recovery_window", 30.0)
        self.declare_parameter("audio_watchdog_enable_usb_reset", True)
        self.declare_parameter("audio_watchdog_usb_reset_cooldown", 300.0)
        self.declare_parameter("usb_sysfs_root", "/sys/bus/usb/devices")
        self.declare_parameter("usb_reset_disconnect_seconds", 1.0)

        self.usb_vid = int(self.get_parameter("usb_vid").value)
        self.usb_pid = int(self.get_parameter("usb_pid").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channel_count = int(self.get_parameter("channel_count").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.doa_rate = float(self.get_parameter("doa_rate").value)
        self.doa_smoothing = float(self.get_parameter("doa_smoothing").value)
        self.track_only_when_vad = bool(self.get_parameter("track_only_when_vad").value)
        self.doa_when_vad_lost = str(self.get_parameter("doa_when_vad_lost").value)
        self.vad_hold_ms = int(self.get_parameter("vad_hold_ms").value)
        self.min_speech_energy = float(self.get_parameter("min_speech_energy").value)
        self.voice_switch_margin = float(self.get_parameter("voice_switch_margin").value)
        self.voice_switch_min_hold_ms = int(self.get_parameter("voice_switch_min_hold_ms").value)
        self.snap_on_voice_onset = bool(self.get_parameter("snap_on_voice_onset").value)
        self.enable_audio_publish = bool(self.get_parameter("enable_audio_publish").value)
        self.enable_led_control = bool(self.get_parameter("enable_led_control").value)
        self.enable_vad_publish = bool(self.get_parameter("enable_vad_publish").value)
        self.apply_audio_tuning = bool(self.get_parameter("apply_audio_tuning").value)
        self.mic_gain = float(self.get_parameter("mic_gain").value)
        self.ref_gain = float(self.get_parameter("ref_gain").value)
        self.agc_max_gain = float(self.get_parameter("agc_max_gain").value)
        self.agc_gain = float(self.get_parameter("agc_gain").value)
        self.asr_output_gain = float(self.get_parameter("asr_output_gain").value)
        self.audio_output_left = [int(value) for value in self.get_parameter("audio_output_left").value]
        self.audio_output_right = [int(value) for value in self.get_parameter("audio_output_right").value]
        self.audio_device = str(self.get_parameter("audio_device").value)
        self.audio_format = str(self.get_parameter("audio_format").value)
        self.audio_chunk_frames = int(self.get_parameter("audio_chunk_frames").value)
        self.audio_profile_name = str(self.get_parameter("audio_profile").value)
        self.audio_watchdog_enabled = bool(self.get_parameter("audio_watchdog_enabled").value)
        self.audio_watchdog_zero_fraction = float(
            self.get_parameter("audio_watchdog_zero_fraction").value
        )
        self.audio_watchdog_bad_seconds = float(
            self.get_parameter("audio_watchdog_bad_seconds").value
        )
        self.audio_watchdog_usb_reset_after = max(
            int(self.get_parameter("audio_watchdog_usb_reset_after").value), 1
        )
        self.audio_watchdog_recovery_window = max(
            float(self.get_parameter("audio_watchdog_recovery_window").value), 1.0
        )
        self.audio_watchdog_enable_usb_reset = bool(
            self.get_parameter("audio_watchdog_enable_usb_reset").value
        )
        self.audio_watchdog_usb_reset_cooldown = max(
            float(self.get_parameter("audio_watchdog_usb_reset_cooldown").value), 0.0
        )
        self.usb_sysfs_root = str(self.get_parameter("usb_sysfs_root").value)
        self.usb_reset_disconnect_seconds = max(
            float(self.get_parameter("usb_reset_disconnect_seconds").value), 0.1
        )

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._audio_thread: threading.Thread | None = None
        self._backend = None
        self._smoothed_doa_vector: tuple[float, float] | None = None
        self._last_status: dict[str, Any] = {}
        self._audio_frame_index = 0
        self._last_voice_time: float | None = None
        self._last_published_doa: float | None = None
        self._tracked_beam_index: int | None = None
        self._tracked_beam_since: float | None = None
        self._was_voice_active = False
        self._audio_integrity_restarts: deque[float] = deque()
        self._audio_integrity_failures = 0
        self._usb_auto_reset_count = 0
        self._last_usb_auto_reset = float("-inf")

        self.doa_pub = self.create_publisher(Float32, "/xvf3800/doa_deg", 10)
        self.vad_pub = self.create_publisher(Bool, "/xvf3800/vad", 10)
        self.status_pub = self.create_publisher(String, "/xvf3800/status", 10)
        self.audio_raw_pub = self.create_publisher(AudioFrame, "/xvf3800/audio/raw", 10)
        self.audio_conference_pub = self.create_publisher(AudioFrame, "/xvf3800/audio/conference", 10)
        self.audio_asr_pub = self.create_publisher(AudioFrame, "/xvf3800/audio/asr", 10)

        self.get_status_srv = self.create_service(Trigger, "/xvf3800/get_status", self._handle_get_status)
        self.set_mute_srv = self.create_service(SetBool, "/xvf3800/set_mute", self._handle_set_mute)
        self.set_led_srv = self.create_service(SetParameters, "/xvf3800/set_led", self._handle_set_led)
        self.set_gain_srv = self.create_service(SetParameters, "/xvf3800/set_gain", self._handle_set_gain)
        self.reboot_srv = self.create_service(Trigger, "/xvf3800/reboot_device", self._handle_reboot)
        self.reset_srv = self.create_service(Trigger, "/xvf3800/reset_device", self._handle_reset)
        self.save_srv = self.create_service(Trigger, "/xvf3800/save_configuration", self._handle_save)

        self._connect_backend()
        self._status_timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self._poll_status)
        self._doa_timer = self.create_timer(1.0 / max(self.doa_rate, 1.0), self._poll_doa)
        if self.enable_audio_publish:
            self._start_audio_thread()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._audio_thread and self._audio_thread.is_alive():
            self._audio_thread.join(timeout=3.0)
        with self._lock:
            if self._backend:
                self._backend.close()
                self._backend = None
        return super().destroy_node()

    def _connect_backend(self) -> None:
        if self._backend:
            self._backend.close()
        self.get_logger().info(
            f"connecting to XVF3800 vid=0x{self.usb_vid:04x} pid=0x{self.usb_pid:04x} index={self.device_index}"
        )
        self._backend = XVF3800USBBackend(self.usb_vid, self.usb_pid, self.device_index)
        self._apply_audio_tuning()

    def _start_audio_thread(self) -> None:
        if self._audio_thread and self._audio_thread.is_alive():
            return

        def factory() -> ArecordStream:
            return ArecordStream(
                device=self.audio_device,
                sample_rate=self.sample_rate,
                channel_count=self.channel_count,
                chunk_frames=self.audio_chunk_frames,
                audio_format=self.audio_format,
            )

        profile = select_profile(self.channel_count, self.audio_profile_name)
        integrity_monitor = AudioIntegrityMonitor(
            sample_rate=self.sample_rate,
            zero_fraction_threshold=self.audio_watchdog_zero_fraction,
            bad_seconds=self.audio_watchdog_bad_seconds,
        )

        def on_chunk(chunk: bytes, stop_event: threading.Event, frame_index: int) -> None:
            frames = chunk_to_arrays(chunk, self.channel_count, self.audio_format)
            if self.audio_watchdog_enabled:
                integrity_monitor.observe(frames)
            now = self.get_clock().now().to_msg()
            if self.audio_raw_pub.get_subscription_count() > 0:
                self._publish_audio(
                    self.audio_raw_pub, frames.tobytes(), now, frame_index
                )
            if self.audio_conference_pub.get_subscription_count() > 0:
                self._publish_audio(
                    self.audio_conference_pub,
                    frames[:, profile.conference_channel].tobytes(),
                    now,
                    frame_index,
                )
            if self.audio_asr_pub.get_subscription_count() > 0:
                self._publish_audio(
                    self.audio_asr_pub,
                    frames[:, profile.asr_channel].tobytes(),
                    now,
                    frame_index,
                )

        self._audio_thread = threading.Thread(
            target=run_audio_loop,
            args=(factory, self._stop_event, on_chunk, self.get_logger()),
            kwargs={"on_error": self._handle_audio_capture_error},
            daemon=True,
        )
        self._audio_thread.start()

    def _handle_audio_capture_error(self, exc: Exception) -> None:
        if not isinstance(exc, AudioIntegrityError):
            return
        now = time.monotonic()
        self._audio_integrity_failures += 1
        self._audio_integrity_restarts.append(now)
        cutoff = now - self.audio_watchdog_recovery_window
        while self._audio_integrity_restarts and self._audio_integrity_restarts[0] < cutoff:
            self._audio_integrity_restarts.popleft()
        attempt_count = len(self._audio_integrity_restarts)
        self.get_logger().warning(
            "audio integrity watchdog restarting arecord "
            f"({attempt_count}/{self.audio_watchdog_usb_reset_after} before USB reset)"
        )
        if not self.audio_watchdog_enable_usb_reset:
            return
        if attempt_count < self.audio_watchdog_usb_reset_after:
            return
        if now - self._last_usb_auto_reset < self.audio_watchdog_usb_reset_cooldown:
            self.get_logger().warning("USB auto-reset suppressed by cooldown")
            return
        self._reset_usb_audio_device()
        self._audio_integrity_restarts.clear()

    def _reset_usb_audio_device(self) -> None:
        self.get_logger().error(
            "persistent zero-filled audio detected; re-authorizing the XVF3800 USB device"
        )
        with self._lock:
            if self._backend:
                self._backend.close()
                self._backend = None
            device_path = find_usb_sysfs_device(
                self.usb_vid,
                self.usb_pid,
                self.device_index,
                self.usb_sysfs_root,
            )
            reset_usb_authorization(device_path, self.usb_reset_disconnect_seconds)
            deadline = time.monotonic() + 10.0
            last_error: Exception | None = None
            while time.monotonic() < deadline and not self._stop_event.is_set():
                try:
                    self._connect_backend()
                    last_error = None
                    break
                except Exception as exc:
                    last_error = exc
                    time.sleep(0.5)
            if last_error is not None:
                raise RuntimeError(f"XVF3800 did not return after USB reset: {last_error}")
        self._last_usb_auto_reset = time.monotonic()
        self._usb_auto_reset_count += 1
        self.get_logger().warning("XVF3800 USB auto-reset completed")

    def _apply_audio_tuning(self) -> None:
        if not self.apply_audio_tuning or self._backend is None:
            return

        try:
            self._backend.set_gain(mic_gain=self.mic_gain, ref_gain=self.ref_gain)
            self._backend.set_agc(max_gain=self.agc_max_gain, gain=self.agc_gain)
            self._backend.set_asr_output_gain(self.asr_output_gain)
            self._backend.set_audio_output_muxes(
                left_mux=self.audio_output_left,
                right_mux=self.audio_output_right,
            )
            self.get_logger().info(
                "Applied XVF3800 audio tuning: "
                f"mic_gain={self.mic_gain:.1f} ref_gain={self.ref_gain:.1f} "
                f"agc_max_gain={self.agc_max_gain:.1f} agc_gain={self.agc_gain:.1f} "
                f"left_mux={self.audio_output_left} right_mux={self.audio_output_right}"
            )
        except Exception as exc:
            self.get_logger().warning(f"failed to apply XVF3800 audio tuning: {exc}")

    def _publish_audio(self, publisher, payload: bytes, stamp, frame_index: int) -> None:
        frame = AudioFrame()
        frame.index = frame_index
        frame.pts = stamp
        frame.frame_type.value = AudioFrameType.FRAME_TYPE_AUDIO
        frame.data = payload
        publisher.publish(frame)

    def _with_backend(self, fn):
        with self._lock:
            if not self._backend:
                self._connect_backend()
            return fn(self._backend)

    def _poll_status(self) -> None:
        try:
            status = self._with_backend(lambda backend: backend.snapshot())
            self._augment_status(status)
            self._last_status = status
            self._publish_status(status)
        except Exception as exc:
            self.get_logger().error(f"status poll failed: {exc}")
            try:
                with self._lock:
                    if self._backend:
                        self._backend.reconnect()
            except Exception as reconnect_exc:
                self.get_logger().error(f"reconnect failed: {reconnect_exc}")

    def _poll_doa(self) -> None:
        try:
            data = self._with_backend(lambda backend: backend.doa_snapshot())
        except Exception as exc:
            self.get_logger().debug(f"doa poll failed: {exc}")
            return
        vad = data.get("vad")
        if self.enable_vad_publish and vad is not None:
            self.vad_pub.publish(Bool(data=bool(vad)))
        doa = data.get("doa_deg")
        azimuths = data.get("azimuth_values") or []
        energies = data.get("speech_energy") or []
        self._publish_tracked_doa(doa, vad, azimuths, energies)

    def _augment_status(self, status: dict[str, Any]) -> None:
        status.update(
            {
                "audio_device": self.audio_device,
                "audio_format": self.audio_format,
                "channel_count": self.channel_count,
                "sample_rate": self.sample_rate,
                "audio_profile": self.audio_profile_name,
                "doa_smoothing": self.doa_smoothing,
                "doa_rate": self.doa_rate,
                "track_only_when_vad": self.track_only_when_vad,
                "doa_when_vad_lost": self.doa_when_vad_lost,
                "vad_hold_ms": self.vad_hold_ms,
                "min_speech_energy": self.min_speech_energy,
                "voice_switch_margin": self.voice_switch_margin,
                "voice_switch_min_hold_ms": self.voice_switch_min_hold_ms,
                "tracked_beam_index": self._tracked_beam_index,
                "audio_watchdog_enabled": self.audio_watchdog_enabled,
                "audio_integrity_failures": self._audio_integrity_failures,
                "usb_auto_reset_count": self._usb_auto_reset_count,
            }
        )

    def _select_voice_beam(
        self, azimuths: list[float], energies: list[float], now_sec: float
    ) -> tuple[float | None, float | None, int | None]:
        """Pick the loudest voice beam, with hysteresis to keep tracking stable.

        Returns (azimuth_deg, energy, beam_index) or (None, None, None) when no
        beam meets the speech-energy threshold.
        """
        if not azimuths or not energies:
            return None, None, None
        n = min(len(azimuths), len(energies))
        if n == 0:
            return None, None, None

        loudest_idx = 0
        loudest_energy = float(energies[0])
        for i in range(1, n):
            e = float(energies[i])
            if e > loudest_energy:
                loudest_energy = e
                loudest_idx = i

        if loudest_energy < self.min_speech_energy:
            return None, None, None

        # Hysteresis: stick with the previously tracked beam unless a different
        # beam is louder by `voice_switch_margin` AND the held beam has been
        # held for at least `voice_switch_min_hold_ms`.
        chosen_idx = loudest_idx
        held_idx = self._tracked_beam_index
        if held_idx is not None and 0 <= held_idx < n:
            held_energy = float(energies[held_idx])
            held_for = (now_sec - self._tracked_beam_since) if self._tracked_beam_since else float("inf")
            min_hold = self.voice_switch_min_hold_ms / 1000.0
            margin = max(self.voice_switch_margin, 1.0)
            if held_idx != loudest_idx:
                if held_for < min_hold or loudest_energy < held_energy * margin:
                    chosen_idx = held_idx

        if chosen_idx != self._tracked_beam_index:
            self._tracked_beam_index = chosen_idx
            self._tracked_beam_since = now_sec

        azimuth_rad = float(azimuths[chosen_idx])
        azimuth_deg = (math.degrees(azimuth_rad) + 360.0) % 360.0
        return azimuth_deg, float(energies[chosen_idx]), chosen_idx

    def _publish_tracked_doa(
        self,
        fallback_doa_deg: float | None,
        vad: Any,
        azimuths: list[float],
        energies: list[float],
    ) -> None:
        vad_active = bool(vad) if vad is not None else False
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        hold_sec = max(self.vad_hold_ms, 0) / 1000.0
        recently_voice = (
            self._last_voice_time is not None and (now_sec - self._last_voice_time) <= hold_sec
        )

        if vad_active:
            beam_doa, _energy, _idx = self._select_voice_beam(azimuths, energies, now_sec)
            raw_doa = beam_doa if beam_doa is not None else fallback_doa_deg
            if raw_doa is None:
                return
            self._last_voice_time = now_sec
            voice_onset = not self._was_voice_active
            self._was_voice_active = True
            tracked_doa = self._smooth_doa(float(raw_doa), reset=voice_onset and self.snap_on_voice_onset)
            self._last_published_doa = tracked_doa
            self.doa_pub.publish(Float32(data=tracked_doa))
            return

        # VAD inactive
        self._was_voice_active = False
        if not self.track_only_when_vad and fallback_doa_deg is not None:
            tracked_doa = self._smooth_doa(float(fallback_doa_deg), reset=False)
            self._last_published_doa = tracked_doa
            self.doa_pub.publish(Float32(data=tracked_doa))
            return

        if self._last_published_doa is None:
            return

        if self.doa_when_vad_lost == "hold" or recently_voice:
            self.doa_pub.publish(Float32(data=self._last_published_doa))

    def _smooth_doa(self, doa_deg: float, reset: bool = False) -> float:
        angle = math.radians(doa_deg)
        current = (math.cos(angle), math.sin(angle))
        if reset or self._smoothed_doa_vector is None:
            self._smoothed_doa_vector = current
        else:
            alpha = min(max(self.doa_smoothing, 0.0), 1.0)
            prev = self._smoothed_doa_vector
            blended = (
                (1.0 - alpha) * prev[0] + alpha * current[0],
                (1.0 - alpha) * prev[1] + alpha * current[1],
            )
            norm = math.hypot(blended[0], blended[1]) or 1.0
            self._smoothed_doa_vector = (blended[0] / norm, blended[1] / norm)
        return (
            math.degrees(math.atan2(self._smoothed_doa_vector[1], self._smoothed_doa_vector[0])) + 360.0
        ) % 360.0

    def _publish_status(self, status: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(msg)

    def _handle_get_status(self, request, response):
        try:
            status = self._with_backend(lambda backend: backend.snapshot())
            self._augment_status(status)
            self._last_status = status
            self._publish_status(status)
            response.success = True
            response.message = json.dumps(status, sort_keys=True)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_set_mute(self, request, response):
        try:
            self._with_backend(lambda backend: backend.set_mute(bool(request.data)))
            response.success = True
            response.message = "mute updated"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_set_led(self, request, response):
        result = []
        try:
            if not self.enable_led_control:
                raise RuntimeError("LED control is disabled by parameter")
            params = {param.name: param.value for param in request.parameters}
            led_kwargs: dict[str, Any] = {}
            mapping = {
                "effect": ("effect", int),
                "brightness": ("brightness", int),
                "gammify": ("gammify", bool),
                "speed": ("speed", int),
                "color": ("color", int),
                "doa_base_color": ("doa_base_color", int),
                "doa_highlight_color": ("doa_highlight_color", int),
                "ring_colors": ("ring_colors", list),
            }
            normalized: dict[str, Any] = {}
            for key, param in params.items():
                if key not in mapping:
                    result.append(SetParametersResult(successful=False, reason=f"unsupported led parameter: {key}"))
                    continue
                _, expected = mapping[key]
                value: Any
                if expected is bool:
                    value = bool(param.bool_value if param.type == ParameterType.PARAMETER_BOOL else param.integer_value)
                elif expected is int:
                    value = int(param.integer_value or param.double_value or 0)
                elif expected is list:
                    value = list(param.integer_array_value)
                else:
                    value = param.string_value
                normalized[key] = value
                result.append(SetParametersResult(successful=True, reason=""))

            if any(not item.successful for item in result):
                response.results = result
                return response

            if "doa_base_color" in normalized or "doa_highlight_color" in normalized:
                led_kwargs["doa_colors"] = [
                    int(normalized.get("doa_base_color", 0)),
                    int(normalized.get("doa_highlight_color", 0)),
                ]
            if "ring_colors" in normalized:
                led_kwargs["ring_colors"] = normalized["ring_colors"]
            for key in ("effect", "brightness", "gammify", "speed", "color"):
                if key in normalized:
                    led_kwargs[key] = normalized[key]

            self._with_backend(lambda backend: backend.set_led(**led_kwargs))
        except Exception as exc:
            if not result:
                result.append(SetParametersResult(successful=False, reason=str(exc)))
            else:
                for item in result:
                    if item.successful:
                        item.reason = str(exc)
                        item.successful = False
            response.results = result
            return response

        response.results = result or [SetParametersResult(successful=True, reason="")]
        return response

    def _handle_set_gain(self, request, response):
        result = []
        try:
            params = {param.name: param.value for param in request.parameters}
            gain_kwargs: dict[str, Any] = {}
            for key, param in params.items():
                if key not in {"mic_gain", "ref_gain"}:
                    result.append(SetParametersResult(successful=False, reason=f"unsupported gain parameter: {key}"))
                    continue
                gain_kwargs[key] = float(param.double_value or param.integer_value or 0.0)
                result.append(SetParametersResult(successful=True, reason=""))
            if any(not item.successful for item in result):
                response.results = result
                return response
            self._with_backend(lambda backend: backend.set_gain(**gain_kwargs))
        except Exception as exc:
            if not result:
                result.append(SetParametersResult(successful=False, reason=str(exc)))
            else:
                for item in result:
                    if item.successful:
                        item.reason = str(exc)
                        item.successful = False
            response.results = result
            return response
        response.results = result or [SetParametersResult(successful=True, reason="")]
        return response

    def _handle_reboot(self, request, response):
        try:
            self._with_backend(lambda backend: backend.reboot())
            response.success = True
            response.message = "reboot command sent"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_reset(self, request, response):
        try:
            self._with_backend(lambda backend: backend.reset_device())
            response.success = True
            response.message = "reset command sent"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_save(self, request, response):
        try:
            self._with_backend(lambda backend: backend.save_configuration())
            response.success = True
            response.message = "save command sent"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XVF3800Node()
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
