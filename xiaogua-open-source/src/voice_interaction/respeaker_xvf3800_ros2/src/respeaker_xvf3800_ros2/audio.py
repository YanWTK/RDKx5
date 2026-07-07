"""ALSA capture helpers for XVF3800 USB audio output."""

from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class AudioProfile:
    name: str
    conference_channel: int
    asr_channel: int


PROCESSED_STEREO = AudioProfile("processed_stereo", 0, 1)
# Official 6-channel firmware layout:
# 0 = Conference, 1 = ASR, 2..5 = raw microphone channels.
RAW_SIX = AudioProfile("raw_six", 0, 1)

_AUDIO_DTYPE_BY_FORMAT = {
    "S16_LE": np.dtype("<i2"),
    "S32_LE": np.dtype("<i4"),
}


class AudioIntegrityError(RuntimeError):
    """Raised when the capture stream is running but its PCM is corrupted."""


class AudioIntegrityMonitor:
    """Detect sustained zero-filled USB audio without treating low volume as failure."""

    def __init__(
        self,
        sample_rate: int,
        zero_fraction_threshold: float = 0.80,
        bad_seconds: float = 2.0,
    ) -> None:
        self.sample_rate = max(int(sample_rate), 1)
        self.zero_fraction_threshold = min(max(float(zero_fraction_threshold), 0.0), 1.0)
        self.bad_sample_limit = max(int(float(bad_seconds) * self.sample_rate), 1)
        self.bad_samples = 0
        self.last_zero_fraction = 0.0

    def reset(self) -> None:
        self.bad_samples = 0
        self.last_zero_fraction = 0.0

    def observe(self, frames: np.ndarray) -> None:
        if frames.size == 0:
            return
        self.last_zero_fraction = 1.0 - (np.count_nonzero(frames) / frames.size)
        frame_count = int(frames.shape[0]) if frames.ndim > 1 else int(frames.size)
        if self.last_zero_fraction >= self.zero_fraction_threshold:
            self.bad_samples += frame_count
        else:
            self.bad_samples = 0
        if self.bad_samples >= self.bad_sample_limit:
            zero_pct = self.last_zero_fraction * 100.0
            self.reset()
            raise AudioIntegrityError(
                f"PCM remained {zero_pct:.1f}% zero-filled for the configured window"
            )


def find_usb_sysfs_device(
    vid: int,
    pid: int,
    device_index: int = 0,
    sysfs_root: str = "/sys/bus/usb/devices",
) -> Path:
    """Find a physical USB device (not an interface) by VID/PID."""
    matches: list[Path] = []
    for entry in sorted(Path(sysfs_root).iterdir()):
        if ":" in entry.name:
            continue
        try:
            entry_vid = int((entry / "idVendor").read_text().strip(), 16)
            entry_pid = int((entry / "idProduct").read_text().strip(), 16)
        except (FileNotFoundError, PermissionError, ValueError, OSError):
            continue
        if entry_vid == vid and entry_pid == pid:
            matches.append(entry)
    if device_index < 0 or device_index >= len(matches):
        raise FileNotFoundError(
            f"USB device {vid:04x}:{pid:04x} index={device_index} not found in {sysfs_root}"
        )
    return matches[device_index]


def reset_usb_authorization(device_path: Path, disconnect_seconds: float = 1.0) -> None:
    """Re-authorize one USB device, equivalent to a logical unplug/replug."""
    authorized = device_path / "authorized"
    if not authorized.exists():
        raise FileNotFoundError(f"USB authorization control not found: {authorized}")
    authorized.write_text("0")
    time.sleep(max(float(disconnect_seconds), 0.1))
    authorized.write_text("1")


def select_profile(channel_count: int, profile_name: str) -> AudioProfile:
    if profile_name == "processed_stereo":
        return PROCESSED_STEREO
    if profile_name == "raw_six":
        return RAW_SIX
    if channel_count >= 6:
        return RAW_SIX
    return PROCESSED_STEREO


def bytes_per_sample(audio_format: str) -> int:
    if audio_format not in _AUDIO_DTYPE_BY_FORMAT:
        raise ValueError(f"unsupported audio format: {audio_format}")
    return _AUDIO_DTYPE_BY_FORMAT[audio_format].itemsize


class ArecordStream:
    def __init__(
        self,
        device: str,
        sample_rate: int,
        channel_count: int,
        chunk_frames: int,
        audio_format: str = "S32_LE",
    ) -> None:
        self.device = device
        self.sample_rate = sample_rate
        self.channel_count = channel_count
        self.chunk_frames = chunk_frames
        self.audio_format = audio_format
        self.bytes_per_sample = bytes_per_sample(audio_format)
        self._process: subprocess.Popen | None = None

    def start(self) -> None:
        cmd = [
            "arecord",
            "-q",
            "-D",
            self.device,
            "-f",
            self.audio_format,
            "-c",
            str(self.channel_count),
            "-r",
            str(self.sample_rate),
            "-t",
            "raw",
        ]
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

    def stop(self) -> None:
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
        self._process = None

    def read_exact(self, byte_count: int) -> bytes:
        if not self._process or self._process.stdout is None:
            raise RuntimeError("audio stream is not running")
        buffer = bytearray()
        while len(buffer) < byte_count:
            chunk = self._process.stdout.read(byte_count - len(buffer))
            if not chunk:
                detail = self.stderr_text()
                if detail:
                    raise EOFError(f"audio stream ended: {detail}")
                raise EOFError("audio stream ended")
            buffer.extend(chunk)
        return bytes(buffer)

    def poll(self) -> int | None:
        return None if not self._process else self._process.poll()

    def stderr_text(self) -> str:
        if not self._process or not self._process.stderr or self._process.poll() is None:
            return ""
        try:
            return self._process.stderr.read().decode("utf-8", "ignore").strip()
        except Exception:
            return ""


def chunk_to_arrays(chunk: bytes, channel_count: int, audio_format: str) -> np.ndarray:
    samples = np.frombuffer(chunk, dtype=_AUDIO_DTYPE_BY_FORMAT[audio_format])
    if samples.size % channel_count != 0:
        raise ValueError("audio chunk is not aligned to whole frames")
    return samples.reshape((-1, channel_count))


def split_audio_channels(frames: np.ndarray, profile: AudioProfile) -> tuple[bytes, bytes, bytes]:
    raw_bytes = frames.tobytes()
    conference = frames[:, profile.conference_channel].tobytes()
    asr = frames[:, profile.asr_channel].tobytes()
    return raw_bytes, conference, asr


def run_audio_loop(
    stream_factory: Callable[[], ArecordStream],
    stop_event: threading.Event,
    on_chunk: Callable[[bytes, bytes, bytes, int], None],
    logger,
    reconnect_delay: float = 1.0,
    on_error: Callable[[Exception], None] | None = None,
) -> None:
    while not stop_event.is_set():
        stream = stream_factory()
        try:
            stream.start()
            logger.info("audio capture started")
            frame_index = 0
            byte_count = stream.chunk_frames * stream.channel_count * stream.bytes_per_sample
            while not stop_event.is_set():
                chunk = stream.read_exact(byte_count)
                on_chunk(chunk, stop_event, frame_index)
                frame_index += 1
                if stream.poll() is not None:
                    raise EOFError("arecord process exited")
        except Exception as exc:
            logger.error(f"audio capture failed: {exc}")
            stream.stop()
            if on_error is not None:
                try:
                    on_error(exc)
                except Exception as recovery_exc:
                    logger.error(f"audio recovery callback failed: {recovery_exc}")
            if not stop_event.is_set():
                time.sleep(reconnect_delay)
        else:
            stream.stop()
