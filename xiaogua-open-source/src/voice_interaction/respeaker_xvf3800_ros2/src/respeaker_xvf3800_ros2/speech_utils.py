"""Speech processing helpers for XVF3800 audio nodes."""

from __future__ import annotations

import re
import wave
from pathlib import Path

import numpy as np


_AUDIO_DTYPE_BY_FORMAT = {
    "S16_LE": np.dtype("<i2"),
    "S32_LE": np.dtype("<i4"),
}


def normalize_text(text: str) -> str:
    """Normalize ASR text for phrase matching."""

    return re.sub(r"[\s\W_]+", "", text).casefold()


def match_phrase(text: str, phrases: list[str]) -> str | None:
    """Return the first normalized phrase that appears in the text."""

    normalized_text = normalize_text(text)
    if not normalized_text:
        return None

    for phrase in phrases:
        normalized_phrase = normalize_text(phrase)
        if not normalized_phrase:
            continue
        if normalized_phrase in normalized_text:
            return phrase
    return None


def pcm_to_s16le_bytes(pcm: bytes, audio_format: str) -> bytes:
    """Convert PCM bytes to signed 16-bit little-endian bytes."""

    if audio_format == "S16_LE":
        return pcm
    if audio_format == "S32_LE":
        samples = np.frombuffer(pcm, dtype="<i4")
        samples = np.clip(samples >> 16, -32768, 32767).astype("<i2")
        return samples.tobytes()
    raise ValueError(f"Unsupported audio format: {audio_format}")


def normalize_pcm(pcm_bytes: bytes, peak: int = 24000, max_gain: float = 8.0) -> bytes:
    """Boost quiet mono PCM to a usable level."""

    if not pcm_bytes:
        return pcm_bytes

    samples = np.frombuffer(pcm_bytes, dtype="<i2").astype(np.float32)
    if samples.size == 0:
        return pcm_bytes

    current_peak = float(np.max(np.abs(samples)))
    if current_peak <= 0.0:
        return pcm_bytes

    gain = min(float(peak) / current_peak, max_gain)
    if gain <= 1.0:
        return pcm_bytes

    normalized = np.clip(samples * gain, -32768, 32767).astype("<i2")
    return normalized.tobytes()


def tail_pcm(pcm_bytes: bytes, sample_rate: int, channels: int, sample_width: int, seconds: float) -> bytes:
    """Return the last ``seconds`` of PCM bytes."""

    if seconds <= 0:
        return pcm_bytes

    bytes_per_second = sample_rate * channels * sample_width
    tail_bytes = int(bytes_per_second * seconds)
    if tail_bytes <= 0:
        return pcm_bytes
    return pcm_bytes[-tail_bytes:]


def should_end_recording(
    now: float,
    start_time: float,
    voice_seen: bool,
    vad_active: bool,
    last_voice_time: float | None,
    max_duration: float,
    silence_timeout: float,
) -> bool:
    """Return True when a live recording should stop."""

    if max_duration <= 0:
        return True
    if now - start_time >= max_duration:
        return True
    if voice_seen and not vad_active and last_voice_time is not None and now - last_voice_time >= silence_timeout:
        return True
    return False


def write_wav_file(path: Path, pcm_bytes: bytes, sample_rate: int, channels: int, sample_width: int) -> None:
    """Write PCM bytes to a WAV file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm_bytes)
