"""Bailian qwen3-asr-flash-2026-02-10 client helpers.

This module keeps the API key out of source control. It reads:
- DASHSCOPE_API_KEY
- DASHSCOPE_BASE_URL (optional)
"""

from __future__ import annotations

import base64
import os
import time
from pathlib import Path
from typing import Callable

from .direct_bailian_client import DirectBailianClient, DirectBailianError


DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = "qwen3-asr-flash"


class ASRClientError(RuntimeError):
    """Base ASR client error."""


class MissingAPIKeyError(ASRClientError):
    """Raised when DASHSCOPE_API_KEY is missing."""


class ASRRequestError(ASRClientError):
    """Raised on request/network/API parameter errors."""


class ASRResponseError(ASRClientError):
    """Raised when the ASR response is empty or malformed."""


class BailianASRClient:
    """qwen3-asr-flash-2026-02-10 wrapper."""

    # Rate limiting: minimum seconds between requests
    _last_request_time: float = 0.0
    _min_interval: float = 1.0  # Minimum 1 second between requests

    def __init__(
        self,
        api_key: str | None = None,
        base_url: str | None = None,
        model: str = DEFAULT_MODEL,
        log_callback: Callable[[str], None] | None = None,
    ) -> None:
        resolved_api_key = (api_key or os.getenv("DASHSCOPE_API_KEY") or "").strip()
        if not resolved_api_key:
            raise MissingAPIKeyError(
                "Missing DASHSCOPE_API_KEY. Set it first, for example: "
                'export DASHSCOPE_API_KEY="your_api_key_here"'
            )

        resolved_base_url = (
            base_url or os.getenv("DASHSCOPE_BASE_URL") or DEFAULT_BASE_URL
        ).strip()

        self.model = model
        self.base_url = resolved_base_url
        self._log_callback = log_callback
        self._client = DirectBailianClient(
            api_key=resolved_api_key,
            base_url=resolved_base_url,
            component="asr",
            log_callback=log_callback or (lambda _message: None),
        )

    def transcribe_wav(self, wav_path: str, language: str = "zh") -> str:
        """Send a WAV file to Bailian ASR and return the transcribed text."""

        # Rate limiting: wait if needed
        now = time.monotonic()
        elapsed = now - BailianASRClient._last_request_time
        if elapsed < BailianASRClient._min_interval:
            wait_sec = BailianASRClient._min_interval - elapsed
            if self._log_callback is not None:
                self._log_callback(
                    f"API_TIMING event=local_rate_limit component=asr wait_sec={wait_sec:.3f}"
                )
            time.sleep(wait_sec)

        audio_path = Path(wav_path).expanduser()
        if not audio_path.exists():
            raise ASRRequestError(f"Audio file does not exist: {audio_path}")
        if audio_path.stat().st_size == 0:
            raise ASRRequestError(f"Audio file is empty: {audio_path}")

        try:
            audio_base64 = base64.b64encode(audio_path.read_bytes()).decode("ascii")
            audio_data_url = f"data:audio/wav;base64,{audio_base64}"
        except OSError as exc:
            raise ASRRequestError(f"Failed to read audio file: {audio_path}: {exc}") from exc

        try:
            text = self._client.chat_completion(
                model=self.model,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_audio",
                                "input_audio": {
                                    "data": audio_data_url,
                                    "format": "wav",
                                },
                            }
                        ],
                    }
                ],
                extra_body={
                    "asr_options": {
                        "language": language,
                        "enable_itn": True,
                    }
                },
                payload_bytes=len(audio_base64),
            )
        except DirectBailianError as exc:
            raise ASRRequestError(f"Bailian ASR request failed: {exc}") from exc

        # Update last request time
        BailianASRClient._last_request_time = time.monotonic()

        if not text:
            raise ASRResponseError("ASR returned an empty text result.")
        return text
