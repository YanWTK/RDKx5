"""Bailian Qwen3-VL client helpers for VLM target selection.

Reads API key from environment by default:
- DASHSCOPE_API_KEY
- DASHSCOPE_BASE_URL (optional)
"""

from __future__ import annotations

import base64
import os
from typing import Callable

import cv2

from .direct_bailian_client import DirectBailianClient, DirectBailianError


DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = os.getenv("BAILIAN_MODEL", "qwen3-vl-plus")


class BailianVLMError(RuntimeError):
    """Base error for Bailian VLM calls."""


class MissingAPIKeyError(BailianVLMError):
    """Raised when the configured API key environment variable is missing."""


class BailianVLMRequestError(BailianVLMError):
    """Raised on network/API/request errors."""


class BailianVLMResponseError(BailianVLMError):
    """Raised when Bailian returns an empty or malformed response."""


class BailianVLMClient:
    """OpenAI-compatible client for Qwen3-VL-Flash on Bailian."""

    def __init__(
        self,
        api_key: str | None = None,
        api_key_env: str = "DASHSCOPE_API_KEY",
        base_url: str | None = None,
        model: str = DEFAULT_MODEL,
        timeout: float = 20.0,
        enable_thinking: bool = False,
        component: str = "bailian_vlm",
        log_callback: Callable[[str], None] | None = None,
    ) -> None:
        key_env = (api_key_env or "DASHSCOPE_API_KEY").strip()
        resolved_api_key = (api_key or os.getenv(key_env) or "").strip()
        if not resolved_api_key:
            raise MissingAPIKeyError(
                f"Missing {key_env}. Set it first, for example: "
                f'export {key_env}="your_api_key_here"'
            )

        resolved_base_url = (
            base_url or os.getenv("DASHSCOPE_BASE_URL") or DEFAULT_BASE_URL
        ).strip()

        self.model = model
        self.base_url = resolved_base_url
        self.enable_thinking = bool(enable_thinking)
        self._client = DirectBailianClient(
            api_key=resolved_api_key,
            base_url=resolved_base_url,
            timeout=timeout,
            component=component,
            log_callback=log_callback or (lambda _message: None),
        )

    def select_target_id(self, image_bgr, prompt: str) -> str:
        ok, buffer = cv2.imencode(".jpg", image_bgr)
        if not ok:
            raise BailianVLMRequestError("failed to JPEG-encode prompt image")

        image_base64 = base64.b64encode(buffer).decode("ascii")
        try:
            return self._client.chat_completion(
                model=self.model,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                },
                            },
                        ],
                    }
                ],
                temperature=0,
                # DashScope OpenAI-compatible endpoint requires nesting under
                # chat_template_kwargs for enable_thinking to take effect;
                # a flat extra_body={"enable_thinking": ...} is silently
                # ignored, leaving qwen3-vl-plus in default thinking mode.
                extra_body={"chat_template_kwargs": {"enable_thinking": self.enable_thinking}},
                payload_kind="jpeg_base64",
                payload_bytes=len(image_base64),
            )
        except DirectBailianError as exc:
            raise BailianVLMRequestError(f"Unexpected Bailian VLM failure: {exc}") from exc
