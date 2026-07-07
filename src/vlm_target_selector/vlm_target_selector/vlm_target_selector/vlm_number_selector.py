#!/usr/bin/env python3
"""VLM helper for selecting a numbered candidate by target name."""

from __future__ import annotations

import base64
import re
from typing import Any

import cv2
import requests

from .bailian_vlm_client import BailianVLMClient


DEFAULT_NUMBER_PROMPT = (
    "画面中有多个带编号的物品，用户需要【{target_name}】，"
    "请只返回该物品的纯数字编号。不要输出多余内容。如果没有，请回复 0。"
)


class VlmNumberSelector:
    def __init__(
        self,
        use_local_vlm: bool,
        vlm_url: str,
        timeout: float,
        prompt_template: str = DEFAULT_NUMBER_PROMPT,
        bailian_client: BailianVLMClient | None = None,
    ) -> None:
        self.use_local_vlm = bool(use_local_vlm)
        self.vlm_url = vlm_url
        self.timeout = float(timeout)
        self.prompt_template = prompt_template or DEFAULT_NUMBER_PROMPT
        self.bailian_client = bailian_client

    def select_target_by_vlm(self, numbered_image_bgr, target_name: str, max_index: int) -> tuple[int, str]:
        prompt = self.prompt_template.format(target_name=target_name)
        raw_reply = self._ask_local(numbered_image_bgr, prompt) if self.use_local_vlm else self._ask_bailian(numbered_image_bgr, prompt)
        selected = parse_selected_index(raw_reply, max_index)
        return selected, raw_reply

    def _ask_local(self, image_bgr, prompt: str) -> str:
        ok, buffer = cv2.imencode(".jpg", image_bgr)
        if not ok:
            raise RuntimeError("failed to JPEG-encode numbered image")
        payload = {
            "prompt": prompt,
            "image": base64.b64encode(buffer).decode("ascii"),
            "image_name": "numbered_candidates.jpg",
        }
        response = requests.post(self.vlm_url, json=payload, timeout=self.timeout)
        if response.status_code != 200:
            raise RuntimeError(f"local VLM HTTP {response.status_code}: {response.text[:160]}")
        body = response.json()
        return str(body.get("ai_response") or body.get("response") or body.get("text") or "")

    def _ask_bailian(self, image_bgr, prompt: str) -> str:
        if self.bailian_client is None:
            raise RuntimeError("bailian_client is not configured")
        return self.bailian_client.select_target_id(image_bgr, prompt)


def parse_selected_index(raw_reply: Any, max_index: int) -> int:
    text = str(raw_reply or "").strip()
    match = re.search(r"\d+", text)
    if not match:
        return 0
    try:
        value = int(match.group(0))
    except Exception:
        return 0
    if value < 0 or value > int(max_index):
        return 0
    return value
