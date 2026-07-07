"""Simple rule-based robot command parser."""

from __future__ import annotations

import re


def _normalize_text(text: str) -> str:
    """Remove whitespace and punctuation to improve matching."""

    cleaned = text.strip()
    return re.sub(r"[，。！？,!.?；;:：、\s]+", "", cleaned)


def parse_robot_command(text: str) -> dict:
    """Parse ASR text into a structured command dictionary."""

    raw_text = text.strip()
    normalized = _normalize_text(raw_text)

    if not normalized:
        return {"raw_text": raw_text, "intent": "unknown", "confidence": 0.0}

    if any(phrase in normalized for phrase in ("停下", "停止", "别动")):
        return {"raw_text": raw_text, "intent": "stop", "confidence": 0.95}

    if any(phrase in normalized for phrase in ("过来", "到我这里来")):
        return {"raw_text": raw_text, "intent": "move_to_user", "confidence": 0.9}

    if any(phrase in normalized for phrase in ("跟着我", "跟随我")):
        return {"raw_text": raw_text, "intent": "follow_me", "confidence": 0.9}

    if any(phrase in normalized for phrase in ("回充电桩", "回去充电", "去充电")):
        return {"raw_text": raw_text, "intent": "go_charge", "confidence": 0.9}

    target_match = re.search(r"去(客厅|卧室)", normalized)
    if target_match:
        return {
            "raw_text": raw_text,
            "intent": "navigate_to",
            "target": target_match.group(1),
            "confidence": 0.88,
        }

    if any(phrase in normalized for phrase in ("看看前面有什么", "前面有什么")):
        return {"raw_text": raw_text, "intent": "look_front", "confidence": 0.85}

    return {"raw_text": raw_text, "intent": "unknown", "confidence": 0.1}
