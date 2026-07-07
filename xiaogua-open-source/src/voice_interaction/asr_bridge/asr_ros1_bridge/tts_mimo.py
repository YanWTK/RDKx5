"""MiMo TTS 引擎 (HTTP 同步合成)"""

import base64
import os

import requests

MIMO_API_KEY = os.getenv("MIMO_API_KEY", "")
MIMO_API_URL = os.getenv("MIMO_API_URL", "https://api.xiaomimimo.com/v1/chat/completions")
MIMO_MODEL = os.getenv("MIMO_MODEL", "mimo-v2.5-tts")
MIMO_VOICE = os.getenv("MIMO_VOICE", "冰糖")
MIMO_STYLE = os.getenv("MIMO_STYLE", "温柔活泼，语速适中，像在和朋友聊天")


class TTSMimoEngine:
    def synthesize(self, text: str, voice: str = "", style: str = "") -> bytes:
        voice = voice or MIMO_VOICE
        style = style or MIMO_STYLE
        resp = requests.post(
            MIMO_API_URL,
            headers={"api-key": MIMO_API_KEY, "Content-Type": "application/json"},
            json={
                "model": MIMO_MODEL,
                "messages": [
                    {"role": "user", "content": style},
                    {"role": "assistant", "content": text},
                ],
                "audio": {"format": "pcm16", "voice": voice},
            },
            timeout=30,
        )
        return base64.b64decode(resp.json()["choices"][0]["message"]["audio"]["data"])
