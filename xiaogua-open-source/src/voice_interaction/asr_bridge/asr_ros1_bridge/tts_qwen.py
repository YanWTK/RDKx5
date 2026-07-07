"""Qwen3-TTS Realtime 引擎 (WebSocket 流式合成)"""

import base64
import os
import threading
import time

import dashscope
import websocket
from dashscope.audio.qwen_tts_realtime import (
    AudioFormat,
    QwenTtsRealtime,
    QwenTtsRealtimeCallback,
)

QWEN_MODEL = os.getenv("QWEN_TTS_MODEL", "qwen3-tts-instruct-flash-realtime")
QWEN_VOICE = os.getenv("QWEN_TTS_VOICE", "Cherry")
QWEN_MODE = os.getenv("QWEN_TTS_MODE", "server_commit")
QWEN_CONNECT_TIMEOUT_SEC = float(os.getenv("QWEN_TTS_CONNECT_TIMEOUT_SEC", "10"))


class _QwenTtsRealtimeWithTimeout(QwenTtsRealtime):
    def connect(self) -> None:
        self.ws = websocket.WebSocketApp(
            self.url,
            header=self._get_websocket_header(),
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        self.thread = threading.Thread(target=self.ws.run_forever)
        self.thread.daemon = True
        self.thread.start()

        start_time = time.time()
        while (
            not (self.ws.sock and self.ws.sock.connected)
            and (time.time() - start_time) < QWEN_CONNECT_TIMEOUT_SEC
        ):
            time.sleep(0.1)
        if not (self.ws.sock and self.ws.sock.connected):
            raise TimeoutError(
                f"websocket connection could not established within "
                f"{QWEN_CONNECT_TIMEOUT_SEC:g}s. Please check your network "
                "connection, firewall settings, or server status."
            )
        self.callback.on_open()


class _Callback(QwenTtsRealtimeCallback):
    def __init__(self):
        self._buf = bytearray()
        self._done = threading.Event()
        self._err = None

    def on_open(self) -> None:
        pass

    def on_close(self, close_status_code, close_msg) -> None:
        self._done.set()

    def on_event(self, response: str) -> None:
        try:
            t = response.get("type", "")
            if t == "response.audio.delta":
                chunk = response.get("delta", "")
                if chunk:
                    self._buf.extend(base64.b64decode(chunk))
            elif t == "response.done":
                self._done.set()
            elif t == "session.finished":
                self._done.set()
        except Exception as e:
            self._err = e
            self._done.set()

    def wait(self, timeout=None):
        result = self._done.wait(timeout)
        if self._err:
            raise self._err
        return result


class TTSQwenEngine:
    def synthesize(self, text: str, voice: str = "", style: str = "") -> bytes:
        api_key = os.getenv("DASHSCOPE_API_KEY", "")
        if not api_key:
            raise RuntimeError("DASHSCOPE_API_KEY not set")
        dashscope.api_key = api_key

        voice = voice or QWEN_VOICE
        cb = _Callback()
        conn = _QwenTtsRealtimeWithTimeout(model=QWEN_MODEL, callback=cb)
        try:
            conn.connect()
            params = dict(
                voice=voice,
                response_format=AudioFormat.PCM_24000HZ_MONO_16BIT,
                mode=QWEN_MODE,
            )
            if style:
                params["instructions"] = style
            conn.update_session(**params)
            conn.append_text(text)
            conn.finish()
            if not cb.wait(timeout=30):
                raise TimeoutError("Qwen TTS timeout waiting for audio")
            if cb._err:
                raise cb._err
            if not cb._buf:
                raise RuntimeError("Qwen TTS returned no audio")
            return bytes(cb._buf)
        finally:
            try:
                conn.close()
            except Exception:
                pass
