#!/usr/bin/env python3
"""宿主机TTS节点 - 直接播放音频，避免容器设备冲突"""

import os
import json
import threading
import tempfile
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# 加载配置 (必须在引擎 import 之前，引擎模块级变量依赖 env)
_config_path = "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/tts_mimo.env"
if os.path.exists(_config_path):
    with open(_config_path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("export ") and "=" in line:
                k, v = line[7:].split("=", 1)
                os.environ[k.strip()] = v.strip().strip('"').strip("'")

# TTS 后端引擎 (env 加载后 import，确保模块级变量正确)
TTS_BACKEND = os.getenv("TTS_BACKEND", "qwen").lower()
if TTS_BACKEND == "qwen":
    from .tts_qwen import TTSQwenEngine as _TTSEngine
else:
    from .tts_mimo import TTSMimoEngine as _TTSEngine

_engine = _TTSEngine()

MIMO_VOICE = os.getenv("MIMO_VOICE", "冰糖")
MIMO_STYLE = os.getenv("MIMO_STYLE", "温柔活泼，语速适中，像在和朋友聊天")
_DEFAULT_VOICE = os.getenv("QWEN_TTS_VOICE", "Cherry") if TTS_BACKEND == "qwen" else MIMO_VOICE
PLAY_CMD = os.getenv("TTS_PLAY_CMD", "aplay -q -D plughw:C16K6Ch,0 -f S16_LE -r 24000 -c 1 {file}")
LOCAL_DIR = "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config"
PERSONA_CONFIG_PATH = os.getenv(
    "VOICE_PERSONA_PATH",
    "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json",
)
PERSONA_PROFILE = os.getenv("VOICE_PERSONA_PROFILE", "")


class PersonaConfig:
    def __init__(self, path, profile_name=""):
        self.path = path
        self.profile_name = profile_name
        self._mtime = None
        self._profile = {}

    def profile(self):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            self._mtime = None
            self._profile = {}
            return {}

        if self._mtime == mtime:
            return self._profile

        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
            profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
            requested = self.profile_name or str(data.get("active_profile") or "default")
            profile = profiles.get(requested) or profiles.get("default") or {}
            if not isinstance(profile, dict):
                profile = {}
            self._profile = profile
            self._mtime = mtime
        except Exception:
            self._profile = {}
            self._mtime = mtime
        return self._profile


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_host_node')
        self.sub = self.create_subscription(String, '/tts_text', self.callback, 10)
        self.playing_pub = self.create_publisher(Bool, '/tts_playing', 10)
        self._play_lock = threading.Lock()
        self._persona = PersonaConfig(PERSONA_CONFIG_PATH, PERSONA_PROFILE)
        self._publish_playing(False)
        self.get_logger().info(f'TTS Host Node 启动, 后端: {TTS_BACKEND}, 默认音色: {_DEFAULT_VOICE}')

    def _publish_playing(self, playing: bool) -> None:
        msg = Bool()
        msg.data = bool(playing)
        self.playing_pub.publish(msg)

    def callback(self, msg):
        text = msg.data.strip()
        if not text:
            return
        threading.Thread(target=self._play, args=(text,), daemon=True).start()

    def _play(self, text):
        with self._play_lock:
            self._publish_playing(True)
            if text.startswith("__local__:"):
                try:
                    filename = text[len("__local__:"):]
                    path = os.path.join(LOCAL_DIR, filename)
                    if os.path.exists(path):
                        self.get_logger().info(f'播放本地: {filename}')
                        result = subprocess.run(PLAY_CMD.format(file=path), shell=True, timeout=30, capture_output=True, text=True)
                        if result.returncode != 0:
                            self.get_logger().error(f'播放失败: {result.stderr}')
                    else:
                        self.get_logger().warning(f'本地音频不存在: {path}')
                finally:
                    self._publish_playing(False)
                return

            self.get_logger().info(f'合成: {text}')
            tmp_path = None
            try:
                persona = self._persona.profile()
                tts_style = str(persona.get("tts_style") or MIMO_STYLE)
                tts_voice = str(persona.get("tts_voice") or MIMO_VOICE)
                audio = _engine.synthesize(text, voice=tts_voice, style=tts_style)
                with tempfile.NamedTemporaryFile(suffix=".pcm", delete=False) as f:
                    f.write(audio)
                    tmp_path = f.name
                result = subprocess.run(PLAY_CMD.format(file=tmp_path), shell=True, timeout=30, capture_output=True, text=True)
                if result.returncode != 0:
                    self.get_logger().error(f'播放失败: {result.stderr}')
                self.get_logger().info('播放完成')
            except Exception as e:
                self.get_logger().error(f'TTS失败: {e}')
            finally:
                if tmp_path and os.path.exists(tmp_path):
                    os.unlink(tmp_path)
                self._publish_playing(False)


def main():
    rclpy.init()
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
