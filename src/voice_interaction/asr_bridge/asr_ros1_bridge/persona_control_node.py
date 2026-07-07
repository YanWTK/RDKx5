#!/usr/bin/env python3
"""Runtime voice persona control for the App."""

import json
import os
import requests
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


PERSONA_CONFIG_PATH = os.getenv(
    "VOICE_PERSONA_PATH",
    "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json",
)

PRESET_CONFIRM_TEXT = {
    "default": "已切换到默认助手风格",
    "calm_butler": "已切换到沉稳管家风格",
    "playful_partner": "小瓜切到活泼模式啦",
    "chatty_funny": "话痨模式启动，小瓜要多说两句了",
}


def _clean_text(value, limit=120):
    text = str(value or "").strip()
    text = " ".join(text.split())
    return text[:limit]


def _load_config(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            data.setdefault("profiles", {})
            return data
    except Exception:
        pass
    return {"active_profile": "default", "profiles": {}}


def _atomic_write_json(path, data):
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".voice_persona.", suffix=".json", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)


def _format_template(template, fallback, style):
    text = str(template or fallback)
    try:
        return text.format(style=style)
    except Exception:
        return fallback.format(style=style)


def _custom_profile(style, base=None, template=None):
    base = base if isinstance(base, dict) else {}
    template = template if isinstance(template, dict) else {}
    voice = str(base.get("tts_voice") or "Cherry")
    tts_style = _format_template(
        template.get("tts_style"),
        "{style}；适合家用取物机器人，发音清楚，语气自然。不要吓人，不要阴阳怪气，不要说脏话，不要承诺未完成的动作。",
        style,
    )
    task_instruction = _format_template(
        template.get("task_tts_instruction"),
        "tts_text 按这个自定义风格生成：{style}。必须保留任务事实，成功失败不能说反，中文短句，通常不超过30个汉字。",
        style,
    )
    phrase_instruction = _format_template(
        template.get("llm_phrase_instruction"),
        "{style}。保留事实，成功失败不能说反，不编造位置和物品，不要提系统细节，中文短句，通常不超过32个汉字。",
        style,
    )
    return {
        "description": str(template.get("description") or "App 自定义播报风格"),
        "tts_voice": voice,
        "tts_style": tts_style,
        "task_tts_instruction": task_instruction,
        "llm_phrase_instruction": phrase_instruction,
        "replacements": {},
        "wake_replies": base.get("wake_replies", []) if isinstance(base.get("wake_replies"), list) else [],
    }


def _app_options(data):
    profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
    configured = data.get("app_options", []) if isinstance(data, dict) else []
    options = []
    seen = set()
    if isinstance(configured, list):
        for item in configured:
            if not isinstance(item, dict):
                continue
            option_id = _clean_text(item.get("id") or item.get("profile") or "", limit=64)
            if not option_id or option_id in seen:
                continue
            option_type = _clean_text(item.get("type") or "preset", limit=16)
            if option_type != "custom" and option_id not in profiles:
                continue
            profile = profiles.get(option_id, {}) if isinstance(profiles, dict) else {}
            label = _clean_text(item.get("label") or profile.get("description") or option_id, limit=24)
            options.append({
                "id": option_id,
                "label": label,
                "type": "custom" if option_type == "custom" else "preset",
                "description": str(profile.get("description") or item.get("description") or ""),
            })
            seen.add(option_id)
    for option_id in ("default", "calm_butler", "playful_partner", "chatty_funny"):
        if option_id in seen or option_id not in profiles:
            continue
        profile = profiles.get(option_id) or {}
        options.append({
            "id": option_id,
            "label": str(profile.get("description") or option_id),
            "type": "preset",
            "description": str(profile.get("description") or ""),
        })
        seen.add(option_id)
    if "custom" not in seen:
        profile = profiles.get("custom", {}) if isinstance(profiles, dict) else {}
        options.append({
            "id": "custom",
            "label": "自定义",
            "type": "custom",
            "description": str(profile.get("description") or "App 自定义播报风格"),
        })
    return options


class PersonaControlNode(Node):
    def __init__(self):
        super().__init__("voice_persona_control")
        self.declare_parameter("persona_path", PERSONA_CONFIG_PATH)
        self.declare_parameter("set_topic", "/voice_persona/set")
        self.declare_parameter("status_topic", "/voice_persona/status")
        self.declare_parameter("tts_topic", "/tts_text")
        self.declare_parameter("announce_on_change", True)
        self.declare_parameter("custom_confirm_use_llm", True)
        self.declare_parameter("custom_confirm_model", "qwen3.6-flash")
        self.declare_parameter("custom_confirm_base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1")
        self.declare_parameter("custom_confirm_api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("custom_confirm_timeout_sec", 2.5)
        self.path = str(self.get_parameter("persona_path").value)
        set_topic = str(self.get_parameter("set_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        tts_topic = str(self.get_parameter("tts_topic").value)
        self._announce_on_change = _as_bool(self.get_parameter("announce_on_change").value)
        self._custom_confirm_use_llm = _as_bool(
            self.get_parameter("custom_confirm_use_llm").value
        )
        self._custom_confirm_model = str(self.get_parameter("custom_confirm_model").value)
        self._custom_confirm_base_url = str(
            self.get_parameter("custom_confirm_base_url").value
        )
        self._custom_confirm_api_key_env = str(
            self.get_parameter("custom_confirm_api_key_env").value
        )
        self._custom_confirm_timeout = float(
            self.get_parameter("custom_confirm_timeout_sec").value
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, status_topic, qos)
        self.tts_pub = self.create_publisher(String, tts_topic, 10)
        self.create_subscription(String, set_topic, self._on_set, 10)
        self._publish_status("ready")
        self.create_timer(2.0, self._publish_ready_status)
        self.get_logger().info(
            f"voice persona control started: set={set_topic} status={status_topic} "
            f"tts={tts_topic} path={self.path}"
        )

    def _publish_ready_status(self):
        self._publish_status("ready")

    def _on_set(self, msg):
        raw = str(msg.data or "").strip()
        if not raw:
            self._publish_status("error", reason="empty request")
            return
        try:
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                payload = {"profile": str(payload)}
        except Exception:
            payload = {"custom_style": raw}

        try:
            data = _load_config(self.path)
            profiles = data.setdefault("profiles", {})
            requested = _clean_text(
                payload.get("profile")
                or payload.get("active_profile")
                or payload.get("mode")
                or ""
            )
            custom_style = _clean_text(
                payload.get("custom_style")
                or payload.get("style")
                or payload.get("description")
                or ""
            )

            if custom_style:
                base_name = _clean_text(payload.get("base_profile") or data.get("active_profile") or "default")
                base = profiles.get(base_name) or profiles.get("default") or {}
                profiles["custom"] = _custom_profile(
                    custom_style,
                    base,
                    data.get("custom_profile_template"),
                )
                data["active_profile"] = "custom"
                changed = "custom"
            else:
                profile = requested or "default"
                if profile not in profiles:
                    self._publish_status("error", reason=f"unknown profile: {profile}")
                    return
                data["active_profile"] = profile
                changed = profile

            data["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            _atomic_write_json(self.path, data)
            self._publish_status("ok", active_profile=changed, custom_style=custom_style)
            self._announce_change(changed, custom_style)
        except Exception as exc:
            self.get_logger().warning(f"failed to set voice persona: {exc}")
            self._publish_status("error", reason=str(exc))

    def _publish_status(self, status, **extra):
        data = _load_config(self.path)
        payload = {
            "status": status,
            "active_profile": data.get("active_profile", "default"),
            "profiles": sorted((data.get("profiles") or {}).keys()),
            "app_options": _app_options(data),
        }
        payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _announce_change(self, profile, custom_style=""):
        if not self._announce_on_change:
            return
        text = ""
        if custom_style:
            text = self._custom_confirm_text(custom_style)
        else:
            text = PRESET_CONFIRM_TEXT.get(str(profile), f"已切换到{profile}风格")
        text = _clean_text(text, limit=48)
        if text:
            self.tts_pub.publish(String(data=text))
            self.get_logger().info(f"voice persona announce: {text}")

    def _custom_confirm_text(self, custom_style):
        fallback = "自定义风格已切换"
        if not self._custom_confirm_use_llm:
            return fallback
        api_key = os.getenv(self._custom_confirm_api_key_env, "").strip()
        if not api_key:
            return fallback
        try:
            prompt = (
                "你是家庭服务机器人小瓜。用户刚刚把你的播报风格切换为："
                f"{custom_style}\n"
                "请生成一句符合这个风格的切换成功确认播报。"
                "只能输出一句中文，不要解释，不要 Markdown，不要提系统细节。"
                "不要承诺执行任何任务。长度控制在 8 到 28 个汉字。"
            )
            reply = self._call_bailian_confirm(prompt, api_key)
            text = _clean_text(reply, limit=48).strip("`\"'“”‘’ ")
            return text or fallback
        except Exception as exc:
            self.get_logger().warning(f"custom persona confirm LLM failed: {exc}")
            return fallback

    def _call_bailian_confirm(self, prompt, api_key):
        base_url = self._custom_confirm_base_url.rstrip("/")
        url = f"{base_url}/chat/completions"
        response = requests.post(
            url,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            json={
                "model": self._custom_confirm_model,
                "messages": [{"role": "user", "content": prompt}],
                "temperature": 0.8,
                "chat_template_kwargs": {"enable_thinking": False},
            },
            timeout=self._custom_confirm_timeout,
        )
        if response.status_code < 200 or response.status_code >= 300:
            raise RuntimeError(f"HTTP {response.status_code}: {response.text[:160]}")
        body = response.json()
        choices = body.get("choices")
        if isinstance(choices, list) and choices:
            message = choices[0].get("message") if isinstance(choices[0], dict) else None
            if isinstance(message, dict):
                return str(message.get("content") or "")
        return str(
            body.get("text")
            or body.get("output")
            or body.get("response")
            or ""
        )


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def main():
    rclpy.init()
    node = PersonaControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
