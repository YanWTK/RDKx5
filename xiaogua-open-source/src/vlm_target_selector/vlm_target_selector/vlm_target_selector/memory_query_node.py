#!/usr/bin/env python3
"""Query patrol object memory with a natural-language command."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String

from .bailian_vlm_client import DEFAULT_BASE_URL as BAILIAN_DEFAULT_BASE_URL
from .direct_bailian_client import DirectBailianClient


BEVERAGE_TERMS = [
    "渴",
    "喝",
    "饮料",
    "水",
    "茶",
    "冰红茶",
    "绿茶",
    "可乐",
    "咖啡",
    "奶",
    "瓶",
    "杯",
]

BEVERAGE_CLASSES = {"bottle", "cup"}

DEFAULT_MEMORY_SELECT_PROMPT = """你是家庭服务机器人的巡逻记忆检索器。
机器人已经在巡逻时用 YOLO + VLM 记住了一批物品，每个候选都有 id、名称、可能名称、YOLO 类别和 point_id。
你的任务是根据用户原话和任务理解结果，从候选里选择最应该去找的一个物品。

只允许输出一个 JSON 对象，不要输出 Markdown，不要解释。

输出格式：
{
  "target_id": "候选 id；如果没有合适目标则为空字符串",
  "target_name": "用于后续视觉确认的目标名称",
  "confidence": 0.0,
  "reason": "一句中文理由"
}

选择原则：
1. target_name 和 semantic_hint 是任务理解结果，优先级高于用户原话中的泛化表达。
1a. 如果任务理解或用户原话指定了 source_location/source_point_id，候选已经按该地点过滤；只能在这些候选中选择，不要选择其他地点的物品。
2. 用户说“我渴了”“喝的”“饮料”时，优先选择 bottle 或 cup，且名称/可能名称更像饮料的候选。
3. 用户明确说“冰红茶”“水”“杯子”“遥控器”等具体物品时，优先匹配名称或 possible_names。
4. 允许近似记忆匹配：如果没有完美匹配，但候选是用户目标的上位类、同义类或强相关物品，可以选择该候选并在 reason 里说明不确定；例如用户找“卡皮巴拉玩偶”，记忆库只有“毛绒玩具”，可以选择“毛绒玩具”，因为它可能就是卡皮巴拉玩偶。
5. 如果多个候选都可能，选择最接近用户需求且 point_id 不为空的候选。
6. 如果没有合理候选，target_id 输出空字符串，confidence 输出 0。

用户原话：
{user_command}

任务理解：
{task_understanding_json}

候选记忆：
{candidates_json}

输出："""


class ObjectMemoryQueryNode(Node):
    def __init__(self) -> None:
        super().__init__("object_memory_query")

        self.declare_parameter("memory_path", "")
        self.declare_parameter("query_topic", "/object_memory/query")
        self.declare_parameter("result_topic", "/object_memory/query_result")
        self.declare_parameter("min_score", 10.0)
        self.declare_parameter("max_candidates", 5)
        self.declare_parameter("max_memory_candidates", 50)
        self.declare_parameter("use_llm_selection", True)
        self.declare_parameter("allow_rule_fallback", True)
        self.declare_parameter("use_local_llm", True)
        self.declare_parameter("llm_url", "http://127.0.0.1:8000/analyze")
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("prompt_template", DEFAULT_MEMORY_SELECT_PROMPT)
        self.declare_parameter("bailian_model", "qwen3.6-flash")
        self.declare_parameter("bailian_base_url", "")
        self.declare_parameter("bailian_api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("bailian_enable_thinking", False)

        memory_path = str(self.get_parameter("memory_path").value).strip()
        if not memory_path:
            memory_path = str(Path.home() / ".ros" / "patrol_memory" / "object_memory.json")
        self._memory_path = Path(memory_path).expanduser()
        self._min_score = float(self.get_parameter("min_score").value)
        self._max_candidates = max(1, int(self.get_parameter("max_candidates").value))
        self._max_memory_candidates = max(
            1, int(self.get_parameter("max_memory_candidates").value)
        )
        self._use_llm_selection = _as_bool(self.get_parameter("use_llm_selection").value)
        self._allow_rule_fallback = _as_bool(
            self.get_parameter("allow_rule_fallback").value
        )
        self._use_local_llm = _as_bool(self.get_parameter("use_local_llm").value)
        self._llm_url = str(self.get_parameter("llm_url").value)
        self._timeout = float(self.get_parameter("request_timeout_sec").value)
        self._prompt_template = str(self.get_parameter("prompt_template").value)
        self._bailian_model = str(self.get_parameter("bailian_model").value)
        self._bailian_base_url = (
            str(self.get_parameter("bailian_base_url").value).strip()
            or BAILIAN_DEFAULT_BASE_URL
        )
        self._bailian_api_key_env = str(self.get_parameter("bailian_api_key_env").value)
        self._bailian_enable_thinking = _as_bool(
            self.get_parameter("bailian_enable_thinking").value
        )
        self._bailian_client = None

        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("result_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("query_topic").value),
            self._on_query,
            10,
        )

        self.get_logger().info(
            "object_memory_query started | "
            f"memory={self._memory_path} | "
            f"selection={'llm' if self._use_llm_selection else 'rule'}"
        )

    def _on_query(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError("query payload must be an object")
        except Exception as exc:
            self._publish({"success": False, "reason": f"invalid JSON: {exc}"})
            return

        request_id = str(cmd.get("request_id") or "").strip()
        query = str(cmd.get("query") or cmd.get("user_command") or "").strip()
        target_name = str(cmd.get("target_name") or "").strip()
        semantic_hint = str(cmd.get("semantic_hint") or "").strip()
        source_location = str(cmd.get("source_location") or "").strip()
        source_point_id = str(cmd.get("source_point_id") or "").strip()
        if not query:
            self._publish({
                "success": False,
                "request_id": request_id,
                "reason": "query is empty",
            })
            return

        try:
            objects = self._load_memory()
        except Exception as exc:
            self._publish({
                "success": False,
                "request_id": request_id,
                "query": query,
                "reason": str(exc),
            })
            return

        original_count = len(objects)
        if source_point_id:
            objects = [
                obj for obj in objects
                if isinstance(obj, dict)
                and str(obj.get("point_id") or obj.get("detected_at_point") or "").strip() == source_point_id
            ]
            self.get_logger().info(
                f"source location constraint: location={source_location or '<empty>'} "
                f"point_id={source_point_id} candidates={len(objects)}/{original_count}"
            )
            if not objects:
                self._publish({
                    "success": False,
                    "request_id": request_id,
                    "query": query,
                    "target_name": target_name,
                    "semantic_hint": semantic_hint,
                    "source_location": source_location,
                    "source_point_id": source_point_id,
                    "reason": f"specified source location has no remembered objects: {source_location or source_point_id}",
                    "selection_method": "source_filter",
                    "candidates": [],
                })
                return

        if self._use_llm_selection:
            try:
                result = self._select_with_llm(cmd, query, target_name, semantic_hint, objects)
                if result is not None:
                    self._publish(result)
                    return
            except Exception as exc:
                self.get_logger().warn(f"LLM memory selection failed: {exc}")
                if not self._allow_rule_fallback:
                    self._publish({
                        "success": False,
                        "request_id": request_id,
                        "query": query,
                        "target_name": target_name,
                        "semantic_hint": semantic_hint,
                        "source_location": source_location,
                        "source_point_id": source_point_id,
                        "reason": f"LLM memory selection failed: {exc}",
                        "selection_method": "llm",
                    })
                    return

        if not self._allow_rule_fallback and self._use_llm_selection:
            self._publish({
                "success": False,
                "request_id": request_id,
                "query": query,
                "target_name": target_name,
                "semantic_hint": semantic_hint,
                "source_location": source_location,
                "source_point_id": source_point_id,
                "reason": "LLM did not select a remembered object",
                "selection_method": "llm",
            })
            return

        result = self._select_with_rule_fallback(
            request_id,
            query,
            target_name,
            semantic_hint,
            objects,
        )
        self._publish(result)

    def _select_with_llm(
        self,
        cmd: dict[str, Any],
        query: str,
        target_name: str,
        semantic_hint: str,
        objects: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        request_id = str(cmd.get("request_id") or "").strip()
        compact_candidates = _compact_candidates(objects, self._max_memory_candidates)
        if not compact_candidates:
            return {
                "success": False,
                "request_id": request_id,
                "query": query,
                "target_name": target_name,
                "semantic_hint": semantic_hint,
                "reason": "object memory is empty",
                "selection_method": "llm",
                "candidates": [],
            }

        task_understanding = cmd.get("task_understanding")
        if not isinstance(task_understanding, dict):
            task_understanding = {
                "target_name": target_name,
                "semantic_hint": semantic_hint,
            }
        prompt = self._render_prompt(query, task_understanding, compact_candidates)
        raw_reply = self._ask_local_llm(prompt) if self._use_local_llm else self._ask_bailian(prompt)
        selected = _parse_json_object(raw_reply)
        if selected is None:
            raise ValueError(f"LLM did not return JSON: {raw_reply[:200]!r}")

        selected_id = str(selected.get("target_id") or "").strip()
        if not selected_id:
            return {
                "success": False,
                "request_id": request_id,
                "query": query,
                "target_name": target_name,
                "semantic_hint": semantic_hint,
                "reason": str(selected.get("reason") or "LLM selected no target"),
                "selection_method": "llm",
                "raw_reply": raw_reply,
                "candidates": compact_candidates[: self._max_candidates],
            }

        target_obj = None
        for obj in objects:
            if isinstance(obj, dict) and str(obj.get("id") or "") == selected_id:
                target_obj = obj
                break
        if target_obj is None:
            raise ValueError(f"LLM selected unknown target_id: {selected_id}")

        selected_name = target_name or _resolve_target_name(query, target_obj)
        confidence = _safe_float(selected.get("confidence"), 0.0)
        reason = str(selected.get("reason") or "").strip()
        return {
            "success": True,
            "request_id": request_id,
            "query": query,
            "target_id": selected_id,
            "target_name": selected_name,
            "point_id": str(target_obj.get("point_id") or ""),
            "target_obj": target_obj,
            "score": confidence,
            "reasons": [reason] if reason else ["LLM selected target from memory"],
            "selection_method": "llm",
            "raw_reply": raw_reply,
            "candidates": compact_candidates[: self._max_candidates],
        }

    def _select_with_rule_fallback(
        self,
        request_id: str,
        query: str,
        target_name: str,
        semantic_hint: str,
        objects: list[dict[str, Any]],
    ) -> dict[str, Any]:
        scored = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            score, reasons = _score_object(query, obj, target_name, semantic_hint)
            if score <= 0.0:
                continue
            scored.append((score, obj, reasons))

        scored.sort(
            key=lambda item: (
                item[0],
                str(item[1].get("point_id") or ""),
                str(item[1].get("id") or ""),
            ),
            reverse=True,
        )

        candidates = [
            {
                "target_id": str(obj.get("id") or ""),
                "target_name": _resolve_target_name(query, obj),
                "point_id": str(obj.get("point_id") or ""),
                "main_name": str(obj.get("main_name") or ""),
                "yolo_class": str(obj.get("yolo_class") or ""),
                "score": round(score, 3),
                "reasons": reasons,
            }
            for score, obj, reasons in scored[: self._max_candidates]
        ]

        if not scored or scored[0][0] < self._min_score:
            return {
                "success": False,
                "request_id": request_id,
                "query": query,
                "reason": "no remembered object matches the query",
                "selection_method": "rule_fallback",
                "candidates": candidates,
            }

        best_score, best_obj, best_reasons = scored[0]
        if not target_name:
            target_name = _resolve_target_name(query, best_obj)
        return {
            "success": True,
            "request_id": request_id,
            "query": query,
            "target_id": str(best_obj.get("id") or ""),
            "target_name": target_name,
            "point_id": str(best_obj.get("point_id") or ""),
            "target_obj": best_obj,
            "score": round(best_score, 3),
            "reasons": best_reasons,
            "selection_method": "rule_fallback",
            "candidates": candidates,
        }

    def _render_prompt(
        self,
        query: str,
        task_understanding: dict[str, Any],
        candidates: list[dict[str, Any]],
    ) -> str:
        return (
            self._prompt_template
            .replace("{user_command}", query)
            .replace(
                "{task_understanding_json}",
                json.dumps(task_understanding, ensure_ascii=False, indent=2),
            )
            .replace(
                "{candidates_json}",
                json.dumps(candidates, ensure_ascii=False, indent=2),
            )
        )

    def _ask_local_llm(self, prompt: str) -> str:
        response = requests.post(
            self._llm_url,
            json={"prompt": prompt},
            timeout=self._timeout,
        )
        if response.status_code != 200:
            raise RuntimeError(f"local LLM HTTP {response.status_code}: {response.text[:160]}")
        body = response.json()
        return str(body.get("ai_response") or body.get("response") or body.get("text") or "")

    def _ask_bailian(self, prompt: str) -> str:
        if self._bailian_client is None:
            api_key = os.getenv(self._bailian_api_key_env, "").strip()
            if not api_key:
                raise RuntimeError(f"missing {self._bailian_api_key_env}")
            self._bailian_client = DirectBailianClient(
                api_key=api_key,
                base_url=self._bailian_base_url,
                timeout=self._timeout,
                component="object_memory_query",
                log_callback=self.get_logger().info,
            )
        return self._bailian_client.chat_completion(
            model=self._bailian_model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0,
            # DashScope OpenAI-compatible endpoint requires nesting under
            # chat_template_kwargs for enable_thinking to take effect.
            extra_body={"chat_template_kwargs": {"enable_thinking": self._bailian_enable_thinking}},
            payload_kind="utf8_prompt",
            payload_bytes=len(prompt.encode("utf-8")),
        )

    def _load_memory(self) -> list[dict[str, Any]]:
        if not self._memory_path.exists():
            raise ValueError(f"memory file does not exist: {self._memory_path}")
        with self._memory_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("memory JSON must be a list")
        return data

    def _publish(self, payload: dict[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False)
        self.get_logger().info(text)
        self._result_pub.publish(String(data=text))


def _score_object(
    query: str,
    obj: dict[str, Any],
    target_name: str = "",
    semantic_hint: str = "",
) -> tuple[float, list[str]]:
    q = _normalize(" ".join(part for part in (target_name, semantic_hint, query) if part))
    score = 0.0
    reasons: list[str] = []

    names = []
    main_name = str(obj.get("main_name") or "").strip()
    if main_name:
        names.append(("main_name", main_name))
    possible = obj.get("possible_names")
    if isinstance(possible, list):
        for item in possible:
            text = str(item).strip()
            if text and text not in [name for _, name in names]:
                names.append(("possible_name", text))

    for field, name in names:
        n = _normalize(name)
        if not n:
            continue
        if n and n in q:
            delta = 100.0 if field == "main_name" else 80.0
            score += delta
            reasons.append(f"{field}_in_query:{name}")
        elif q and q in n:
            delta = 60.0 if field == "main_name" else 45.0
            score += delta
            reasons.append(f"query_in_{field}:{name}")
        else:
            overlap = _term_overlap(q, n)
            if overlap:
                delta = min(30.0, overlap * 8.0)
                score += delta
                reasons.append(f"name_term_overlap:{name}")

    yolo_class = str(obj.get("yolo_class") or "").strip()
    if yolo_class and _normalize(yolo_class) in q:
        score += 25.0
        reasons.append(f"yolo_class_in_query:{yolo_class}")

    if _looks_like_beverage_request(q):
        if yolo_class in BEVERAGE_CLASSES:
            score += 30.0
            reasons.append("beverage_request_class_match")
        if any(_normalize(term) in _normalize(" ".join(name for _, name in names)) for term in BEVERAGE_TERMS):
            score += 25.0
            reasons.append("beverage_request_name_match")

    if "瓶" in q and yolo_class == "bottle":
        score += 20.0
        reasons.append("bottle_requested")
    if "杯" in q and yolo_class == "cup":
        score += 20.0
        reasons.append("cup_requested")

    if str(obj.get("point_id") or "").strip():
        score += 1.0

    return score, reasons


def _compact_candidates(objects: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    candidates = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        target_id = str(obj.get("id") or "").strip()
        if not target_id:
            continue
        candidates.append({
            "id": target_id,
            "main_name": str(obj.get("main_name") or ""),
            "possible_names": [
                str(item)
                for item in obj.get("possible_names", [])
                if str(item).strip()
            ] if isinstance(obj.get("possible_names"), list) else [],
            "yolo_class": str(obj.get("yolo_class") or ""),
            "backup_yolo_classes": [
                str(item)
                for item in obj.get("backup_yolo_classes", [])
                if str(item).strip()
            ] if isinstance(obj.get("backup_yolo_classes"), list) else [],
            "point_id": str(obj.get("point_id") or ""),
            "detected_at_point": str(obj.get("detected_at_point") or ""),
        })
        if len(candidates) >= limit:
            break
    return candidates


def _resolve_target_name(query: str, obj: dict[str, Any]) -> str:
    q = query.strip()
    names = []
    main_name = str(obj.get("main_name") or "").strip()
    if main_name:
        names.append(main_name)
    possible = obj.get("possible_names")
    if isinstance(possible, list):
        names.extend(str(item).strip() for item in possible if str(item).strip())

    normalized_query = _normalize(q)
    for name in names:
        if _normalize(name) and _normalize(name) in normalized_query:
            return name
    if main_name:
        return main_name
    for name in names:
        return name
    return str(obj.get("id") or "target")


def _looks_like_beverage_request(normalized_query: str) -> bool:
    return any(_normalize(term) in normalized_query for term in BEVERAGE_TERMS)


def _normalize(text: str) -> str:
    return re.sub(r"\s+", "", str(text).strip().lower())


def _term_overlap(a: str, b: str) -> int:
    terms = [term for term in BEVERAGE_TERMS if _normalize(term) in a]
    return sum(1 for term in terms if _normalize(term) in b)


def _parse_json_object(text: str) -> dict[str, Any] | None:
    raw = str(text or "").strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?", "", raw, flags=re.IGNORECASE).strip()
        raw = re.sub(r"```$", "", raw).strip()
    match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
    if match:
        raw = match.group(0)
    try:
        data = json.loads(raw)
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def _safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectMemoryQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
