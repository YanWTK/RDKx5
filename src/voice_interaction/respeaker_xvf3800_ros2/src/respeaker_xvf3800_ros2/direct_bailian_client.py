"""Persistent direct HTTP client for Bailian ASR chat requests."""

from __future__ import annotations

import json
import time
from typing import Any, Callable

import httpx

from .http_timing import HttpTimingRecorder, TimingTransport


class DirectBailianError(RuntimeError):
    pass


class DirectBailianClient:
    RETRYABLE_STATUS = {408, 409, 429}

    def __init__(
        self,
        *,
        api_key: str,
        base_url: str,
        component: str,
        log_callback: Callable[[str], None],
        connect_timeout: float = 5.0,
        read_timeout: float = 600.0,
        write_timeout: float = 600.0,
        max_retries: int = 2,
    ) -> None:
        self._endpoint = f"{base_url.rstrip('/')}/chat/completions"
        self._max_retries = max(0, int(max_retries))
        self._recorder = HttpTimingRecorder(component, log_callback)
        self._client = httpx.Client(
            timeout=httpx.Timeout(
                connect=connect_timeout,
                read=read_timeout,
                write=write_timeout,
                pool=read_timeout,
            ),
            transport=TimingTransport(self._recorder),
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
        )

    def chat_completion(
        self,
        *,
        model: str,
        messages: list[dict[str, Any]],
        extra_body: dict[str, Any],
        payload_bytes: int,
    ) -> str:
        started = self._recorder.begin(model, payload_bytes)
        payload = {"model": model, "messages": messages, **extra_body}
        try:
            body = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            self._recorder.serialization(started, len(body))
            response = self._post_with_retries(body)
            content = self._extract_content(response)
        except Exception as exc:
            self._recorder.finish(started, type(exc).__name__)
            raise
        self._recorder.finish(started, "success")
        return content

    def _post_with_retries(self, body: bytes) -> httpx.Response:
        for retry_index in range(self._max_retries + 1):
            try:
                response = self._client.post(self._endpoint, content=body)
            except (httpx.TimeoutException, httpx.NetworkError) as exc:
                if retry_index >= self._max_retries:
                    raise DirectBailianError(str(exc)) from exc
                self._sleep(retry_index, type(exc).__name__)
                continue

            if 200 <= response.status_code < 300:
                return response
            retryable = (
                response.status_code in self.RETRYABLE_STATUS
                or response.status_code >= 500
            )
            if retryable and retry_index < self._max_retries:
                self._sleep(
                    retry_index,
                    f"http_{response.status_code}",
                    response.headers.get("retry-after"),
                )
                continue
            raise DirectBailianError(self._format_error(response))
        raise DirectBailianError("request failed")

    def _sleep(
        self,
        retry_index: int,
        reason: str,
        retry_after: str | None = None,
    ) -> None:
        wait_sec = min(0.5 * (2**retry_index), 2.0)
        if retry_after:
            try:
                wait_sec = min(max(float(retry_after), 0.0), 5.0)
            except ValueError:
                pass
        self._recorder.retry(reason, wait_sec)
        time.sleep(wait_sec)

    @staticmethod
    def _format_error(response: httpx.Response) -> str:
        request_id = (
            response.headers.get("x-request-id")
            or response.headers.get("request-id")
            or response.headers.get("x-dashscope-request-id")
            or "-"
        )
        message = response.text[:240]
        try:
            body = response.json()
            error = body.get("error") if isinstance(body, dict) else None
            if isinstance(error, dict):
                message = str(error.get("message") or error.get("code") or message)
        except Exception:
            pass
        return (
            f"Bailian HTTP {response.status_code}: {message} "
            f"(request_id={request_id})"
        )

    @staticmethod
    def _extract_content(response: httpx.Response) -> str:
        try:
            content = response.json()["choices"][0]["message"]["content"]
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            raise DirectBailianError(
                f"response missing choices[0].message.content: {response.text[:200]}"
            ) from exc

        if isinstance(content, str):
            text = content.strip()
        elif isinstance(content, list):
            parts: list[str] = []
            for item in content:
                if isinstance(item, str):
                    parts.append(item)
                elif isinstance(item, dict):
                    value = item.get("text") or item.get("content")
                    if value:
                        parts.append(str(value))
            text = "".join(parts).strip()
        else:
            text = str(content or "").strip()
        if not text:
            raise DirectBailianError("Bailian returned empty content")
        return text
