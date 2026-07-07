"""Small persistent HTTP client for Bailian's Chat Completions endpoint."""

from __future__ import annotations

import json
import time
from typing import Any, Callable

import httpx

from .http_timing import HttpTimingRecorder, TimingTransport


class DirectBailianError(RuntimeError):
    """Base error for direct Bailian requests."""


class DirectBailianConnectionError(DirectBailianError):
    """Network or timeout failure."""


class DirectBailianAPIError(DirectBailianError):
    """Non-success HTTP response."""


class DirectBailianResponseError(DirectBailianError):
    """Malformed or empty success response."""


class DirectBailianClient:
    RETRYABLE_STATUS = {408, 409, 429}

    def __init__(
        self,
        *,
        api_key: str,
        base_url: str,
        timeout: float,
        component: str,
        log_callback: Callable[[str], None],
        max_retries: int = 2,
    ) -> None:
        self._endpoint = f"{base_url.rstrip('/')}/chat/completions"
        self._max_retries = max(0, int(max_retries))
        self._recorder = HttpTimingRecorder(component, log_callback)
        self._client = httpx.Client(
            timeout=httpx.Timeout(float(timeout)),
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
        payload_kind: str,
        payload_bytes: int,
        temperature: float | None = None,
        extra_body: dict[str, Any] | None = None,
    ) -> str:
        started = self._recorder.begin(model, payload_kind, payload_bytes)
        payload: dict[str, Any] = {"model": model, "messages": messages}
        if temperature is not None:
            payload["temperature"] = temperature
        if extra_body:
            payload.update(extra_body)

        try:
            body = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            self._recorder.log_serialization(started, len(body))
            response = self._post_with_retries(body)
            content = self._extract_content(response)
        except Exception as exc:
            self._recorder.finish(started, type(exc).__name__)
            raise

        self._recorder.finish(started, "success")
        return content

    def close(self) -> None:
        self._client.close()

    def _post_with_retries(self, body: bytes) -> httpx.Response:
        last_error: Exception | None = None
        for retry_index in range(self._max_retries + 1):
            try:
                response = self._client.post(self._endpoint, content=body)
            except (httpx.TimeoutException, httpx.NetworkError) as exc:
                last_error = exc
                if retry_index >= self._max_retries:
                    raise DirectBailianConnectionError(str(exc)) from exc
                self._sleep_before_retry(retry_index, type(exc).__name__)
                continue

            if 200 <= response.status_code < 300:
                return response

            if self._is_retryable(response.status_code) and retry_index < self._max_retries:
                self._sleep_before_retry(
                    retry_index,
                    f"http_{response.status_code}",
                    response.headers.get("retry-after"),
                )
                continue

            raise DirectBailianAPIError(self._format_api_error(response))

        raise DirectBailianConnectionError(str(last_error or "request failed"))

    def _sleep_before_retry(
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
        self._recorder.log_retry(reason, wait_sec)
        time.sleep(wait_sec)

    @classmethod
    def _is_retryable(cls, status_code: int) -> bool:
        return status_code in cls.RETRYABLE_STATUS or status_code >= 500

    @staticmethod
    def _format_api_error(response: httpx.Response) -> str:
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
            body = response.json()
            content = body["choices"][0]["message"]["content"]
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            raise DirectBailianResponseError(
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
            raise DirectBailianResponseError("Bailian returned empty content")
        return text
