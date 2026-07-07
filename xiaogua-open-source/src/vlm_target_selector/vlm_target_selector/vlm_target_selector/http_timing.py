"""Low-noise timing instrumentation for synchronous HTTP calls."""

from __future__ import annotations

import itertools
import threading
import time
from dataclasses import dataclass
from typing import Callable

import httpx


LogCallback = Callable[[str], None]


@dataclass
class _CallState:
    call_id: int
    model: str
    payload_kind: str
    payload_bytes: int
    attempt: int = 0


class HttpTimingRecorder:
    """Track SDK calls and every underlying HTTP attempt without logging data."""

    def __init__(self, component: str, log_callback: LogCallback) -> None:
        self.component = component
        self._log_callback = log_callback
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._local = threading.local()

    def begin(self, model: str, payload_kind: str, payload_bytes: int) -> float:
        state = _CallState(
            call_id=next(self._ids),
            model=model,
            payload_kind=payload_kind,
            payload_bytes=max(0, int(payload_bytes)),
        )
        self._local.state = state
        self._emit(
            f"API_TIMING event=api_start component={self.component} "
            f"call={state.call_id} model={model} payload_kind={payload_kind} "
            f"payload_bytes={state.payload_bytes}"
        )
        return time.monotonic()

    def finish(self, started: float, outcome: str) -> None:
        state = self._state()
        elapsed = time.monotonic() - started
        self._emit(
            f"API_TIMING event=api_finish component={self.component} "
            f"call={state.call_id} attempts={state.attempt} outcome={outcome} "
            f"elapsed_sec={elapsed:.3f}"
        )
        self._local.state = None

    def next_attempt(self) -> tuple[_CallState, int]:
        state = self._state()
        with self._lock:
            state.attempt += 1
            attempt = state.attempt
        return state, attempt

    def log_attempt_success(
        self,
        state: _CallState,
        attempt: int,
        elapsed: float,
        response: httpx.Response,
    ) -> None:
        request_id = (
            response.headers.get("x-request-id")
            or response.headers.get("request-id")
            or response.headers.get("x-dashscope-request-id")
            or "-"
        )
        content_length = response.headers.get("content-length", "-")
        self._emit(
            f"API_TIMING event=http_headers component={self.component} "
            f"call={state.call_id} attempt={attempt} status={response.status_code} "
            f"headers_sec={elapsed:.3f} response_bytes={content_length} "
            f"request_id={request_id}"
        )

    def log_attempt_failure(
        self,
        state: _CallState,
        attempt: int,
        elapsed: float,
        exc: BaseException,
    ) -> None:
        detail = str(exc).replace("\n", " ")[:180] or "-"
        self._emit(
            f"API_TIMING event=http_error component={self.component} "
            f"call={state.call_id} attempt={attempt} elapsed_sec={elapsed:.3f} "
            f"error_type={type(exc).__name__} detail={detail!r}"
        )

    def log_serialization(self, call_started: float, body_bytes: int) -> None:
        state = self._state()
        self._emit(
            f"API_TIMING event=json_ready component={self.component} "
            f"call={state.call_id} body_bytes={max(0, int(body_bytes))} "
            f"elapsed_sec={time.monotonic() - call_started:.3f}"
        )

    def log_retry(self, status: str, wait_sec: float) -> None:
        state = self._state()
        self._emit(
            f"API_TIMING event=http_retry component={self.component} "
            f"call={state.call_id} after_attempt={state.attempt} "
            f"reason={status} wait_sec={wait_sec:.3f}"
        )

    def _state(self) -> _CallState:
        state = getattr(self._local, "state", None)
        if state is None:
            state = _CallState(next(self._ids), "unknown", "unknown", 0)
            self._local.state = state
        return state

    def _emit(self, message: str) -> None:
        try:
            self._log_callback(message)
        except Exception:
            # Diagnostics must never affect the robot task.
            pass


class TimingTransport(httpx.BaseTransport):
    def __init__(self, recorder: HttpTimingRecorder) -> None:
        self._recorder = recorder
        self._transport = httpx.HTTPTransport(retries=0)

    def handle_request(self, request: httpx.Request) -> httpx.Response:
        state, attempt = self._recorder.next_attempt()
        started = time.monotonic()
        try:
            response = self._transport.handle_request(request)
        except BaseException as exc:
            self._recorder.log_attempt_failure(
                state, attempt, time.monotonic() - started, exc
            )
            raise
        self._recorder.log_attempt_success(
            state, attempt, time.monotonic() - started, response
        )
        return response

    def close(self) -> None:
        self._transport.close()
