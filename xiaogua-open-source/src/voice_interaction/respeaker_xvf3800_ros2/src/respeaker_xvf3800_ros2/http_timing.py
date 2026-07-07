"""HTTP-attempt timing for the synchronous ASR API client."""

from __future__ import annotations

import itertools
import threading
import time
from dataclasses import dataclass
from typing import Callable

import httpx


@dataclass
class _State:
    call_id: int
    model: str
    payload_bytes: int
    attempt: int = 0


class HttpTimingRecorder:
    def __init__(self, component: str, log_callback: Callable[[str], None]) -> None:
        self.component = component
        self._log = log_callback
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._local = threading.local()

    def begin(self, model: str, payload_bytes: int) -> float:
        state = _State(next(self._ids), model, max(0, int(payload_bytes)))
        self._local.state = state
        self._emit(
            f"API_TIMING event=api_start component={self.component} "
            f"call={state.call_id} model={model} payload_kind=wav_base64 "
            f"payload_bytes={state.payload_bytes}"
        )
        return time.monotonic()

    def finish(self, started: float, outcome: str) -> None:
        state = self._state()
        self._emit(
            f"API_TIMING event=api_finish component={self.component} "
            f"call={state.call_id} attempts={state.attempt} outcome={outcome} "
            f"elapsed_sec={time.monotonic() - started:.3f}"
        )
        self._local.state = None

    def next_attempt(self) -> tuple[_State, int]:
        state = self._state()
        with self._lock:
            state.attempt += 1
            return state, state.attempt

    def success(self, state: _State, attempt: int, elapsed: float, response) -> None:
        request_id = (
            response.headers.get("x-request-id")
            or response.headers.get("request-id")
            or response.headers.get("x-dashscope-request-id")
            or "-"
        )
        self._emit(
            f"API_TIMING event=http_headers component={self.component} "
            f"call={state.call_id} attempt={attempt} status={response.status_code} "
            f"headers_sec={elapsed:.3f} request_id={request_id}"
        )

    def failure(self, state: _State, attempt: int, elapsed: float, exc) -> None:
        detail = str(exc).replace("\n", " ")[:180] or "-"
        self._emit(
            f"API_TIMING event=http_error component={self.component} "
            f"call={state.call_id} attempt={attempt} elapsed_sec={elapsed:.3f} "
            f"error_type={type(exc).__name__} detail={detail!r}"
        )

    def serialization(self, started: float, body_bytes: int) -> None:
        state = self._state()
        self._emit(
            f"API_TIMING event=json_ready component={self.component} "
            f"call={state.call_id} body_bytes={max(0, int(body_bytes))} "
            f"elapsed_sec={time.monotonic() - started:.3f}"
        )

    def retry(self, reason: str, wait_sec: float) -> None:
        state = self._state()
        self._emit(
            f"API_TIMING event=http_retry component={self.component} "
            f"call={state.call_id} after_attempt={state.attempt} "
            f"reason={reason} wait_sec={wait_sec:.3f}"
        )

    def _state(self) -> _State:
        state = getattr(self._local, "state", None)
        if state is None:
            state = _State(next(self._ids), "unknown", 0)
            self._local.state = state
        return state

    def _emit(self, message: str) -> None:
        try:
            self._log(message)
        except Exception:
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
            self._recorder.failure(state, attempt, time.monotonic() - started, exc)
            raise
        self._recorder.success(state, attempt, time.monotonic() - started, response)
        return response

    def close(self) -> None:
        self._transport.close()
