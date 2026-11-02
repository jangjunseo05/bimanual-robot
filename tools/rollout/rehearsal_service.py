"""팔 없이 시험하는 가짜 조작 실행기. rollout_service.py와 같은 주소·응답을 흉내 내며 하드웨어를 열지 않는다.

주문 서버와 주행 연동을 팔·모델 없이 확인할 때 쓴다. 팔 포트·카메라·GPU를 쓰지 않으므로 아무것도 움직이지 않는다.
실제 실행기와 같은 포트를 쓰므로 start_scenario.bat과 함께 켜지 않는다.

    python tools/rollout/rehearsal_service.py
"""
from __future__ import annotations

import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST, PORT = "127.0.0.1", 8770
RUN_S = {"pour": 3.0, "place": 2.0}
ARMS_AFTER = {"pour": "carrying_cup", "place": "stowed", "stow": "stowed", "stow_right": "carrying_cup"}


class Rehearsal:
    def __init__(self) -> None:
        self.arms = "unknown"
        self.current: str | None = None
        self.last: dict | None = None
        self.stop_requested = threading.Event()
        self.lock = threading.Lock()

    def status(self) -> dict:
        return {"state": "running" if self.current else "ready", "current": self.current,
                "timings": {"rehearsal": True}, "last_result": self.last, "recent_results": [],
                "idle_read_failures": 0, "stow_pose": "place_end", "arms": self.arms}

    def run(self, task: str) -> dict:
        with self.lock:
            self.current, self.arms = task, "busy"
            self.stop_requested.clear()
        stopped = self.stop_requested.wait(RUN_S.get(task, 0.5))
        reason = "stopped" if stopped else "stowed" if task.startswith("stow") else "auto_done"
        with self.lock:
            self.arms = "hold" if stopped else ARMS_AFTER[task]
            self.current = None
            self.last = {"task": task, "ok": not stopped, "reason": reason,
                         "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        print(f"[rehearsal {time.strftime('%H:%M:%S')}] {task}: {reason}, arms={self.arms}", flush=True)
        return self.last


class Server(ThreadingHTTPServer):
    # Windows는 이 옵션이 켜져 있으면 실제 실행기가 쓰는 포트에도 겹쳐 열린다.
    allow_reuse_address = False


def handler_factory(rehearsal: Rehearsal):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args) -> None:
            return

        def _json(self, code: int, body: dict) -> None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.startswith("/status"):
                self._json(HTTPStatus.OK, rehearsal.status())
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?")[0].strip("/")
            if path == "stop":
                rehearsal.stop_requested.set()
                self._json(HTTPStatus.OK, {"stopped": rehearsal.current is not None})
                return
            task = path.removeprefix("run/")
            if task in ARMS_AFTER:
                self._json(HTTPStatus.OK, rehearsal.run(task))
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_task"})

    return Handler


def main() -> None:
    server = Server((HOST, PORT), handler_factory(Rehearsal()))
    print(f"[rehearsal] 가짜 조작 실행기 http://{HOST}:{PORT} (팔·카메라를 쓰지 않음, Ctrl+C로 종료)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
