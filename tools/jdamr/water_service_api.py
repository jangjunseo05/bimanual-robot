#!/usr/bin/env python3
"""Authenticated HTTP facade for inference or navigation-only water missions.

The default mission includes Windows inference.  ``serve --navigation-only``
keeps the same start/status/stop API while running dock -> water station ->
table 1 -> dock without calling Windows.  Status reports the selected ``mode``.
"""

from __future__ import annotations

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import sys
import threading
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from water_service import WaterServiceController


DEFAULT_STATE_DIRECTORY = Path.home() / ".local/state/bimanual-robot/water-service"
MAX_BODY_BYTES = 4096
FOLLOW_INTERVAL_S = 0.5


def _read_or_create_token(path: Path) -> str:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        token = path.read_text(encoding="utf-8").strip()
        if not token:
            raise RuntimeError(f"API token file is empty: {path}")
        os.chmod(path, 0o600)
        return token
    token = secrets.token_urlsafe(32)
    with os.fdopen(descriptor, "w", encoding="utf-8") as output:
        output.write(token + "\n")
    return token


def _json_bytes(value: dict[str, Any]) -> bytes:
    return (json.dumps(value, ensure_ascii=False) + "\n").encode("utf-8")


class _MissionServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        controller: WaterServiceController,
        token: str,
        allow_origin: str | None,
    ) -> None:
        super().__init__(address, _MissionHandler)
        self.controller = controller
        self.token = token
        self.allow_origin = allow_origin


class _MissionHandler(BaseHTTPRequestHandler):
    server: _MissionServer

    def _send(self, status: int, value: dict[str, Any]) -> None:
        payload = _json_bytes(value)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        origin = self.headers.get("Origin")
        if self.server.allow_origin and origin == self.server.allow_origin:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.end_headers()
        self.wfile.write(payload)

    def _authorized(self) -> bool:
        supplied = self.headers.get("Authorization", "")
        expected = "Bearer " + self.server.token
        if hmac.compare_digest(supplied, expected):
            return True
        self._send(401, {"error": "unauthorized"})
        return False

    def _read_object(self) -> dict[str, Any]:
        raw_length = self.headers.get("Content-Length", "0")
        try:
            length = int(raw_length)
        except ValueError as error:
            raise ValueError("Content-Length must be an integer") from error
        if length < 0 or length > MAX_BODY_BYTES:
            raise ValueError(f"request body must be at most {MAX_BODY_BYTES} bytes")
        raw = self.rfile.read(length)
        if not raw:
            return {}
        try:
            value = json.loads(raw)
        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError("request body must be valid JSON") from error
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    def do_GET(self) -> None:
        if self.path not in ("/api/mission/status", "/status", "/api/mission/logs"):
            self._send(404, {"error": "not found"})
            return
        if self._authorized():
            try:
                result = (self.server.controller.logs() if self.path == "/api/mission/logs"
                          else self.server.controller.status())
                self._send(200, result)
            except (OSError, RuntimeError, ValueError) as error:
                self._send(503, {"error": str(error)})

    def do_POST(self) -> None:
        routes = {
            "/api/mission/start": "start",
            "/start": "start",
            "/api/mission/stop": "stop",
            "/stop": "stop",
        }
        action = routes.get(self.path)
        if action is None:
            self._send(404, {"error": "not found"})
            return
        if not self._authorized():
            return
        try:
            body = self._read_object()
            if action == "start":
                self._start(body)
            else:
                if body:
                    raise ValueError("stop request body must be empty")
                self._send(200, self.server.controller.stop())
        except ValueError as error:
            self._send(400, {"error": str(error)})

    def _start(self, body: dict[str, Any]) -> None:
        unknown = set(body) - {"request_id"}
        if unknown:
            raise ValueError("unknown start request fields: " + ", ".join(sorted(unknown)))
        request_id = body.get("request_id")
        if request_id is not None:
            if not isinstance(request_id, str) or not request_id.strip():
                raise ValueError("request_id must be a non-empty string")
            if len(request_id) > 128:
                raise ValueError("request_id must be at most 128 characters")
            request_id = request_id.strip()
        if not self.server.controller.status().get("configured"):
            self._send(503, {"error": "inference_url is not configured"})
            return
        try:
            result = self.server.controller.start(request_id=request_id)
        except RuntimeError as error:
            self._send(409, {"error": str(error)})
            return
        except ValueError as error:
            self._send(503, {"error": str(error)})
            return
        self._send(200 if result.get("duplicate") else 202, result)

    def do_OPTIONS(self) -> None:
        origin = self.headers.get("Origin")
        if not self.server.allow_origin or origin != self.server.allow_origin:
            self._send(403, {"error": "origin not allowed"})
            return
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", origin)
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
        self.send_header("Vary", "Origin")
        self.end_headers()

    def log_message(self, format_string: str, *arguments: object) -> None:
        sys.stderr.write("water-service-api: " + format_string % arguments + "\n")


def _follow_web(controller: WaterServiceController, done: threading.Event, dry_run: bool) -> None:
    """Follow the control page: start the mission its Start asks for and stop on its STOP.

    A lost link is only logged here.  A driving mission already stops itself when its own
    order server reads fail, and nothing starts without a fresh read.
    """
    last_error = None
    while not done.wait(FOLLOW_INTERVAL_S):
        try:
            action = controller.follow_web(controller.read_web_state(), dry_run=dry_run)
            controller.report_web(dry_run=dry_run)
        except (RuntimeError, ValueError) as error:
            if str(error) != last_error:
                print(f"water-service-api: follow: {error}", file=sys.stderr, flush=True)
            last_error = str(error)
            continue
        last_error = None
        if action:
            print(f"water-service-api: follow: {action}", flush=True)


def _client_request(url: str, token: str, action: str, request_id: str | None) -> int:
    path = f"/api/mission/{action}"
    payload = None
    method = "GET"
    if action not in ("status", "logs"):
        body = {"request_id": request_id} if action == "start" and request_id else {}
        payload = _json_bytes(body)
        method = "POST"
    request = Request(
        url.rstrip("/") + path,
        data=payload,
        method=method,
        headers={
            "Accept": "application/json",
            "Authorization": "Bearer " + token,
            "Content-Type": "application/json",
        },
    )
    try:
        with urlopen(request, timeout=15.0) as response:
            raw = response.read()
            status = response.status
    except HTTPError as error:
        raw = error.read()
        status = error.code
    except (URLError, TimeoutError, OSError) as error:
        print(json.dumps({"error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1
    sys.stdout.buffer.write(raw)
    return 0 if 200 <= status < 300 else 1


def _watch_logs(url: str, token: str) -> int:
    """Stream read-only progress; closing the viewer sends no robot command."""
    previous = None
    latest = {}
    try:
        while True:
            request = Request(url.rstrip("/") + "/api/mission/logs",
                              headers={"Authorization": "Bearer " + token})
            try:
                with urlopen(request, timeout=12.0) as response:
                    document = json.load(response)
                summary = {key: document.get(key) for key in
                           ("mission_id", "state", "phase", "error", "navigation_failure_reason")}
                if summary != previous:
                    print(json.dumps(summary, ensure_ascii=False), flush=True)
                    previous = summary
                for event in document.get("navigation_events", []):
                    stamp = event.get("monotonic_s")
                    key = document.get("navigation_run")
                    if isinstance(stamp, (int, float)) and stamp > latest.get(key, -1):
                        print(json.dumps({name: event[name] for name in
                                          ("event", "phase", "reason", "detail", "waypoint_id",
                                           "monotonic_s", "wall_time_s") if name in event},
                                         ensure_ascii=False), flush=True)
                        latest[key] = stamp
                if document.get("navigation_error"):
                    print(document["navigation_error"], file=sys.stderr, flush=True)
            except (OSError, ValueError) as error:
                print("log read: " + str(error), file=sys.stderr, flush=True)
            time.sleep(2.0)
    except KeyboardInterrupt:
        return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", type=Path, default=DEFAULT_STATE_DIRECTORY)
    parser.add_argument("--token-file", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)

    serve = subparsers.add_parser("serve", help="run the mission HTTP API")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8771)
    serve.add_argument("--order-url", dest="inference_url", default=os.environ.get("JDAMR_ORDER_URL"),
                       help="Windows order server, e.g. http://192.168.0.15:8765")
    serve.add_argument("--inference-timeout-s", type=float, default=300.0)
    serve.add_argument(
        "--navigation-only",
        action="store_true",
        help="run dock -> water station -> table 1 -> dock without Windows inference",
    )
    serve.add_argument("--follow-web", action="store_true",
                       help="start the mission when the control page Start folds the arms; stop on its STOP")
    serve.add_argument("--follow-dry-run", action="store_true",
                       help="with --follow-web: report what would happen; never start or stop anything")
    serve.add_argument("--map-session", type=Path)
    serve.add_argument("--allow-origin", help="one exact browser Origin allowed by CORS")

    for name in ("start", "status", "logs", "watch", "stop"):
        command = subparsers.add_parser(name, help=f"call the local API {name} endpoint")
        command.add_argument("--url", default="http://127.0.0.1:8771")
        if name == "start":
            command.add_argument("--request-id")
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    state_directory = arguments.state_dir.expanduser().resolve()
    token_path = (arguments.token_file or state_directory / "api-token").expanduser().resolve()
    token = _read_or_create_token(token_path)
    if arguments.command == "watch":
        return _watch_logs(arguments.url, token)
    if arguments.command != "serve":
        return _client_request(
            arguments.url, token, arguments.command, getattr(arguments, "request_id", None)
        )

    controller = WaterServiceController(
        inference_url=arguments.inference_url,
        state_directory=state_directory,
        map_session=arguments.map_session,
        inference_timeout_s=arguments.inference_timeout_s,
        run_inference=not arguments.navigation_only,
    )
    server = _MissionServer(
        (arguments.host, arguments.port), controller, token, arguments.allow_origin
    )
    follow_done = threading.Event()
    if arguments.follow_web:
        if arguments.navigation_only or not arguments.inference_url:
            raise SystemExit("--follow-web needs --order-url and a mission with inference")
        threading.Thread(target=_follow_web, args=(controller, follow_done, arguments.follow_dry_run),
                         name="follow-web", daemon=True).start()

    def terminate(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        follow_done.set()
        controller.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
