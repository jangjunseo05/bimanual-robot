#!/usr/bin/env python3
"""Local web order API for the CPU-only service mission controller."""
from __future__ import annotations

import argparse
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from collections import deque
from typing import Any
from urllib.parse import parse_qs, urlparse
from uuid import uuid4

from src.mission.service_execution_backend import CommandBackend, ImmediateBackend, Ros2ManipulationBackend, Ros2PlannedArtifactBackend, Ros2PlannedRoundtripBackend, Ros2PlannedSessionBackend, RolloutServiceBackend
from src.mission.service_mission_control import DryRunRuntime, MissionController, ORDER_STATES, OrderStateError
from src.mission.service_order_store import ServiceOrderStore


ROOT = Path(__file__).resolve().parents[2]
INDEX = Path(__file__).resolve().parents[2] / 'apps/service_order/static/index.html'
DASHBOARD_JS = INDEX.with_name("dashboard.js")
ORDER_PAGE = INDEX.with_name("order.html")
ORDER_JS = INDEX.with_name("order.js")
CONTROL_PAGE = INDEX.with_name("control.html")
CONTROL_JS = INDEX.with_name("control.js")
API_JS = INDEX.with_name("api.js")
CONFIG_JS = INDEX.with_name("config.js")
STATIC_FILES = {
    "/": (INDEX, "text/html"),
    "/index.html": (INDEX, "text/html"),
    "/dashboard.js": (DASHBOARD_JS, "text/javascript"),
    "/order": (ORDER_PAGE, "text/html"),
    "/order.js": (ORDER_JS, "text/javascript"),
    "/control": (CONTROL_PAGE, "text/html"),
    "/control.js": (CONTROL_JS, "text/javascript"),
    "/api.js": (API_JS, "text/javascript"),
    "/config.js": (CONFIG_JS, "text/javascript"),
}
ORDER_PATH = re.compile(r"^/api/orders/([A-Za-z0-9._:-]{1,64})/(cancel|accept|reject)$")
TICKET_PATH = re.compile(r"^/api/tickets/([A-Za-z0-9._:-]{1,64})/served$")
# 메뉴 사진(static/menu/*.jpg), 관제 지도(static/map/*.png|json), 손님 화면 글꼴(static/fonts/*.woff)
ASSET_PATH = re.compile(r"^/(menu|map|fonts)/([a-z0-9_]{1,40})\.(jpg|png|json|woff)$")
ASSET_TYPES = {"jpg": "image/jpeg", "png": "image/png", "json": "application/json; charset=utf-8", "woff": "font/woff"}
CLIENT_REF_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,64}$")
ORDER_NO_DIGITS = 3  # 주문 번호 = YYMMDD + 일련번호 3자리 (예: 261027001)
DRIVE_LINK_TIMEOUT_S = 3.0  # 주행 PC 보고가 이보다 오래 없으면 미연결로 표시한다
DRIVE_INIT_PICKUP_TIMEOUT_S = 5.0  # 주행 PC가 이 안에 초기화 요청을 가져가지 않으면 실패로 표시한다
DRIVE_INIT_IDLE = {"id": None, "state": "idle", "message": None, "requested": None}
DRIVE_INIT_ACTIVE = ("requested", "running")


def _trim(value, depth: int = 0):
    """Keep the dashboard snapshot small for the browser: drop long arrays (scan ranges, path points)."""
    if isinstance(value, dict):
        return {k: _trim(v, depth + 1) for k, v in value.items() if depth < 4}
    if isinstance(value, list):
        return value if len(value) <= 60 else {"count": len(value)}
    return value


class RobotPoseFeed:
    """Polls the Pi navigation dashboard (ammr_dashboard /api/state) for the live map pose and ROS status.

    Besides the pose it keeps a trimmed copy of the dashboard snapshot (topic freshness, ROS graph,
    battery, odometry, Nav2 goal) for the developer console on /control.

    The dashboard listens on the Pi loopback only, so reach it through an SSH tunnel:
    `ssh -N -L 8090:127.0.0.1:8090 lim@<pi>` (or pass --robot-ssh to let this server open it).
    """

    def __init__(self, url: str, interval_s: float = 1.0, ssh_target: str | None = None) -> None:
        self.url = url
        self.interval_s = interval_s
        self.ssh_target = ssh_target
        self.pose: dict = {"live": False, "error": "not polled yet"}
        self.nav: dict = {"connected": False, "error": "not polled yet", "url": url}
        self._stop = threading.Event()
        self._tunnel = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        if self.ssh_target:
            import subprocess

            port = urlparse(self.url).port or 8090
            self._tunnel = subprocess.Popen(
                ["ssh", "-N", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes", "-o", "ConnectTimeout=8",
                 "-L", f"{port}:127.0.0.1:8090", self.ssh_target],
                stdin=subprocess.DEVNULL,
            )
        self._thread.start()

    def _run(self) -> None:
        import time
        import urllib.request

        while not self._stop.is_set():
            try:
                started = time.perf_counter()
                with urllib.request.urlopen(self.url, timeout=2.0) as response:
                    data = json.loads(response.read().decode("utf-8"))
                self.nav = {"connected": True, "error": None, "url": self.url, "received_at": time.time(),
                            "latency_ms": round((time.perf_counter() - started) * 1000, 1), "snapshot": _trim(data)}
                loc = (data.get("navigation") or {}).get("localization") or {}
                if not all(isinstance(loc.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_deg")):
                    raise ValueError("dashboard has no map pose yet")
                self.pose = {
                    "live": bool(loc.get("live", True)), "x_m": loc["x_m"], "y_m": loc["y_m"], "yaw_deg": loc["yaw_deg"],
                    "frame_id": loc.get("frame_id", "map"), "age_s": loc.get("age_s"), "received_at": time.time(),
                }
            except Exception as exc:  # noqa: BLE001 - 위치는 보조 정보다. 실패해도 주문 처리는 계속한다.
                if not isinstance(exc, ValueError) or "map pose" not in str(exc):
                    self.nav = {"connected": False, "error": str(exc)[:160], "url": self.url,
                                "last_snapshot_at": self.nav.get("received_at") or self.nav.get("last_snapshot_at")}
                self.pose = {"live": False, "error": str(exc)[:160], "last": self.pose if self.pose.get("x_m") is not None else self.pose.get("last")}
            self._stop.wait(self.interval_s)

    def close(self) -> None:
        self._stop.set()
        if self._tunnel is not None:
            self._tunnel.terminate()


class BackendBusyError(RuntimeError):
    """Raised when another backend command is still in progress."""


def default_state_dir() -> Path:
    state_home = os.environ.get("XDG_STATE_HOME")
    base = Path(state_home).expanduser() if state_home else Path.home() / ".local/state"
    return base / "bimanual-robot/service-order-server"


class ServiceApplication:
    """Thread-safe application boundary shared by HTTP and the dry-run loop."""

    def __init__(
        self,
        controller: MissionController | None = None,
        state_dir: Path | None = None,
        backend: CommandBackend | None = None,
    ) -> None:
        self.state_dir = Path(state_dir) if state_dir is not None else None
        self.lock = threading.RLock()
        self.store = ServiceOrderStore(self.state_dir) if self.state_dir is not None else None
        persisted = self.store.load_snapshot() if self.store is not None else None
        selected_backend = backend or ImmediateBackend()
        if persisted is not None and selected_backend.mode == "ros2_planned_roundtrip":
            if self.store is not None:
                self.store.close()
            selected_backend.close()
            raise RuntimeError("ros2-planned-roundtrip cannot resume a persisted Isaac world")
        if persisted is not None:
            source = controller or MissionController()
            self.controller = MissionController.from_snapshot(
                persisted,
                config=source.config,
                layout=source.layout,
                require_operator=source.require_operator,
                clock=source.clock,
            )
        else:
            self.controller = controller or MissionController()
        self.runtime = DryRunRuntime(self.controller)
        self.backend = selected_backend
        self.execution_lock = threading.Lock()
        self.last_backend_result: dict | None = None
        self.auto_paused = False
        # 긴급 정지 잠금: 해제할 때까지 다음 단계와 Start를 막는다. 주행 쪽은 /api/state 로 읽는다.
        self.emergency: dict = {"active": False, "at": None, "order_id": None}
        self.drive_link: dict | None = None
        # 주행 초기화 요청: 주행 PC가 /api/state 로 읽어 실행하고 heartbeat 로 결과를 알린다.
        self.drive_init: dict = dict(DRIVE_INIT_IDLE)
        # 주문표: 주문 번호별 메뉴 줄(로봇 음료 + 직원이 가져다주는 음식). 로봇 미션은 음료만 맡는다.
        self.tickets: dict[str, dict] = self._load_tickets()
        self.pose_feed: RobotPoseFeed | None = None
        self.started_at = datetime.now().astimezone()
        # 개발자 콘솔용 최근 실행 명령(단계별 backend 호출, 결과, 걸린 시간)
        self.recent_commands: deque[dict] = deque(maxlen=40)
        self._persist()

    def _load_tickets(self) -> dict[str, dict]:
        path = self.state_dir / "tickets.json" if self.state_dir is not None else None
        if path is None or not path.exists():
            return {}
        try:
            return {ticket["order_id"]: ticket for ticket in json.loads(path.read_text(encoding="utf-8"))}
        except (ValueError, KeyError, TypeError):
            return {}

    def _persist_tickets(self) -> None:
        if self.state_dir is None:
            return
        self.state_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.state_dir / "tickets.json.tmp"
        temporary.write_text(json.dumps(list(self.tickets.values()), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.state_dir / "tickets.json")

    def _next_order_no(self) -> str:
        """Next YYMMDDNNN for today.

        Orders from before the 9-digit format (WEB-..., STAFF-...) are shown on /control with the lowest
        free numbers of their day (control.js `buildDisplayNumbers`), so a new number also skips those slots.
        """
        prefix = datetime.now().strftime("%y%m%d")
        used: set[int] = set()
        legacy = 0
        records = [*((o.order_id, o.created_at) for o in self.controller.orders.values()),
                   *((t["order_id"], t.get("created_at")) for t in self.tickets.values())]
        for order_id, created_at in dict(records).items():
            if re.fullmatch(r"\d{9}", order_id):
                if order_id.startswith(prefix):
                    used.add(int(order_id[6:]))
            elif created_at and datetime.fromisoformat(created_at.replace("Z", "+00:00")).astimezone().strftime("%y%m%d") == prefix:
                legacy += 1
        slot = 0
        while legacy:
            slot += 1
            if slot not in used:
                legacy -= 1
        return f"{prefix}{max([slot, *used]) + 1:0{ORDER_NO_DIGITS}d}"

    @staticmethod
    def _validate_items(raw: Any) -> list[dict]:
        if raw is None:
            return []
        if not isinstance(raw, list) or len(raw) > 20:
            raise ValueError("items must be a list of at most 20 menu lines")
        items = []
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError("each item must be an object")
            name, qty, price = item.get("name"), item.get("qty", 1), item.get("price", 0)
            if not isinstance(name, str) or not 0 < len(name) <= 40:
                raise ValueError("item name must be 1-40 characters")
            if isinstance(qty, bool) or not isinstance(qty, int) or not 1 <= qty <= 20:
                raise ValueError("item qty must be an integer from 1 to 20")
            if isinstance(price, bool) or not isinstance(price, int) or not 0 <= price <= 1_000_000:
                raise ValueError("item price must be an integer from 0 to 1000000")
            items.append({"name": name, "qty": qty, "price": price, "robot": bool(item.get("robot", False))})
        return items

    def _persist(self, command: dict | None = None) -> None:
        if self.state_dir is None:
            return
        self.state_dir.mkdir(parents=True, exist_ok=True)
        snapshot = self.controller.snapshot()
        snapshot['execution_backend'] = self.backend.info()
        snapshot['simulator_accessed'] = snapshot['execution_backend'].get('simulator_accessed', False)
        if self.store is not None:
            self.store.persist(snapshot, command)
        temporary = self.state_dir / "snapshot.json.tmp"
        temporary.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.state_dir / "snapshot.json")
        event_temporary = self.state_dir / "events.jsonl.tmp"
        event_temporary.write_text(
            "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in self.controller.events),
            encoding="utf-8",
        )
        event_temporary.replace(self.state_dir / "events.jsonl")

    def state(self) -> dict:
        with self.lock:
            snapshot = self.controller.snapshot()
            snapshot["stats"] = self._stats(snapshot)
            snapshot["persistence"] = (
                self.store.info() if self.store is not None else {"enabled": False, "backend": None}
            )
            snapshot["restaurant"] = {
                "room_bounds_m": self.controller.layout["room_bounds_m"],
                "kitchen": self.controller.layout["kitchen"],
                "tables": self.controller.layout["tables"],
                "waypoints": self.controller.layout["waypoints"],
            }
            snapshot["execution_backend"] = self.backend.info()
            snapshot['simulator_accessed'] = snapshot['execution_backend'].get('simulator_accessed', False)
            snapshot["last_backend_result"] = self.last_backend_result
            snapshot["auto_paused"] = self.auto_paused
            snapshot["emergency_stop"] = dict(self.emergency)
            snapshot["start"] = self._start_status(snapshot["execution_backend"])
            snapshot["drive_link"] = self._drive_link_status()
            snapshot["drive_init"] = self._drive_init_status()
            snapshot["tickets"] = list(self.tickets.values())
            snapshot["robot_pose"] = self.pose_feed.pose if self.pose_feed is not None else None
            snapshot["robot_nav"] = self.pose_feed.nav if self.pose_feed is not None else None
            snapshot["recent_commands"] = list(self.recent_commands)
            snapshot["server"] = {
                "started_at": self.started_at.isoformat(timespec="seconds"),
                "uptime_s": round((datetime.now().astimezone() - self.started_at).total_seconds(), 1),
                "pid": os.getpid(),
                "auto_paused": self.auto_paused,
                "busy": self.execution_lock.locked(),
            }
            return snapshot

    def drive_heartbeat(self, payload: dict) -> dict:
        """주행 PC가 관제를 따라가는 중임을 알린다. 표시용이며 미션을 바꾸지 않는다."""
        with self.lock:
            self.drive_link = {"received": time.monotonic(),
                               **{k: payload.get(k) for k in ("state", "phase", "mission_id", "error", "dry_run")}}
            self._update_drive_init(payload.get("init"))
            return {"ok": True}

    def _drive_link_status(self) -> dict:
        if self.drive_link is None:
            return {"connected": False}
        age_s = time.monotonic() - self.drive_link["received"]
        return {**{k: v for k, v in self.drive_link.items() if k != "received"},
                "connected": age_s < DRIVE_LINK_TIMEOUT_S, "age_s": round(age_s, 1)}

    def _update_drive_init(self, report: Any) -> None:
        if self.drive_init["state"] not in DRIVE_INIT_ACTIVE:
            return
        report = report if isinstance(report, dict) else {}
        outcome = report.get("state")
        if report.get("id") == self.drive_init["id"] and outcome in ("running", "succeeded", "failed"):
            self.drive_init.update(state=outcome, message=report.get("message"))
            if outcome != "running":
                self.controller.record_event(f"DRIVE_INIT_{outcome.upper()}")
                self._persist()
        elif self.drive_init["state"] == "running":
            # 주행 PC 실행기가 재시작되면 진행 중이던 초기화를 잃는다.
            self.drive_init.update(state="failed", message="주행 PC가 진행 중인 초기화를 보고하지 않습니다")

    def _expire_drive_init(self) -> None:
        requested = self.drive_init["requested"]
        if (self.drive_init["state"] == "requested"
                and time.monotonic() - requested > DRIVE_INIT_PICKUP_TIMEOUT_S):
            self.drive_init.update(state="failed", message="주행 PC가 초기화 요청을 가져가지 않았습니다")

    def _drive_init_block(self) -> str | None:
        """Why init may not run now.  An order ends at cup placement while the car still returns to the dock."""
        link = self._drive_link_status()
        if self.emergency["active"]:
            return "emergency_stop"
        if self.controller.active_order is not None or link.get("state") == "running":
            return "mission_running"
        if not link["connected"]:
            return "drive_not_connected"
        if self.drive_init["state"] in DRIVE_INIT_ACTIVE:
            return "drive_init_running"
        return None

    def _drive_init_status(self) -> dict:
        self._expire_drive_init()
        reason = self._drive_init_block()
        return {**{k: self.drive_init[k] for k in ("id", "state", "message")},
                "ready": reason is None, "reason": reason}

    def request_drive_init(self) -> dict:
        """Control page init: the drive PC reads this request and runs jdamr_depart.py init, which sends no motion."""
        with self.lock:
            self._expire_drive_init()
            reason = self._drive_init_block()
            if reason is not None:
                raise OrderStateError(reason)
            self.drive_init = {**DRIVE_INIT_IDLE, "id": uuid4().hex[:8], "state": "requested",
                               "requested": time.monotonic()}
            self.controller.record_event("DRIVE_INIT_REQUESTED")
            self._persist()
            return {"drive_init": self._drive_init_status()}

    def _start_status(self, backend_info: dict) -> dict:
        """Whether the control page Start may run now, and the order it would release."""
        accepted = self.controller.accepted_orders()
        reason = None
        if self.emergency["active"]:
            reason = "emergency_stop"
        elif self.controller.active_order is not None:
            reason = "mission_running"
        elif self.drive_init["state"] in DRIVE_INIT_ACTIVE:
            reason = "drive_init_running"
        elif self.drive_init["state"] == "failed":
            reason = "drive_init_failed"
        elif not accepted:
            reason = "no_accepted_order"
        elif backend_info.get("manipulation_service_state") not in (None, "ready"):
            reason = "manipulation_not_ready"
        return {"ready": reason is None, "reason": reason, "order_id": accepted[0].order_id if accepted else None}

    def _operator_action(self, action, order_id: str) -> dict:
        with self.lock:
            order = action(order_id)
            self._persist()
            return {"order": order.__dict__, "state": self.controller.snapshot()}

    def accept(self, order_id: str) -> dict:
        return self._operator_action(self.controller.accept, order_id)

    def _drop_staff_items(self, order_id: str) -> None:
        """A refused or withdrawn order takes its food lines with it; staff should not serve them."""
        ticket = self.tickets.get(order_id)
        if ticket is not None and ticket["staff_state"] == "PREPARING":
            ticket["staff_state"] = None
            self._persist_tickets()

    def reject(self, order_id: str) -> dict:
        with self.lock:
            result = self._operator_action(self.controller.reject, order_id)
            self._drop_staff_items(order_id)
            return result

    def start_mission(self) -> dict:
        """Control page Start: release the oldest accepted order so the mission begins."""
        with self.lock:
            status = self._start_status(self.backend.info())
            if not status["ready"]:
                raise OrderStateError(status["reason"])
            order = self.controller.start_next()
            # 초기화 결과는 한 회차용이다. 다음 회차 전에 다시 눌러야 하므로 표시를 지운다.
            self.drive_init = dict(DRIVE_INIT_IDLE)
            self._persist()
            return {"order": order.__dict__, "state": self.controller.snapshot()}

    def emergency_stop(self) -> dict:
        """Latch STOP first so nothing new can start, then stop the arms, then end the mission."""
        with self.lock:
            if not self.emergency["active"]:
                self.emergency = {"active": True, "at": datetime.now().astimezone().isoformat(timespec="seconds"),
                                  "order_id": self.controller.active_order_id}
        arm_stop_sent = getattr(self.backend, "emergency_stop", self.backend.cancel_active)()
        with self.lock:
            order = self.controller.emergency_stop()
            self._persist()
            return {"emergency_stop": dict(self.emergency), "arm_stop_sent": arm_stop_sent,
                    "order": order.__dict__ if order else None, "state": self.controller.snapshot()}

    def reset_emergency_stop(self) -> dict:
        with self.lock:
            if self.emergency["active"]:
                self.emergency = {"active": False, "at": None, "order_id": None}
                release = getattr(self.backend, "release_emergency_stop", None)
                if release is not None:
                    release()
                self.controller.record_event("EMERGENCY_STOP_RESET")
                self._persist()
            return {"emergency_stop": dict(self.emergency), "state": self.controller.snapshot()}

    def set_auto_paused(self, paused: bool) -> dict:
        """Pause or resume automatic phase execution; a running backend command still finishes."""
        with self.lock:
            self.auto_paused = paused
            return {"auto_paused": self.auto_paused}

    def _stats(self, snapshot: dict) -> dict:
        if self.store is not None:
            return self.store.stats()
        counts = {state: 0 for state in ORDER_STATES}
        for order in snapshot["orders"]:
            counts[order["state"]] += 1
        return {
            "total": len(snapshot["orders"]),
            "pending": counts["PENDING"],
            "accepted": counts["ACCEPTED"],
            "queued": counts["QUEUED"],
            "running": counts["RUNNING"],
            "succeeded": counts["SUCCEEDED"],
            "failed": counts["FAILED"],
            "canceled": counts["CANCELED"],
            "rejected": counts["REJECTED"],
            "events": len(snapshot["events"]),
            "commands": len(self.runtime.commands),
        }

    def history(self, kind: str, limit: int) -> list[dict]:
        with self.lock:
            if self.store is None:
                if kind == "orders":
                    return list(reversed(self.controller.snapshot()["orders"]))[:limit]
                return self.controller.events[-limit:]
            return self.store.order_history(limit) if kind == "orders" else self.store.event_history(limit)

    def submit(self, payload: dict) -> tuple[dict, bool]:
        """Accept an order. Without order_id the server assigns the next YYMMDDNNN number.

        `drink` (optional) goes to the robot mission; `items` (optional) are the menu lines for the
        ticket, food included, which staff serve.  `client_ref` makes a resend idempotent.
        """
        with self.lock:
            if not isinstance(payload, dict):
                raise ValueError("order payload must be an object")
            client_ref = payload.get("client_ref")
            if client_ref is not None and (not isinstance(client_ref, str) or not CLIENT_REF_PATTERN.fullmatch(client_ref)):
                raise ValueError("client_ref must be 1-64 safe identifier characters")
            if client_ref is not None:
                for ticket in self.tickets.values():
                    if ticket.get("client_ref") == client_ref:
                        order = self.controller.orders.get(ticket["order_id"])
                        return {"order": order.__dict__ if order else None, "ticket": ticket,
                                "state": self.controller.snapshot()}, False
            items = self._validate_items(payload.get("items"))
            drink = payload.get("drink")
            if drink is None and not items:
                raise ValueError("an order needs a drink or menu items")
            if payload.get("table_id") not in self.controller.table_ids:
                raise ValueError("unknown table_id")
            mission_payload = {k: v for k, v in payload.items() if k not in {"items", "client_ref"}}
            if "order_id" not in mission_payload:
                mission_payload["order_id"] = self._next_order_no()
            order, created = None, True
            if drink is not None:
                validate_order = getattr(self.backend, "validate_order", None)
                if validate_order is not None:
                    validate_order(mission_payload, existing_order_ids=set(self.controller.orders))
                order, created = self.controller.submit(mission_payload)
            elif mission_payload["order_id"] in self.tickets:
                return {"order": None, "ticket": self.tickets[mission_payload["order_id"]],
                        "state": self.controller.snapshot()}, False
            ticket = self.tickets.get(mission_payload["order_id"])
            if created and (items or client_ref or drink is None):
                staff_items = [item for item in items if not item["robot"]]
                ticket = {
                    "order_id": mission_payload["order_id"],
                    "table_id": mission_payload["table_id"],
                    "drink": drink,
                    "items": items,
                    "total": sum(item["qty"] * item["price"] for item in items),
                    "staff_state": "PREPARING" if staff_items else None,
                    "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                    "served_at": None,
                    "client_ref": client_ref,
                }
                self.tickets[ticket["order_id"]] = ticket
                self._persist_tickets()
            self._persist()
            return {"order": order.__dict__ if order else None, "ticket": ticket,
                    "state": self.controller.snapshot()}, created

    def _record_command(self, command: dict, elapsed_s: float) -> None:
        result = command.get("backend_result") or {}
        self.recent_commands.append({
            "at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "phase": command.get("phase"), "kind": command.get("kind"), "name": command.get("name"),
            "order_id": command.get("order_id"), "success": result.get("success"),
            "failure_code": result.get("failure_code") or None, "adapter": result.get("adapter") or result.get("mode"),
            "duration_s": round(elapsed_s, 3), "superseded": command.get("superseded", False),
        })

    def mark_served(self, order_id: str) -> dict:
        """Staff brought the food lines of a ticket to the table."""
        with self.lock:
            ticket = self.tickets.get(order_id)
            if ticket is None:
                raise KeyError(order_id)
            if ticket["staff_state"] is None:
                raise ValueError("this order has no staff-served items")
            ticket["staff_state"] = "SERVED"
            ticket["served_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            self._persist_tickets()
            return {"ticket": ticket}

    def cancel(self, order_id: str) -> dict:
        with self.lock:
            backend_cancel_requested = (
                self.backend.cancel_active()
                if self.controller.active_order_id == order_id
                else False
            )
            existing = self.controller.orders.get(order_id)
            if existing is not None and existing.state in {"PENDING", "ACCEPTED"}:
                self._drop_staff_items(order_id)
            order = self.controller.cancel(order_id)
            self._persist()
            return {
                "order": order.__dict__,
                "backend_cancel_requested": backend_cancel_requested,
                "state": self.controller.snapshot(),
            }

    def step(self, *, success: bool = True, failure: str | None = None) -> dict:
        if self.backend.mode in {'ros2_planned_session', 'ros2_planned_roundtrip'}:
            raise BackendBusyError('manual phase advance is disabled for persistent executor sessions')
        if not self.execution_lock.acquire(blocking=False):
            raise BackendBusyError("a backend command is already running")
        try:
            with self.lock:
                if self.emergency["active"]:
                    raise BackendBusyError("emergency stop is latched")
                phase_before = self.controller.phase
                order_before = self.controller.active_order_id
                command = self.runtime.step(success=success, failure=failure)
                stored_command = dict(command)
                stored_command.setdefault("phase", phase_before)
                stored_command.setdefault("order_id", order_before)
                stored_command["backend_result"] = {
                    "success": success,
                    "failure_code": failure or "",
                    "adapter": "manual_api_step",
                }
                self.last_backend_result = stored_command["backend_result"]
                self._record_command(stored_command, 0.0)
                self._persist(stored_command)
                return {"executed_command": command, "state": self.controller.snapshot()}
        finally:
            self.execution_lock.release()

    def execute_next(self) -> dict:
        """Execute the current command through the configured backend."""
        if not self.execution_lock.acquire(blocking=False):
            raise BackendBusyError("a backend command is already running")
        try:
            with self.lock:
                if self.emergency["active"]:
                    raise BackendBusyError("emergency stop is latched")
                phase_before = self.controller.phase
                order_before = self.controller.active_order_id
                active_order = self.controller.active_order
                mission_id = active_order.mission_id if active_order is not None else None
                phase_attempt = self.controller.phase_attempt
                command = self.controller.current_command()
                if active_order is not None:
                    command = {
                        **command,
                        "phase": phase_before,
                        "order_id": active_order.order_id,
                        "table_id": active_order.table_id,
                        "drink": active_order.drink,
                    }
                elif phase_before == "NAVIGATE_DOCK":
                    command = {**command, "phase": phase_before}
                self.runtime.commands.append(command)

            started = time.perf_counter()
            result = self.backend.execute(
                command,
                mission_id=mission_id,
                phase_attempt=phase_attempt,
            )
            elapsed_s = time.perf_counter() - started

            with self.lock:
                superseded = (
                    self.controller.phase != phase_before
                    or self.controller.active_order_id != order_before
                )
                if not superseded:
                    self.controller.advance(
                        success=result.success,
                        failure=result.failure_code or None,
                    )
                stored_command = dict(command)
                stored_command.setdefault("phase", phase_before)
                stored_command.setdefault("order_id", order_before)
                stored_command["backend_result"] = result.to_dict()
                stored_command["superseded"] = superseded
                self.last_backend_result = result.to_dict()
                self._record_command(stored_command, elapsed_s)
                self._persist(stored_command)
                return {
                    "executed_command": command,
                    "backend_result": result.to_dict(),
                    "superseded": superseded,
                    "state": self.controller.snapshot(),
                }
        finally:
            self.execution_lock.release()

    def close(self) -> None:
        if self.pose_feed is not None:
            self.pose_feed.close()
        self.backend.close()
        if self.store is not None:
            self.store.close()


def handler_factory(app: ServiceApplication, cors_origin: str | None = None) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "HoldTheFlowOrderServer/1.0"

        def end_headers(self) -> None:
            # 다른 주소(예: Netlify)에 올린 화면이 이 서버 API를 부를 때만 허용한다.
            if cors_origin:
                self.send_header("Access-Control-Allow-Origin", cors_origin)
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Vary", "Origin")
            super().end_headers()

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(HTTPStatus.NO_CONTENT if cors_origin else HTTPStatus.METHOD_NOT_ALLOWED)
            self.send_header("Content-Length", "0")
            self.end_headers()

        body_read = False

        def log_message(self, fmt: str, *args: Any) -> None:
            if urlparse(self.path).path == "/api/state":
                return
            print(f"{self.address_string()} - {fmt % args}")

        def _json(self, status: int, payload: dict) -> None:
            # 본문을 남긴 채 연결을 닫으면 Windows가 응답 전에 연결을 끊는다(WinError 10053)
            length = self.headers.get("Content-Length") or ""
            if not self.body_read and length.isdigit() and int(length) <= 16_384:
                self.rfile.read(int(length))
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            length_text = self.headers.get("Content-Length")
            if length_text is None:
                raise ValueError("Content-Length is required")
            try:
                length = int(length_text)
            except ValueError as exc:
                raise ValueError("Content-Length must be an integer") from exc
            if not 0 < length <= 16_384:
                raise ValueError("request body must be 1-16384 bytes")
            self.body_read = True
            try:
                payload = json.loads(self.rfile.read(length))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("request body must be UTF-8 JSON") from exc
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            return payload

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/api/health":
                self._json(HTTPStatus.OK, {"status": "ok", **app.backend.info()})
                return
            if path == "/api/state":
                self._json(HTTPStatus.OK, app.state())
                return
            if path in {"/api/orders/history", "/api/events"}:
                try:
                    raw_limit = parse_qs(parsed.query).get("limit", ["100"])[0]
                    limit = int(raw_limit)
                    if not 1 <= limit <= 1000:
                        raise ValueError
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "limit_must_be_1_to_1000"})
                    return
                kind = "orders" if path == "/api/orders/history" else "events"
                self._json(HTTPStatus.OK, {kind: app.history(kind, limit)})
                return
            if path in STATIC_FILES:
                file_path, content_type = STATIC_FILES[path]
                body = file_path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", f"{content_type}; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            asset = ASSET_PATH.fullmatch(path)
            if asset and (INDEX.parent / asset.group(1) / f"{asset.group(2)}.{asset.group(3)}").is_file():
                body = (INDEX.parent / asset.group(1) / f"{asset.group(2)}.{asset.group(3)}").read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", ASSET_TYPES[asset.group(3)])
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "max-age=3600")
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                if path == "/api/orders":
                    result, created = app.submit(self._read_json())
                    self._json(HTTPStatus.CREATED if created else HTTPStatus.OK, result)
                    return
                if path == "/api/step":
                    payload = self._read_json()
                    success = payload.get("success", True)
                    if not isinstance(success, bool):
                        raise ValueError("success must be boolean")
                    failure = payload.get("failure")
                    if failure is not None and (not isinstance(failure, str) or not failure):
                        raise ValueError("failure must be a nonempty string")
                    self._json(HTTPStatus.OK, app.step(success=success, failure=failure))
                    return
                if path == "/api/align/confirm":
                    confirm = getattr(app.backend, "confirm_alignment", None)
                    if confirm is None:
                        self._json(HTTPStatus.CONFLICT, {"error": "backend_has_no_alignment_gate"})
                        return
                    length = int(self.headers.get("Content-Length") or 0)
                    station = self._read_json().get("station") if length else None
                    if station is not None and not isinstance(station, str):
                        raise ValueError("station must be a string")
                    result = confirm(station)
                    self._json(HTTPStatus.OK if result.get("confirmed") else HTTPStatus.CONFLICT, result)
                    return
                if path == "/api/control":
                    paused = self._read_json().get("paused")
                    if not isinstance(paused, bool):
                        raise ValueError("paused must be boolean")
                    self._json(HTTPStatus.OK, app.set_auto_paused(paused))
                    return
                if path == "/api/drive/heartbeat":
                    self._json(HTTPStatus.OK, app.drive_heartbeat(self._read_json()))
                    return
                if path == "/api/drive/init":
                    self._json(HTTPStatus.OK, app.request_drive_init())
                    return
                if path == "/api/mission/start":
                    self._json(HTTPStatus.OK, app.start_mission())
                    return
                if path == "/api/emergency-stop":
                    self._json(HTTPStatus.OK, app.emergency_stop())
                    return
                if path == "/api/emergency-stop/reset":
                    self._json(HTTPStatus.OK, app.reset_emergency_stop())
                    return
                match = ORDER_PATH.fullmatch(path)
                if match:
                    action = {"cancel": app.cancel, "accept": app.accept, "reject": app.reject}[match.group(2)]
                    self._json(HTTPStatus.OK, action(match.group(1)))
                    return
                match = TICKET_PATH.fullmatch(path)
                if match:
                    self._json(HTTPStatus.OK, app.mark_served(match.group(1)))
                    return
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            except KeyError as exc:
                self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_order", "detail": str(exc)})
            except ValueError as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_request", "detail": str(exc)})
            except BackendBusyError as exc:
                self._json(HTTPStatus.CONFLICT, {"error": "backend_busy", "detail": str(exc)})
            except OrderStateError as exc:
                self._json(HTTPStatus.CONFLICT, {"error": "order_state_conflict", "detail": str(exc)})

    return Handler


def auto_step(app: ServiceApplication, interval_s: float, stop: threading.Event) -> None:
    while not stop.wait(interval_s):
        state = app.state()
        if state["auto_paused"]:
            continue
        if state["phase"] not in {"IDLE_AT_DOCK", "TERMINAL_HOLD"} or state["queue"] or state["active_order_id"]:
            try:
                app.execute_next()
            except BackendBusyError:
                continue


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--state-dir", type=Path, default=default_state_dir())
    parser.add_argument("--auto-step-s", type=float, default=0.75)
    parser.add_argument("--battery-percent", type=float, default=100.0)
    parser.add_argument(
        "--backend",
        choices=(
            "immediate",
            "ros2-mock",
            "ros2-planned-artifact",
            "ros2-planned-session",
            "ros2-planned-roundtrip",
            "rollout-service",
        ),
        default="immediate",
        help="manipulation backend; ROS modes are simulation-only, rollout-service moves the real arms",
    )
    parser.add_argument("--manipulation-timeout-s", type=float, default=30.0)
    parser.add_argument("--manip-url", default="http://127.0.0.1:8770",
                        help="rollout-service backend: address of tools/rollout/rollout_service.py")
    parser.add_argument("--auto-dispatch", action="store_true",
                        help="start a mission as soon as an order arrives (no operator accept or Start)")
    parser.add_argument("--no-align-confirm", action="store_true",
                        help="rollout-service backend: do not wait for the alignment-done signal before arm work")
    parser.add_argument("--accept-duration", action="store_true",
                        help="rollout-service backend: count a policy that hit its max duration as success")
    parser.add_argument(
        "--cors-origin",
        help="allow this origin (e.g. https://<site>.netlify.app) to call the API from a separately hosted page",
    )
    parser.add_argument(
        "--robot-pose-url",
        help="live map pose source for /control: the Pi navigation dashboard state, e.g. http://127.0.0.1:8090/api/state",
    )
    parser.add_argument(
        "--robot-ssh",
        help="open an SSH tunnel to this Pi (e.g. lim@192.168.0.159) for --robot-pose-url; needs key login",
    )
    parser.add_argument(
        "--executor-socket",
        type=Path,
        help="required Unix socket for ros2-planned-roundtrip",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not 0 < args.port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    if args.auto_step_s <= 0:
        raise ValueError("auto-step-s must be positive")
    if not math.isfinite(args.manipulation_timeout_s) or args.manipulation_timeout_s <= 0:
        raise ValueError("manipulation-timeout-s must be a positive finite number")
    backend: CommandBackend
    if args.backend == "ros2-mock":
        backend = Ros2ManipulationBackend(timeout_sec=args.manipulation_timeout_s)
    elif args.backend == "ros2-planned-artifact":
        backend = Ros2PlannedArtifactBackend(timeout_sec=args.manipulation_timeout_s)
    elif args.backend == "ros2-planned-session":
        backend = Ros2PlannedSessionBackend(timeout_sec=args.manipulation_timeout_s)
    elif args.backend == "ros2-planned-roundtrip":
        if args.executor_socket is None:
            raise ValueError("--executor-socket is required for ros2-planned-roundtrip")
        backend = Ros2PlannedRoundtripBackend(
            executor_socket=args.executor_socket,
            timeout_sec=args.manipulation_timeout_s,
        )
    elif args.backend == "rollout-service":
        backend = RolloutServiceBackend(
            url=args.manip_url,
            require_alignment=not args.no_align_confirm,
            accept_duration=args.accept_duration,
        )
        status = backend.service_status(max_age_s=0)
        print(f"manipulation service {args.manip_url}: {status.get('state')}", flush=True)
    else:
        backend = ImmediateBackend()
    app = ServiceApplication(
        MissionController(
            battery_percent=args.battery_percent,
            require_dock_return=args.backend == "ros2-planned-roundtrip",
            require_operator=not args.auto_dispatch,
        ),
        args.state_dir,
        backend,
    )
    if args.robot_pose_url:
        app.pose_feed = RobotPoseFeed(args.robot_pose_url, ssh_target=args.robot_ssh)
        app.pose_feed.start()
        print(f"robot pose: {args.robot_pose_url}" + (f" via ssh {args.robot_ssh}" if args.robot_ssh else ""), flush=True)
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(app, args.cors_origin))
    stop = threading.Event()
    worker = threading.Thread(target=auto_step, args=(app, args.auto_step_s, stop), daemon=True)
    worker.start()
    print(f"service order dashboard: http://{args.host}:{server.server_port}", flush=True)
    print(f"guest order page: http://{args.host}:{server.server_port}/order?table=1", flush=True)
    print(f"control page: http://{args.host}:{server.server_port}/control", flush=True)
    if backend.info().get("hardware_accessed"):
        print(f"mode: {backend.mode} (REAL ARMS: manipulation phases call the rollout service)", flush=True)
    else:
        print(f"mode: {backend.mode} (hardware disabled; simulator access depends on backend)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        backend.cancel_active()
        server.shutdown()
        server.server_close()
        worker.join(timeout=max(1.0, args.manipulation_timeout_s + 1.0))
        if worker.is_alive():
            print("warning: backend worker did not stop before shutdown timeout", flush=True)
        else:
            app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
