import importlib.util
import json
from http.server import ThreadingHTTPServer
import os
from pathlib import Path
import select
import socket
import sqlite3
import subprocess
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from src.mission.service_mission_control import MissionController, OrderStateError
import src.mission.service_execution_backend as service_execution_backend
from src.mission.service_execution_backend import BackendResult, ImmediateBackend, Ros2ManipulationBackend, Ros2PlannedRoundtripBackend, Ros2PlannedSessionBackend
from apps.service_order.service_order_server import API_JS, BackendBusyError, CONTROL_JS, CONTROL_PAGE, DASHBOARD_JS, INDEX, ORDER_JS, ORDER_PAGE, ServiceApplication, auto_step, default_state_dir, handler_factory
from src.mission.service_order_store import ServiceOrderStore


def post_json(url, payload):
    body = json.dumps(payload).encode()
    request = Request(url, data=body, headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=3) as response:
        return response.status, json.loads(response.read())


@pytest.fixture
def server(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield app, f"http://127.0.0.1:{httpd.server_port}", tmp_path / "state"
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=3)
    app.close()


def test_dashboard_and_health_are_served(server):
    _, base, _ = server
    with urlopen(base + "/", timeout=3) as response:
        html = response.read().decode()
    assert "양팔 물 서빙 관제" in html
    assert "data-table=\"table_4\"" in html
    with urlopen(base + "/api/health", timeout=3) as response:
        health = json.loads(response.read())
    assert health == {
        "status": "ok",
        "mode": "simulation_dry_run",
        "manipulation": "immediate_success",
        "navigation": "immediate_success",
        "charge": "immediate_success",
        "active_goal": False,
        "hardware_accessed": False,
    }

    with urlopen(base + "/dashboard.js", timeout=3) as response:
        javascript = response.read().decode()
    assert "renderMap" in javascript
    assert "execution-backend" in javascript


def test_default_cpu_server_starts_without_ros_pythonpath(tmp_path):
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    process = subprocess.Popen(
        [
            "python3",
            "-m", "apps.service_order.service_order_server",
            "--port",
            str(port),
            "--state-dir",
            str(tmp_path / "cpu-server"),
        ],
        cwd=Path(__file__).resolve().parents[2],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        ready, _, _ = select.select([process.stdout], [], [], 5.0)
        assert ready, process.stderr.read()
        assert process.stdout.readline().startswith("service order dashboard: http://127.0.0.1:")
    finally:
        process.terminate()
        process.wait(timeout=5)


def test_order_api_is_idempotent_and_persists_state(server):
    _, base, state_dir = server
    payload = {"order_id": "WEB-TEST-1", "table_id": "table_2", "drink": "HOT_WATER"}
    first_status, first = post_json(base + "/api/orders", payload)
    second_status, second = post_json(base + "/api/orders", payload)
    assert first_status == 201 and second_status == 200
    assert first["order"]["order_id"] == second["order"]["order_id"]
    assert first["state"]["active_order_id"] == "WEB-TEST-1"
    stored = json.loads((state_dir / "snapshot.json").read_text())
    assert stored["active_order_id"] == "WEB-TEST-1"
    assert (state_dir / "events.jsonl").is_file()
    database = state_dir / "service_mission_control.sqlite3"
    assert database.is_file()
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT COUNT(*) FROM orders").fetchone()[0] == 1
    with urlopen(base + "/api/orders/history?limit=1", timeout=3) as response:
        history = json.loads(response.read())
    assert [item["order_id"] for item in history["orders"]] == ["WEB-TEST-1"]


def test_step_failure_and_cancel_endpoints(server):
    _, base, state_dir = server
    post_json(base + "/api/orders", {
        "order_id": "ACTIVE", "table_id": "table_1", "drink": "COLD_WATER"
    })
    post_json(base + "/api/orders", {
        "order_id": "WAITING", "table_id": "table_3", "drink": "HOT_WATER"
    })
    status, canceled = post_json(base + "/api/orders/WAITING/cancel", {})
    assert status == 200 and canceled["order"]["state"] == "CANCELED"
    status, stepped = post_json(base + "/api/step", {"success": True})
    assert status == 200
    assert stepped["executed_command"]["destination"] == "kitchen"
    with sqlite3.connect(state_dir / "service_mission_control.sqlite3") as connection:
        stored = connection.execute(
            "SELECT order_id, phase, kind FROM commands ORDER BY command_sequence DESC LIMIT 1"
        ).fetchone()
    assert stored == ("ACTIVE", "NAVIGATE_KITCHEN", "navigate")


def test_invalid_order_returns_400(server):
    _, base, _ = server
    with pytest.raises(HTTPError) as error:
        post_json(base + "/api/orders", {
            "order_id": "BAD", "table_id": "table_99", "drink": "COLD_WATER"
        })
    assert error.value.code == 400
    with pytest.raises(HTTPError) as limit_error:
        urlopen(base + "/api/events?limit=0", timeout=3)
    assert limit_error.value.code == 400


def test_static_dashboard_has_no_external_runtime_dependencies():
    html = Path(INDEX).read_text()
    javascript = Path(DASHBOARD_JS).read_text()
    assert "https://" not in html and "http://" not in html
    assert "fetch(\"http" not in javascript and "request(\"http" not in javascript
    assert "request(\"/api/orders\"" in javascript
    assert ".innerHTML" not in javascript


def test_guest_and_control_pages_are_served(server):
    _, base, _ = server
    pages = {
        "/order": "MS RESTAURANT",
        "/order.js": "STEP_BY_PHASE",
        "/control": "로봇 운영 콘솔",
        "/control.js": "SERVICE_PHASES",
        "/api.js": "orderApi",
        "/config.js": "ORDER_API_BASE",
    }
    for path, marker in pages.items():
        with urlopen(base + path, timeout=3) as response:
            assert marker in response.read().decode()


def test_control_page_keeps_operator_controls_in_one_screen():
    html = CONTROL_PAGE.read_text(encoding="utf-8")
    javascript = CONTROL_JS.read_text(encoding="utf-8")
    for control in ("drive-init", "start", "stop", "estop-reset", "align-confirm", "queue", "staff",
                    "topics", "commands", "events", "t-step", "t-fail", "s-drive", "s-location", "s-mission", "s-manip"):
        assert f'id="{control}"' in html
    for removed in ("pause", "history", "f-submit", "b-left", "b-right", "b-pi", "s-battery", "s-nav"):
        assert f'id="{removed}"' not in html
    assert "overflow: hidden" in html.split("@media")[0]
    assert "JSON.stringify(cmd" not in javascript
    assert "battery_percent" not in javascript and "BATTERY_RANGE_V" not in javascript
    assert "orderTally" not in javascript


def test_guest_and_control_pages_have_no_external_runtime_dependencies():
    for page, script in ((ORDER_PAGE, ORDER_JS), (CONTROL_PAGE, CONTROL_JS), (CONTROL_PAGE, API_JS)):
        html = Path(page).read_text(encoding="utf-8")
        javascript = Path(script).read_text(encoding="utf-8")
        assert "https://" not in html and "http://" not in html
        assert "fetch(\"http" not in javascript
        assert ".innerHTML" not in javascript


def test_server_assigns_daily_order_numbers_and_keeps_food_tickets(server):
    app, base, state_dir = server
    _, first = post_json(base + "/api/orders", {"client_ref": "web-a", "table_id": "table_1", "drink": "COLA",
                                                "items": [{"name": "콜라", "qty": 1, "price": 3000, "robot": True},
                                                          {"name": "감자튀김", "qty": 2, "price": 5000}]})
    number = first["ticket"]["order_id"]
    assert len(number) == 9 and number.isdigit() and number.endswith("001")
    assert first["order"]["order_id"] == number and first["order"]["drink"] == "COLA"
    assert first["ticket"]["total"] == 13000 and first["ticket"]["staff_state"] == "PREPARING"

    status, again = post_json(base + "/api/orders", {"client_ref": "web-a", "table_id": "table_1", "drink": "COLA"})
    assert status == 200 and again["ticket"]["order_id"] == number

    _, food_only = post_json(base + "/api/orders", {"table_id": "table_2", "items": [{"name": "스테이크", "qty": 1, "price": 24000}]})
    second = food_only["ticket"]["order_id"]
    assert second == number[:6] + "002" and food_only["order"] is None
    assert second not in app.state()["queue"]

    _, served = post_json(base + f"/api/tickets/{second}/served", {})
    assert served["ticket"]["staff_state"] == "SERVED"
    tickets = json.loads((state_dir / "tickets.json").read_text(encoding="utf-8"))
    assert {t["order_id"] for t in tickets} == {number, second}

    with pytest.raises(HTTPError) as error:
        post_json(base + "/api/orders", {"table_id": "table_1"})
    assert error.value.code == 400


def test_new_order_numbers_skip_slots_shown_for_legacy_orders(server):
    app, base, _ = server
    # 9자리 형식 전의 주문 1건은 관제 화면에서 오늘 001로 보인다. 새 주문은 002부터.
    post_json(base + "/api/orders", {"order_id": "WEB-20261027-090000-ab12", "table_id": "table_1", "drink": "COLD_WATER"})
    _, created = post_json(base + "/api/orders", {"table_id": "table_2", "drink": "HOT_WATER"})
    assert created["order"]["order_id"].endswith("002")


def test_backend_commands_are_logged_for_the_console(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    try:
        app.submit({"table_id": "table_1", "drink": "COLD_WATER"})
        app.execute_next()
        log = app.state()["recent_commands"]
        assert log[-1]["phase"] == "NAVIGATE_KITCHEN" and log[-1]["success"] is True
        assert log[-1]["duration_s"] >= 0 and app.state()["server"]["pid"] > 0
    finally:
        app.close()


def test_menu_photos_and_map_assets_are_served(server):
    _, base, _ = server
    with urlopen(base + "/map/classroom.json", timeout=3) as response:
        meta = json.loads(response.read())
    assert meta["resolution_m"] == 0.05 and {s["id"] for s in meta["stations"]} >= {"dock", "kitchen", "table_1"}
    with urlopen(base + "/map/classroom.png", timeout=3) as response:
        assert response.headers["Content-Type"] == "image/png"
    with urlopen(base + "/fonts/jua.woff", timeout=3) as response:
        assert response.headers["Content-Type"] == "font/woff"
    with pytest.raises(HTTPError) as error:
        urlopen(base + "/map/..%2Fsecret.json", timeout=3)
    assert error.value.code == 404


def test_robot_pose_feed_reads_navigation_dashboard(tmp_path):
    from http.server import BaseHTTPRequestHandler
    import time
    from apps.service_order.service_order_server import RobotPoseFeed

    class Dashboard(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps({"navigation": {"localization": {
                "x_m": 0.02, "y_m": -0.08, "yaw_deg": -1.5, "frame_id": "map", "live": True, "age_s": 0.3}}}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    dashboard = ThreadingHTTPServer(("127.0.0.1", 0), Dashboard)
    threading.Thread(target=dashboard.serve_forever, daemon=True).start()
    app = ServiceApplication(MissionController(), tmp_path / "state")
    app.pose_feed = RobotPoseFeed(f"http://127.0.0.1:{dashboard.server_port}/api/state", interval_s=0.1)
    app.pose_feed.start()
    try:
        deadline = time.time() + 3
        while time.time() < deadline and not app.state()["robot_pose"].get("live"):
            time.sleep(0.05)
        pose = app.state()["robot_pose"]
        assert pose["live"] and pose["x_m"] == 0.02 and pose["yaw_deg"] == -1.5
        nav = app.state()["robot_nav"]
        assert nav["connected"] and nav["snapshot"]["navigation"]["localization"]["x_m"] == 0.02
        dashboard.shutdown()
        dashboard.server_close()
        deadline = time.time() + 5
        while time.time() < deadline and app.state()["robot_pose"].get("live"):
            time.sleep(0.05)
        lost = app.state()["robot_pose"]
        assert not lost["live"] and lost["last"]["x_m"] == 0.02
    finally:
        app.close()


def test_cors_headers_only_when_origin_configured(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    servers = {}
    for name, origin in (("closed", None), ("open", "https://example.netlify.app")):
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app, origin))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        servers[name] = httpd
    try:
        base = f"http://127.0.0.1:{servers['open'].server_port}"
        with urlopen(base + "/api/health", timeout=3) as response:
            assert response.headers["Access-Control-Allow-Origin"] == "https://example.netlify.app"
        with urlopen(Request(base + "/api/orders", method="OPTIONS"), timeout=3) as response:
            assert response.status == 204
        closed = f"http://127.0.0.1:{servers['closed'].server_port}"
        with urlopen(closed + "/api/health", timeout=3) as response:
            assert response.headers["Access-Control-Allow-Origin"] is None
        with pytest.raises(HTTPError) as error:
            urlopen(Request(closed + "/api/orders", method="OPTIONS"), timeout=3)
        assert error.value.code == 405
    finally:
        for httpd in servers.values():
            httpd.shutdown()
            httpd.server_close()
        app.close()


def test_netlify_bundle_contains_pages_and_demo_config(tmp_path):
    from apps.service_order.build_netlify import build

    build(tmp_path / "site", "demo")
    names = {path.name for path in (tmp_path / "site").iterdir()}
    assert {"order.html", "order.js", "control.html", "control.js", "api.js", "config.js", "_redirects"} <= names
    assert '"demo"' in (tmp_path / "site/config.js").read_text(encoding="utf-8")


def test_control_pause_stops_auto_step_until_resumed(server):
    app, base, _ = server
    post_json(base + "/api/orders", {"order_id": "PAUSE-1", "table_id": "table_1", "drink": "COLD_WATER"})
    status, body = post_json(base + "/api/control", {"paused": True})
    assert status == 200 and body == {"auto_paused": True}
    stop = threading.Event()
    worker = threading.Thread(target=auto_step, args=(app, 0.01, stop), daemon=True)
    worker.start()
    stop.wait(0.2)
    assert app.state()["phase"] == "NAVIGATE_KITCHEN"
    post_json(base + "/api/control", {"paused": False})
    stop.wait(0.3)
    stop.set()
    worker.join(timeout=3)
    assert app.state()["auto_paused"] is False
    assert app.state()["phase"] != "NAVIGATE_KITCHEN"
    with pytest.raises(HTTPError) as error:
        post_json(base + "/api/control", {"paused": "yes"})
    assert error.value.code == 400


def test_restart_restores_active_mission_and_history(tmp_path):
    state_dir = tmp_path / "persistent-state"
    first = ServiceApplication(MissionController(), state_dir)
    first.submit({"order_id": "RESTORE-1", "table_id": "table_4", "drink": "HOT_WATER"})
    first.step()
    before = first.state()
    assert before["phase"] == "ALIGN_KITCHEN"
    first.close()

    second = ServiceApplication(MissionController(), state_dir)
    after = second.state()
    assert after["active_order_id"] == "RESTORE-1"
    assert after["phase"] == "ALIGN_KITCHEN"
    assert after["battery_percent"] == before["battery_percent"]
    assert after["persistence"]["backend"] == "sqlite"
    assert any(event["kind"] == "CONTROLLER_RESTORED" for event in after["events"])
    assert second.history("orders", 10)[0]["order_id"] == "RESTORE-1"
    second.close()


def test_default_state_dir_uses_persistent_user_state(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "xdg-state"))
    assert default_state_dir() == tmp_path / "xdg-state/bimanual-robot/service-order-server"


def test_store_rejects_unknown_schema_version(tmp_path):
    state_dir = tmp_path / "future-schema"
    state_dir.mkdir()
    database = state_dir / "service_mission_control.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        connection.execute("INSERT INTO metadata(key, value) VALUES('schema_version', '99')")
    with pytest.raises(RuntimeError, match="unsupported service-order database schema"):
        ServiceOrderStore(state_dir)


@pytest.mark.parametrize("timeout_sec", [0.0, -1.0, float("nan"), float("inf")])
def test_ros2_backend_rejects_non_positive_or_non_finite_timeout(timeout_sec):
    with pytest.raises(ValueError, match="positive finite"):
        Ros2ManipulationBackend(timeout_sec=timeout_sec)


class FailureBackend(ImmediateBackend):
    mode = "test_failure"

    def execute(self, command, *, mission_id, phase_attempt):
        del mission_id, phase_attempt
        if command["kind"] == "manipulate":
            return BackendResult(False, "POSE_TOLERANCE", self.mode)
        return BackendResult(True, adapter=self.mode)


def test_persistent_executor_cannot_be_advanced_by_manual_step(tmp_path):
    backend = ImmediateBackend()
    backend.mode = 'ros2_planned_session'
    app = ServiceApplication(MissionController(), tmp_path, backend)
    try:
        with pytest.raises(BackendBusyError, match='manual phase advance'):
            app.step()
        assert app.controller.phase == 'IDLE_AT_DOCK'
    finally:
        app.close()


def test_executor_observation_is_retained_in_status_and_snapshot(tmp_path):
    class ObservedBackend(ImmediateBackend):
        def info(self):
            return {**super().info(), 'simulator_accessed': True, 'executor_kind': 'isaac_physics'}
    app = ServiceApplication(MissionController(), tmp_path, ObservedBackend())
    try:
        assert app.state()['simulator_accessed'] is True
        saved = json.loads((tmp_path / 'snapshot.json').read_text())
        assert saved['simulator_accessed'] is True
        assert saved['execution_backend']['executor_kind'] == 'isaac_physics'
    finally:
        app.close()


def test_backend_result_controls_phase_and_is_persisted(tmp_path):
    state_dir = tmp_path / "backend-result"
    app = ServiceApplication(MissionController(), state_dir, FailureBackend())
    app.submit({"order_id": "BACKEND-1", "table_id": "table_1", "drink": "COLD_WATER"})
    navigation = app.execute_next()
    assert navigation["backend_result"]["success"] is True
    assert app.controller.phase == "ALIGN_KITCHEN"

    manipulation = app.execute_next()
    assert manipulation["backend_result"]["failure_code"] == "POSE_TOLERANCE"
    assert app.controller.phase == "ALIGN_KITCHEN"
    assert app.controller.phase_attempt == 2
    with sqlite3.connect(state_dir / "service_mission_control.sqlite3") as connection:
        payload = json.loads(connection.execute(
            "SELECT payload_json FROM commands ORDER BY command_sequence DESC LIMIT 1"
        ).fetchone()[0])
    assert payload["backend_result"]["failure_code"] == "POSE_TOLERANCE"
    app.close()


class BlockingCancelBackend(ImmediateBackend):
    mode = "test_blocking_cancel"

    def __init__(self):
        self.started = threading.Event()
        self.canceled = threading.Event()

    def execute(self, command, *, mission_id, phase_attempt):
        del command, mission_id, phase_attempt
        self.started.set()
        assert self.canceled.wait(timeout=3)
        return BackendResult(False, "CANCELED", self.mode)

    def cancel_active(self):
        self.canceled.set()
        return True


def test_active_order_cancel_propagates_and_backend_result_is_superseded(tmp_path):
    backend = BlockingCancelBackend()
    app = ServiceApplication(MissionController(), tmp_path / "cancel", backend)
    app.submit({"order_id": "CANCEL-1", "table_id": "table_1", "drink": "COLD_WATER"})
    app.controller.advance()
    assert app.controller.phase == "ALIGN_KITCHEN"

    result = {}
    worker = threading.Thread(target=lambda: result.update(app.execute_next()))
    worker.start()
    assert backend.started.wait(timeout=2)
    canceled = app.cancel("CANCEL-1")
    worker.join(timeout=3)

    assert not worker.is_alive()
    assert canceled["backend_cancel_requested"] is True
    assert canceled["order"]["state"] == "CANCELED"
    assert result["superseded"] is True
    assert result["backend_result"]["failure_code"] == "CANCELED"
    assert app.controller.orders["CANCEL-1"].state == "CANCELED"
    app.close()


class RoundtripConstraintBackend(ImmediateBackend):
    mode = "ros2_planned_roundtrip"

    def validate_order(self, payload, *, existing_order_ids):
        if payload.get("table_id") != "table_1" or payload.get("drink") != "COLD_WATER":
            raise ValueError("unsupported roundtrip order")
        if existing_order_ids and payload.get("order_id") not in existing_order_ids:
            raise ValueError("one order per world")


def _advance_application_to_dock(app):
    while app.controller.phase != "NAVIGATE_DOCK":
        app.execute_next()


def test_roundtrip_application_completes_only_after_dock_observation(tmp_path):
    app = ServiceApplication(
        MissionController(require_dock_return=True),
        tmp_path / "dock-success",
        RoundtripConstraintBackend(),
    )
    try:
        app.submit({"order_id": "RETURN", "table_id": "table_1", "drink": "COLD_WATER"})
        _advance_application_to_dock(app)
        order = app.controller.orders["RETURN"]
        assert order.state == "RUNNING"
        assert app.controller.active_order_id == "RETURN"

        result = app.execute_next()
        assert result["executed_command"]["phase"] == "NAVIGATE_DOCK"
        assert order.state == "SUCCEEDED"
        assert app.controller.phase == "IDLE_AT_DOCK"
    finally:
        app.close()


class BlockingDockBackend(RoundtripConstraintBackend):
    def __init__(self):
        self.started = threading.Event()
        self.canceled = threading.Event()

    def execute(self, command, *, mission_id, phase_attempt):
        del mission_id, phase_attempt
        if command.get("phase") == "NAVIGATE_DOCK":
            self.started.set()
            assert self.canceled.wait(timeout=3)
        return BackendResult(True, adapter=self.mode)

    def cancel_active(self):
        self.canceled.set()
        return True


class FailingDockBackend(RoundtripConstraintBackend):
    def __init__(self):
        self.dock_calls = 0

    def execute(self, command, *, mission_id, phase_attempt):
        del mission_id, phase_attempt
        if command.get("phase") == "NAVIGATE_DOCK":
            self.dock_calls += 1
            return BackendResult(False, "DOCK_BLOCKED", self.mode)
        return BackendResult(True, adapter=self.mode)


def test_roundtrip_dock_failure_stops_auto_execution(tmp_path):
    backend = FailingDockBackend()
    app = ServiceApplication(
        MissionController(require_dock_return=True),
        tmp_path / "dock-failure",
        backend,
    )
    try:
        app.submit({"order_id": "FAIL-DOCK", "table_id": "table_1", "drink": "COLD_WATER"})
        _advance_application_to_dock(app)
        result = app.execute_next()
        assert result["backend_result"]["failure_code"] == "DOCK_BLOCKED"
        assert app.controller.orders["FAIL-DOCK"].state == "FAILED"
        assert app.controller.phase == "TERMINAL_HOLD"

        stop = threading.Event()
        worker = threading.Thread(target=auto_step, args=(app, 0.01, stop))
        worker.start()
        stop.wait(0.05)
        stop.set()
        worker.join(timeout=1)
        assert backend.dock_calls == 1
    finally:
        app.close()


def test_roundtrip_dock_cancel_is_forwarded_and_late_success_is_superseded(tmp_path):
    backend = BlockingDockBackend()
    app = ServiceApplication(
        MissionController(require_dock_return=True),
        tmp_path / "dock-cancel",
        backend,
    )
    try:
        app.submit({"order_id": "CANCEL-DOCK", "table_id": "table_1", "drink": "COLD_WATER"})
        _advance_application_to_dock(app)
        result = {}
        worker = threading.Thread(target=lambda: result.update(app.execute_next()))
        worker.start()
        assert backend.started.wait(timeout=2)

        canceled = app.cancel("CANCEL-DOCK")
        worker.join(timeout=3)
        assert not worker.is_alive()
        assert canceled["backend_cancel_requested"] is True
        assert canceled["order"]["state"] == "CANCELED"
        assert result["superseded"] is True
        assert app.controller.phase == "TERMINAL_HOLD"
        assert app.controller.orders["CANCEL-DOCK"].state == "CANCELED"
    finally:
        app.close()


def test_roundtrip_rejects_unsupported_and_second_orders(tmp_path):
    app = ServiceApplication(
        MissionController(require_dock_return=True),
        tmp_path / "fresh",
        RoundtripConstraintBackend(),
    )
    try:
        with pytest.raises(ValueError, match="unsupported"):
            app.submit({"order_id": "HOT", "table_id": "table_1", "drink": "HOT_WATER"})
        app.submit({"order_id": "ONLY", "table_id": "table_1", "drink": "COLD_WATER"})
        duplicate, created = app.submit(
            {"order_id": "ONLY", "table_id": "table_1", "drink": "COLD_WATER"}
        )
        assert created is False and duplicate["order"]["order_id"] == "ONLY"
        with pytest.raises(ValueError, match="one order"):
            app.submit({"order_id": "SECOND", "table_id": "table_1", "drink": "COLD_WATER"})
    finally:
        app.close()


def test_roundtrip_refuses_persisted_controller_world(tmp_path):
    state_dir = tmp_path / "persisted"
    first = ServiceApplication(MissionController(), state_dir)
    first.close()
    with pytest.raises(RuntimeError, match="cannot resume"):
        ServiceApplication(MissionController(), state_dir, RoundtripConstraintBackend())


def _roundtrip_backend_without_ros(tmp_path):
    backend = Ros2PlannedRoundtripBackend.__new__(Ros2PlannedRoundtripBackend)
    backend.executor_socket = tmp_path / "executor.sock"
    backend.timeout_sec = 2.0
    backend._identity = None
    backend._identity_lock = threading.Lock()
    backend._executor_evidence = {
        "executor_id": "executor-test-1",
        "executor_kind": "isaac_physics",
        "simulator_accessed": True,
        "scope": "dock_roundtrip",
    }
    backend._executor_id = "executor-test-1"
    return backend


def test_roundtrip_navigation_keeps_identity_for_dock(monkeypatch, tmp_path):
    backend = _roundtrip_backend_without_ros(tmp_path)
    calls = []

    def fake_execute_phase(socket_path, **payload):
        calls.append((socket_path, payload))
        return {
            "success": True,
            "failure_code": "",
            "message": "ok",
            "session_state": "WAITING",
            "executor_kind": "isaac_physics",
            "simulator_accessed": True,
            "hardware_accessed": False,
        }

    monkeypatch.setattr(service_execution_backend, "execute_phase", fake_execute_phase)
    identity = {"order_id": "ORD-1", "table_id": "table_1", "drink": "COLD_WATER"}
    first = backend.execute(
        {"kind": "navigate", "phase": "NAVIGATE_KITCHEN", **identity},
        mission_id="MIS-1",
        phase_attempt=1,
    )
    dock = backend.execute(
        {"kind": "navigate", "phase": "NAVIGATE_DOCK"},
        mission_id=None,
        phase_attempt=1,
    )
    assert first.success and dock.success
    assert [call[1]["phase_id"] for call in calls] == ["NAVIGATE_KITCHEN", "NAVIGATE_DOCK"]
    assert calls[1][1]["mission_id"] == "MIS-1"
    assert calls[1][1]["order_id"] == "ORD-1"
    assert calls[1][1]["executor_id"] == "executor-test-1"


def test_roundtrip_timeout_requests_cancel_and_marks_unknown(monkeypatch, tmp_path):
    backend = _roundtrip_backend_without_ros(tmp_path)
    monkeypatch.setattr(
        service_execution_backend,
        "execute_phase",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            service_execution_backend._planned_ipc_module().PlannedIpcError(
                "IPC_TIMEOUT", "lost response"
            )
        ),
    )
    canceled = []
    monkeypatch.setattr(
        service_execution_backend,
        "cancel_session",
        lambda path, mission_id, executor_id, timeout_sec: (
            canceled.append((mission_id, executor_id)) or True
        ),
    )
    result = backend.execute(
        {
            "kind": "navigate",
            "phase": "NAVIGATE_KITCHEN",
            "order_id": "ORD-1",
            "table_id": "table_1",
            "drink": "COLD_WATER",
        },
        mission_id="MIS-1",
        phase_attempt=1,
    )
    assert result.failure_code == "IPC_TIMEOUT_RESULT_UNKNOWN"
    assert result.action_result == {
        "message": "lost response",
        "result_state": "unknown",
        "cancel_requested": True,
    }
    assert canceled == [("MIS-1", "executor-test-1")]


def test_roundtrip_constructor_requires_fresh_scoped_physics_executor(monkeypatch, tmp_path):
    status = {
        "success": True,
        "executor_id": "executor-test-1",
        "executor_kind": "isaac_physics",
        "scope": "dock_roundtrip",
        "simulator_accessed": True,
        "hardware_accessed": False,
        "mission_id": None,
        "session_state": "WAITING",
    }
    monkeypatch.setattr(service_execution_backend, "call_executor", lambda *args, **kwargs: status)
    monkeypatch.setattr(
        Ros2PlannedSessionBackend,
        "__init__",
        lambda self, timeout_sec: setattr(self, "timeout_sec", timeout_sec),
    )
    backend = Ros2PlannedRoundtripBackend(
        executor_socket=tmp_path / "executor.sock",
        timeout_sec=4.0,
    )
    assert backend._executor_evidence == status
    assert backend._executor_id == "executor-test-1"

    status["scope"] = "manipulation_only"
    with pytest.raises(RuntimeError, match="scope"):
        Ros2PlannedRoundtripBackend(
            executor_socket=tmp_path / "executor.sock",
            timeout_sec=4.0,
        )


def _gated_app(tmp_path):
    return ServiceApplication(MissionController(require_operator=True), tmp_path / "state", ImmediateBackend())


def _accepted_order(app):
    order_id = app.submit({"table_id": "table_1", "drink": "COLD_WATER"})[0]["order"]["order_id"]
    app.accept(order_id)
    return order_id


def _link_drive(app, state=None):
    app.drive_heartbeat({"state": state, "phase": None, "mission_id": None, "error": None, "dry_run": False})


def test_drive_init_needs_a_connected_idle_drive_pc(tmp_path):
    app = _gated_app(tmp_path)
    with pytest.raises(OrderStateError, match="drive_not_connected"):
        app.request_drive_init()
    assert app.state()["drive_init"]["ready"] is False

    # 주문은 컵배치에서 끝나지만 차는 아직 도크로 돌아오는 중이다.
    _link_drive(app, "running")
    assert app.state()["active_order_id"] is None
    with pytest.raises(OrderStateError, match="mission_running"):
        app.request_drive_init()

    _link_drive(app, "succeeded")
    app.emergency_stop()
    with pytest.raises(OrderStateError, match="emergency_stop"):
        app.request_drive_init()
    app.reset_emergency_stop()

    assert app.state()["drive_init"]["ready"] is True
    _accepted_order(app)
    assert app.state()["drive_init"]["ready"] is True
    app.start_mission()
    with pytest.raises(OrderStateError, match="mission_running"):
        app.request_drive_init()
    app.close()


def test_drive_init_blocks_start_until_its_result_and_start_clears_it(tmp_path):
    app = _gated_app(tmp_path)
    _link_drive(app)
    _accepted_order(app)
    request_id = app.request_drive_init()["drive_init"]["id"]
    assert app.state()["drive_init"]["state"] == "requested"
    assert app.state()["start"]["reason"] == "drive_init_running"
    with pytest.raises(OrderStateError, match="drive_init_running"):
        app.request_drive_init()

    app.drive_heartbeat({"init": {"id": request_id, "state": "running", "message": None}})
    assert app.state()["drive_init"]["state"] == "running"
    assert app.state()["start"]["ready"] is False

    app.drive_heartbeat({"init": {"id": request_id, "state": "succeeded", "message": "init complete"}})
    state = app.state()
    assert state["drive_init"]["state"] == "succeeded"
    assert state["drive_init"]["message"] == "init complete"
    assert state["start"]["ready"] is True

    app.start_mission()
    assert app.state()["drive_init"]["state"] == "idle"
    app.close()


def test_unclaimed_or_lost_drive_init_becomes_a_failure(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    _link_drive(app)
    request_id = app.request_drive_init()["drive_init"]["id"]
    app.drive_init["requested"] -= 60
    assert app.state()["drive_init"]["state"] == "failed"

    _link_drive(app)
    request_id = app.request_drive_init()["drive_init"]["id"]
    app.drive_heartbeat({"init": {"id": "stale", "state": "succeeded", "message": None}})
    assert app.state()["drive_init"]["state"] == "requested"
    app.drive_heartbeat({"init": {"id": request_id, "state": "running", "message": None}})
    app.drive_heartbeat({"init": None})
    assert app.state()["drive_init"]["state"] == "failed"
    app.close()


def test_control_page_init_reaches_the_drive_pc_and_returns_its_result(server, tmp_path, monkeypatch):
    app, base, _ = server
    spec = importlib.util.spec_from_file_location(
        "water_service_init", Path(__file__).resolve().parents[2] / "tools/jdamr/water_service.py")
    water_service = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(water_service)
    script = tmp_path / "jdamr_depart.py"
    script.write_text('print("init complete: no motion sent")\n')
    monkeypatch.setattr(water_service, "DEPART_HELPER", script)
    drive = water_service.WaterServiceController(inference_url=base, state_directory=tmp_path / "drive")

    with pytest.raises(HTTPError) as refused:
        post_json(f"{base}/api/drive/init", {})
    assert refused.value.code == 409

    drive.report_web(dry_run=False)
    status, body = post_json(f"{base}/api/drive/init", {})
    assert status == 200
    request_id = body["drive_init"]["id"]

    assert drive.follow_web(drive.read_web_state()) == f"init {request_id}"
    drive.report_web(dry_run=False)
    drive._init_thread.join(timeout=10)
    drive.report_web(dry_run=False)
    result = drive.read_web_state()["drive_init"]
    assert (result["state"], result["message"]) == ("succeeded", "init complete: no motion sent")
    assert result["ready"] is True


def test_control_page_has_the_drive_init_button():
    assert 'id="drive-init"' in CONTROL_PAGE.read_text(encoding="utf-8")
    assert '"/api/drive/init"' in CONTROL_JS.read_text(encoding="utf-8")


def test_failed_drive_init_keeps_accepted_order_without_releasing_start(tmp_path):
    app = _gated_app(tmp_path)
    _link_drive(app)
    order_id = _accepted_order(app)
    request_id = app.request_drive_init()["drive_init"]["id"]
    app.drive_heartbeat({"init": {"id": request_id, "state": "failed", "message": "scan mismatch"}})
    assert app.state()["start"] == {"ready": False, "reason": "drive_init_failed", "order_id": order_id}
    with pytest.raises(OrderStateError, match="drive_init_failed"):
        app.start_mission()
    assert app.controller.accepted_orders()[0].order_id == order_id
    app.drive_heartbeat({"init": {"id": request_id, "state": "succeeded", "message": "init complete"}})
    assert app.state()["start"]["ready"] is False  # terminal result cannot be overwritten
    app.close()
