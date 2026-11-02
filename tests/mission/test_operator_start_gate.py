"""운영자 수락·START·긴급 정지 흐름. 가짜 조작 실행기만 쓰며 실제 팔·차를 움직이지 않는다."""
from http.server import ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import threading
import time
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from apps.service_order.service_order_server import BackendBusyError, ServiceApplication, handler_factory
from src.mission.service_execution_backend import ImmediateBackend, RolloutServiceBackend
from src.mission.service_mission_control import MissionController, OrderStateError
from tests.mission.test_rollout_service_backend import FakeRolloutService


@pytest.fixture
def fake():
    service = FakeRolloutService()
    service.stow_pose = "place_end"
    yield service
    service.close()


def gated_app(backend=None, state_dir=None):
    return ServiceApplication(MissionController(require_operator=True), state_dir, backend or ImmediateBackend())


def place_order(app, **extra):
    return app.submit({"table_id": "table_1", "drink": "COLD_WATER", **extra})[0]["order"]["order_id"]


def order_state(app, order_id):
    return app.controller.orders[order_id].state


def test_order_waits_for_accept_and_start():
    app = gated_app()
    order_id = place_order(app)
    assert order_state(app, order_id) == "PENDING"
    assert app.state()["start"] == {"ready": False, "reason": "no_accepted_order", "order_id": None}
    app.execute_next()
    assert app.controller.phase == "IDLE_AT_DOCK"

    app.accept(order_id)
    assert order_state(app, order_id) == "ACCEPTED"
    assert app.state()["start"] == {"ready": True, "reason": None, "order_id": order_id}
    assert app.controller.phase == "IDLE_AT_DOCK"

    app.start_mission()
    assert order_state(app, order_id) == "RUNNING"
    assert app.controller.phase == "NAVIGATE_KITCHEN"
    assert app.state()["start"]["reason"] == "mission_running"


def test_arms_receive_nothing_before_start(fake):
    app = gated_app(RolloutServiceBackend(url=fake.url))
    order_id = place_order(app)
    app.execute_next()
    app.accept(order_id)
    app.execute_next()
    assert fake.calls == []

    app.start_mission()
    app.execute_next()
    assert fake.calls == ["stow"]
    assert app.controller.phase == "ALIGN_KITCHEN"


def test_start_takes_the_oldest_accepted_order():
    app = gated_app()
    first, second = place_order(app), place_order(app)
    app.accept(second)
    app.accept(first)
    assert app.state()["start"]["order_id"] == first


def test_rejected_or_canceled_order_cannot_start():
    app = gated_app()
    rejected, canceled = place_order(app), place_order(app)
    app.reject(rejected)
    app.cancel(canceled)
    assert order_state(app, rejected) == "REJECTED"
    assert order_state(app, canceled) == "CANCELED"
    with pytest.raises(OrderStateError):
        app.accept(rejected)
    with pytest.raises(OrderStateError):
        app.accept(canceled)
    with pytest.raises(OrderStateError):
        app.start_mission()
    assert app.state()["stats"]["rejected"] == 1


def test_refused_order_takes_its_food_lines_with_it():
    app = gated_app()
    items = [{"name": "감자튀김", "qty": 1, "price": 4000, "robot": False}]
    rejected = app.submit({"table_id": "table_1", "drink": "COLD_WATER", "items": items})[0]["order"]["order_id"]
    canceled = app.submit({"table_id": "table_1", "drink": "COLD_WATER", "items": items})[0]["order"]["order_id"]
    app.reject(rejected)
    app.cancel(canceled)
    assert app.tickets[rejected]["staff_state"] is None
    assert app.tickets[canceled]["staff_state"] is None


def test_operator_order_arrives_accepted():
    app = gated_app()
    order_id = place_order(app, accepted=True)
    assert order_state(app, order_id) == "ACCEPTED"
    assert app.state()["start"]["ready"]


def test_start_is_refused_until_the_manipulation_service_is_ready(fake):
    backend = RolloutServiceBackend(url=fake.url)
    app = gated_app(backend)
    order_id = place_order(app, accepted=True)
    fake.loading_polls = 1
    backend.service_status(max_age_s=0)
    assert app.state()["start"]["reason"] == "manipulation_not_ready"
    with pytest.raises(OrderStateError):
        app.start_mission()
    assert order_state(app, order_id) == "ACCEPTED"
    fake.loading_polls = 0
    backend.service_status(max_age_s=0)
    app.start_mission()
    assert order_state(app, order_id) == "RUNNING"


def test_emergency_stop_latches_until_reset(fake):
    backend = RolloutServiceBackend(url=fake.url)
    app = gated_app(backend)
    order_id = place_order(app, accepted=True)
    waiting = place_order(app, accepted=True)
    app.start_mission()
    app.execute_next()

    result = app.emergency_stop()
    assert result["arm_stop_sent"] is True
    assert fake.stops == 2
    order = app.controller.orders[order_id]
    assert order.state == "FAILED"
    assert order.failure == "ALIGN_KITCHEN:EMERGENCY_STOP"
    state = app.state()
    assert state["emergency_stop"]["active"] is True
    assert state["emergency_stop"]["order_id"] == order_id
    assert state["start"]["reason"] == "emergency_stop"

    with pytest.raises(BackendBusyError):
        app.execute_next()
    with pytest.raises(OrderStateError):
        app.start_mission()
    blocked = backend.execute({"kind": "navigate", "destination": "kitchen"}, mission_id="M2", phase_attempt=1)
    assert (blocked.success, blocked.failure_code) == (False, "EMERGENCY_STOP")
    assert fake.calls == ["stow"]
    assert order_state(app, waiting) == "ACCEPTED"

    app.reset_emergency_stop()
    assert app.state()["emergency_stop"]["active"] is False
    assert order_state(app, waiting) == "ACCEPTED"
    assert app.state()["start"]["order_id"] == waiting


def test_emergency_stop_without_a_mission_still_latches():
    app = gated_app()
    assert app.emergency_stop()["order"] is None
    assert app.state()["emergency_stop"]["active"] is True
    assert app.state()["start"]["reason"] == "emergency_stop"


def test_emergency_stop_releases_an_alignment_wait(fake):
    app = gated_app(RolloutServiceBackend(url=fake.url))
    place_order(app, accepted=True)
    app.start_mission()
    app.execute_next()
    worker = threading.Thread(target=app.execute_next)
    worker.start()
    deadline = time.monotonic() + 3.0
    while app.backend.info()["waiting_alignment"] is None and time.monotonic() < deadline:
        time.sleep(0.02)
    app.emergency_stop()
    worker.join(timeout=3.0)
    assert not worker.is_alive()
    assert fake.calls == ["stow"]
    assert app.controller.active_order is None


def test_restart_keeps_orders_behind_the_gate(tmp_path):
    app = gated_app(state_dir=tmp_path)
    pending, accepted = place_order(app), place_order(app, accepted=True)
    app.close()
    restored = gated_app(state_dir=tmp_path)
    try:
        assert order_state(restored, pending) == "PENDING"
        assert order_state(restored, accepted) == "ACCEPTED"
        restored.execute_next()
        assert restored.controller.phase == "IDLE_AT_DOCK"
        assert restored.state()["start"]["order_id"] == accepted
    finally:
        restored.close()


def test_control_page_start_reaches_the_drive_pc(fake, tmp_path):
    """실제 주문 서버 HTTP로 START → 팔 접기 → 주행 PC 출발 판단까지 잇는다. 차 실행만 가짜다."""
    source = Path(__file__).resolve().parents[2] / "tools/jdamr/water_service.py"
    spec = importlib.util.spec_from_file_location("water_service_link", source)
    water_service = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(water_service)

    app = gated_app(RolloutServiceBackend(url=fake.url))
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    drive = water_service.WaterServiceController(
        inference_url=f"http://127.0.0.1:{httpd.server_port}", state_directory=tmp_path)
    drive.start = Mock()
    try:
        order_id = place_order(app, accepted=True)
        assert drive.follow_web(drive.read_web_state()) is None
        drive.start.assert_not_called()
        assert app.state()["drive_link"] == {"connected": False}

        app.start_mission()
        app.execute_next()
        worker = threading.Thread(target=app.execute_next)
        worker.start()
        deadline = time.monotonic() + 3.0
        while app.backend.info()["waiting_alignment"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        app.backend.service_status(max_age_s=0)

        assert drive.follow_web(drive.read_web_state()) == f"start {order_id}"
        drive.start.assert_called_once_with(f"web-{order_id}")
        drive.report_web(dry_run=False)
        link = app.state()["drive_link"]
        assert link["connected"] is True and link["state"] == "idle"

        app.emergency_stop()
        worker.join(timeout=3.0)
        assert not worker.is_alive()
    finally:
        httpd.shutdown()
        httpd.server_close()


def post(url):
    request = Request(url, data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=5) as response:
        return response.status, json.loads(response.read())


def test_operator_routes_over_http():
    app = gated_app()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_port}"
    try:
        order_id = place_order(app)
        with pytest.raises(HTTPError) as refused:
            post(f"{base}/api/mission/start")
        assert refused.value.code == 409
        assert post(f"{base}/api/orders/{order_id}/accept")[1]["order"]["state"] == "ACCEPTED"
        assert post(f"{base}/api/mission/start")[1]["order"]["state"] == "RUNNING"
        assert post(f"{base}/api/emergency-stop")[1]["emergency_stop"]["active"] is True
        assert post(f"{base}/api/emergency-stop/reset")[1]["emergency_stop"]["active"] is False
        other = place_order(app)
        assert post(f"{base}/api/orders/{other}/reject")[1]["order"]["state"] == "REJECTED"
        with pytest.raises(HTTPError) as conflict:
            post(f"{base}/api/orders/{other}/accept")
        assert conflict.value.code == 409
    finally:
        httpd.shutdown()
        httpd.server_close()
