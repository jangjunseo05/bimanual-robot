"""상주형 조작 실행기의 이동 자세(stow) 처리: 팔 하드웨어·lerobot 없이 가짜 팔로 확인한다."""
import importlib.util
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROLLOUT_DIR = Path(__file__).resolve().parents[2] / "tools" / "rollout"


@pytest.fixture(scope="module")
def svc():
    sys.path.insert(0, str(ROLLOUT_DIR))
    spec = importlib.util.spec_from_file_location("rollout_service_under_test", ROLLOUT_DIR / "rollout_service.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # 보간 대기 생략. time 모듈 자체를 바꾸면 다른 테스트의 대기까지 사라지므로 이 모듈의 참조만 바꾼다
    module.time = SimpleNamespace(sleep=lambda s: None, perf_counter=time.perf_counter,
                                  strftime=time.strftime)
    return module


class FakeArm:
    def __init__(self, joints, stuck=()):
        self.pos = dict(joints)
        self.stuck = set(stuck)
        self.sent = []

    def send_action(self, action):
        self.sent.append(dict(action))
        for k, v in action.items():
            if k not in self.stuck:
                self.pos[k] = v
        return action


class FakeRobot:
    """robot_wrapper 흉내: get_observation()과 inner.left_arm / inner.right_arm."""

    def __init__(self, left, right, stuck=()):
        self.inner = SimpleNamespace(left_arm=FakeArm(left, stuck), right_arm=FakeArm(right, stuck),
                                     is_connected=True)

    def get_observation(self):
        obs = {f"left_{k}": v for k, v in self.inner.left_arm.pos.items()}
        obs.update({f"right_{k}": v for k, v in self.inner.right_arm.pos.items()})
        return obs


JOINTS = ("shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos", "wrist_roll.pos")
CUP_HOLD = dict(zip(JOINTS, (66.7, 29.0, 81.5, -95.5, -94.5)))
RIGHT_AFTER_POUR = dict(zip(JOINTS, (10.0, 20.0, 60.0, -40.0, 5.0)))
STOW = {**{f"left_{k}": v for k, v in zip(JOINTS, (0.0, 0.4, 96.0, -81.3, -89.5))},
        **{f"right_{k}": v for k, v in zip(JOINTS, (-3.3, 0.0, 95.7, -81.5, 1.5))}}


def make_service(svc, robot, stow=STOW):
    s = svc.Service.__new__(svc.Service)
    s.ns = SimpleNamespace(stow_pose="place_end", start_move_s=0.1, start_max_speed=1000.0, start_tol=10.0,
                           margin=10, move_s=0.1)
    hardware = SimpleNamespace(robot_wrapper=robot)
    s.contexts = {"pour": SimpleNamespace(hardware=hardware), "place": SimpleNamespace(hardware=hardware)}
    s.stow_target = stow
    s.lock = threading.Lock()
    s.arms = "unknown"
    s.current_event, s.stop_requested = None, False
    s.state, s.current, s.results, s.timings, s.idle_failures = "ready", None, [], {}, 0
    s.jobs = SimpleNamespace(put=lambda job: None)
    return s


def test_right_only_move_never_commands_the_left_arm(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    right = {k: v for k, v in STOW.items() if k.startswith("right_")}
    ok, off = svc.move_joints(robot, right, 0.1, 1000.0, 10.0, "test")
    assert ok and off == {}
    assert robot.inner.left_arm.sent == []
    assert robot.inner.left_arm.pos == CUP_HOLD
    assert robot.get_observation()["right_elbow_flex.pos"] == pytest.approx(95.7)


def test_already_there_does_not_move(svc):
    robot = FakeRobot({k.removeprefix("left_"): v for k, v in STOW.items() if k.startswith("left_")},
                      {k.removeprefix("right_"): v for k, v in STOW.items() if k.startswith("right_")})
    ok, _ = svc.move_joints(robot, STOW, 0.1, 1000.0, 10.0, "test")
    assert ok
    assert robot.inner.left_arm.sent == [] and robot.inner.right_arm.sent == []


def test_blocked_joint_stops_in_place_and_reports(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR, stuck={"elbow_flex.pos"})
    ok, off = svc.move_joints(robot, STOW, 0.1, 1000.0, 10.0, "test")
    assert not ok
    assert set(off) == {"left_elbow_flex.pos", "right_elbow_flex.pos"}
    # 마지막 명령은 현재 위치 고정(더 밀지 않음)
    assert robot.inner.right_arm.sent[-1]["elbow_flex.pos"] == RIGHT_AFTER_POUR["elbow_flex.pos"]


def test_stow_job_folds_both_arms(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    result = service._run_stow({"submitted": 0.0})
    assert result["ok"] and result["reason"] == "stowed"
    assert service.arms == "stowed"
    assert all(abs(robot.get_observation()[k] - v) < 1e-6 for k, v in STOW.items())


def test_stop_interrupts_the_fold_and_holds_in_place(svc):
    robot = FakeRobot({k.removeprefix("left_"): 0.0 for k in STOW if k.startswith("left_")},
                      {k.removeprefix("right_"): 0.0 for k in STOW if k.startswith("right_")})
    event = threading.Event()
    event.set()
    ok, off = svc.move_joints(robot, STOW, 5.0, 20.0, 10.0, "시험", stop_event=event)
    assert (ok, off) == (False, {"stopped": True})
    assert all(v == 0.0 for v in {**robot.inner.left_arm.pos, **robot.inner.right_arm.pos}.values())


def test_stop_request_reaches_a_running_stow_job(svc):
    held = threading.Event()
    robot = FakeRobot({k.removeprefix("left_"): 0.0 for k in STOW if k.startswith("left_")},
                      {k.removeprefix("right_"): 0.0 for k in STOW if k.startswith("right_")})
    service = make_service(svc, robot)
    send = robot.inner.left_arm.send_action

    def stop_on_first_command(action):
        if not held.is_set():
            held.set()
            assert service.stop()["stopped"] is True
        return send(action)

    robot.inner.left_arm.send_action = stop_on_first_command
    result = service._run_stow({"submitted": time.perf_counter()})
    assert (result["ok"], result["reason"]) == (False, "stopped")
    assert service.arms == "hold"


def test_stow_requires_configured_pose(svc):
    service = make_service(svc, FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR), stow=None)
    code, body, job = service.submit("stow")
    assert code == 404 and body["error"] == "stow_not_configured" and job is None


def test_after_pour_only_the_right_arm_folds(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    result = {}
    arms = service._stow_right_after_pour(SimpleNamespace(robot_wrapper=robot), result)
    assert arms == "carrying_cup" and result["right_stow"] == "ok"
    assert robot.inner.left_arm.sent == [] and robot.inner.left_arm.pos == CUP_HOLD


def test_policy_end_does_not_cancel_the_right_arm_fold(svc):
    # 작업 끝 감지는 정책을 끝내려고 current_event를 켠다. 2026-10-31 이 이벤트로 오른팔 접기가 끊겼다
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    service.current_event = threading.Event()
    service.current_event.set()
    result = {"reason": "auto_done"}
    service._finish_pose("pour", {"end_pose": "final"}, result)
    assert result["right_stow"] == "ok" and service.arms == "carrying_cup"
    assert robot.get_observation()["right_elbow_flex.pos"] == pytest.approx(95.7)


def test_policy_end_does_not_cancel_the_end_pose(svc, monkeypatch):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    service.current_event = threading.Event()
    service.current_event.set()
    seen = []

    def mover(target, move_s, label, max_speed, stop_event=None):
        seen.append(stop_event.is_set())
        return lambda hw: None

    monkeypatch.setattr(svc, "make_pose_mover", mover)
    service._finish_pose("place", {"end_pose": "place_end", "end_target": STOW}, {"reason": "auto_done"})
    assert seen == [False]


def test_stop_during_the_end_move_still_works(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    service.current_event = threading.Event()
    send = robot.inner.right_arm.send_action

    def stop_on_first_command(action):
        service.stop()
        return send(action)

    robot.inner.right_arm.send_action = stop_on_first_command
    result = {"reason": "auto_done"}
    service._finish_pose("pour", {"end_pose": "final"}, result)
    assert result["right_stow"] == {"stopped": True} and service.arms == "hold"


def test_lost_connection_still_relaxes_grippers(svc, monkeypatch):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    robot.inner.is_connected = False
    service = make_service(svc, robot)
    relaxed = []
    monkeypatch.setattr(svc, "relax_grippers", lambda r, margin, when: relaxed.append(when))
    service._finish_pose("pour", {"end_pose": "final"}, {"reason": "error: BiSOFollower is not connected."})
    assert relaxed == ["pour 끝"] and service.arms == "hold"


class BusArm(FakeArm):
    """실제 SOFollower처럼 모터 버스(bus.sync_read)를 가진 가짜 팔."""

    def __init__(self, joints):
        super().__init__(joints)
        self.bus = SimpleNamespace(sync_read=lambda name, num_retry=3: {
            k.removesuffix(".pos"): v for k, v in self.pos.items()})


class CameraLostRobot(FakeRobot):
    """손목캠 USB가 빠져 get_observation()이 실패하는 로봇."""

    def __init__(self, left, right):
        self.inner = SimpleNamespace(left_arm=BusArm(left), right_arm=BusArm(right), is_connected=True)

    def get_observation(self):
        raise RuntimeError("OpenCVCamera(2) latest frame is too old: 756.1 ms (max allowed: 500 ms).")


def test_right_fold_after_pour_works_without_cameras(svc):
    robot = CameraLostRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    result = {}
    arms = service._stow_right_after_pour(SimpleNamespace(robot_wrapper=robot), result)
    assert arms == "carrying_cup" and result["right_stow"] == "ok"
    assert robot.inner.left_arm.sent == []
    assert robot.inner.right_arm.pos["elbow_flex.pos"] == pytest.approx(95.7)
