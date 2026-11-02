"""물따르기 끝 감지: 차이 기록이 판정을 바꾸지 않고 정해진 간격으로만 찍히는지 확인한다."""
import importlib.util
import sys
from pathlib import Path

ROLLOUT_DIR = Path(__file__).resolve().parents[2] / "tools" / "rollout"
sys.path.insert(0, str(ROLLOUT_DIR))
spec = importlib.util.spec_from_file_location("rollout_handoff_under_test", ROLLOUT_DIR / "rollout_handoff.py")
handoff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff)


def obs_at(pose, right_gripper=5.0):
    return {**pose, "left_gripper.pos": 20.0, "right_gripper.pos": right_gripper}


def make_detector():
    return handoff.DoneDetector("pour", tol=10.0, settle_s=1.0, place_tol=20.0)


def test_distance_is_logged_every_interval(capsys):
    det = make_detector()
    obs = obs_at({**handoff.CHASSIS_HOLD, "left_elbow_flex.pos": handoff.CHASSIS_HOLD["left_elbow_flex.pos"] + 12.5})
    for now in (100.0, 101.0, 106.0):
        det.update(now, obs)
    lines = capsys.readouterr().out.strip().splitlines()
    assert len(lines) == 2
    assert "'elbow_flex': 12.5" in lines[0]
    assert "right_gripper=5.0" in lines[0]


def test_end_needs_the_bottle_released_and_a_settled_left_arm():
    det = make_detector()
    assert det.update(0.0, obs_at(handoff.CHASSIS_HOLD, right_gripper=30.0)) is False
    assert det.update(1.0, obs_at(handoff.CHASSIS_HOLD, right_gripper=5.0)) is False
    assert det.update(2.5, obs_at(handoff.CHASSIS_HOLD, right_gripper=5.0)) is True


def test_left_arm_out_of_tolerance_never_ends():
    det = make_detector()
    det.update(0.0, obs_at(handoff.CHASSIS_HOLD, right_gripper=30.0))
    away = obs_at({**handoff.CHASSIS_HOLD, "left_wrist_flex.pos": handoff.CHASSIS_HOLD["left_wrist_flex.pos"] + 11.0})
    assert det.update(1.0, away) is False
    assert det.update(50.0, away) is False
