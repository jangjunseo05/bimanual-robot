"""양팔을 전혀 움직이지 않고 토크만 끈다.

connect()는 이동 명령을 보내지 않는다(버스 연결 + 설정 읽기만) — 포트를
열었다가 바로 disconnect(disable_torque_on_disconnect=True)로 토크를
끊는다. `lerobot-rollout` 실행 중 비정상 종료 등으로 팔이 낯선 자세에서
토크가 걸린 채 멈췄을 때, 안전하게 힘만 빼는 용도.

사용법:
    python tools/rollout/emergency_torque_off.py
"""

from pathlib import Path

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(__file__).resolve().parents[2] / "calibration" / "bi_follower"

cfg = BiSOFollowerConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOFollowerConfig(port="COM3", disable_torque_on_disconnect=True),
    right_arm_config=SOFollowerConfig(port="COM4", disable_torque_on_disconnect=True),
)
robot = BiSOFollower(cfg)
robot.connect(calibrate=False)
print("connected, cutting torque now")
robot.disconnect()
print("TORQUE OFF - done")
