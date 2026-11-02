"""핸드오프(물따르기 -> 컵배치) 롤아웃 래퍼: 그리퍼 과부하 예방 + 끝 자세 선택 + 붓기 훅.

lerobot-rollout을 그대로 실행하되 세 가지를 덧붙인다. lerobot 원본 코드는 고치지 않는다.

1) 실행 전 그리퍼 풀기 (매 실행 자동)
   이전 롤아웃이 토크를 켠 채 끝나 그리퍼가 물체를 계속 밀고 있으면 과부하 보호가 걸려
   다음 연결이 "Missing motor ID 6"으로 실패한다. 연결 전에 양팔 그리퍼(ID 6)를 읽어,
   목표가 실제 위치보다 grip.margin 틱 넘게 밀고 있으면 목표를 "실제 위치 + margin"으로
   당긴다. 토크는 켠 채라 손가락 위치는 그대로다. 그래도 과부하가 1초 안에 안 풀리면
   그 그리퍼만 토크를 껐다 바로 켠다(수 ms).

2) 끝 자세 (--end.pose)
   final      : 마지막 자세 그대로 토크 유지 (기본). 끝나기 직전에 1)과 같은 방식으로
                그리퍼 미는 힘을 줄여, 다음 실행까지 기다리는 동안 과부하가 쌓이지 않게 한다.
   pour_start : 물따르기 v2 시작 자세(시연 40개 평균)로 end.move_s초에 걸쳐 이동 후 유지.
   initial    : lerobot 기본(이번 실행 시작 자세로 복귀).
   <저장 이름> : save_start_pose.py로 저장한 자세(config/rollout_poses/<이름>.json)로, 추론과 작업 끝
                감지가 모두 끝난 뒤 이동 후 유지. 이동 시간은 5)와 같은 규칙(end.move_s와 start.max_speed).
                그리퍼는 기본으로 건드리지 않는다(--end.grippers를 주면 함께 맞춘다).

3) 붓기 훅 (--pour_hook): tools/rollout/rollout_pour_hook.py의 훅을 함께 켠다.
   훅 값은 --hook.* 로 조정한다.

4) 작업 끝 자동 감지 (--end.auto) - 끝나면 duration을 기다리지 않고 바로 2)로 넘어간다.
   pour  : 오른손이 병을 쥐었다 놓은 뒤, 왼팔(그리퍼 제외 5관절)이 상판 적재 자세
           (컵배치 v2 시작 자세 평균) ±end.tol도 안에 end.settle_s초 머물면 끝.
           v2 시연 40개 모두 병을 놓은 뒤에만 이 자세에 들어왔다(이전 0프레임).
   place : 그리퍼 값은 클수록 열림이다(쥔 컵 10~24, 놓을 때 62~96, 빈손으로 닫으면 ~1).
           컵 놓음(측정값과 정책 명령이 모두 40 초과로 0.3초 유지) -> 놓은 자리에서 왼팔이
           10도 이상 빠짐 -> 그리퍼가 10 미만으로 다시 닫히고(명령도) 왼팔 5관절이 물따르기
           시작 자세 ±end.place_tol도 안에 end.settle_s초 머물면 끝.
           v2 시연 순서: 열기 -> 팔 빼며 닫기(놓은 자리에서 74~86도 이동) -> 약 1초 뒤 시작
           자세 도착 -> 7~10초 정지. 테이블 접촉으로 측정값이 조금 출렁여도 40까지 안 가고,
           컵을 다시 쥐면 컵에 막혀 17~22에 머물러 "닫힘"으로 보지 않는다.
           v2 시연 40개 마지막 자세는 이 자세와 관절별 평균 0.1~2.8도 차이였다.
           ±15도면 wrist_roll이 18도 벗어나 끝난 1개(ep6)를 놓쳐 기본값을 20도로 둔다
           (컵 놓음이 먼저 감지돼야 판정하므로 놓기 전에 자세가 비슷해도 끝나지 않는다).
   duration은 최대 시간(안전 상한)으로만 남는다.

5) 시작 자세 (--start.pose=<이름 또는 json 경로>)
   save_start_pose.py로 텔레옵해서 저장한 자세(config/rollout_poses/<이름>.json)로 추론 직전에
   맞춘 뒤 추론을 시작한다. 그리퍼는 기본으로 건드리지 않는다(컵배치는 컵을 쥔 채 시작하므로).
   --start.grippers를 주면 그리퍼도 저장값으로 맞춘다.
   이동 시간 = max(start.move_s, 가장 많이 움직이는 관절 각도 / start.max_speed).
   도착 후 저장값과 start.tol도 넘게 차이 나는 관절이 있으면 그 자리에 멈추고(현재 위치를
   목표로 고정) 추론을 시작하지 않는다. 이때 끝 자세는 --end.pose와 상관없이 "그대로 유지"다.

6) GR00T 저메모리 로딩 (정책이 groot면 자동, --groot.fp32로 끔)
   GR00T N1.7 체크포인트는 Qwen3-VL 백본을 fp32로 들고 있어 그대로 올리면 GPU 약 9.4GB(+로딩 중
   가중치 12.6GB)라 노트북 RTX 5050(8GB)에서 메모리 부족이 나거나 공유 메모리로 넘쳐 추론 1회가
   2초를 넘는다. 그래서 CPU에서 불러와 모델 전체를 bf16으로 바꾼 뒤 GPU로 옮긴다(약 6.3GB, 추론 약 0.55초).
   학습 forward도 bf16 autocast였으므로 계산 정밀도는 같고, 가중치만 bf16으로 반올림된다.

사용법 (평소 lerobot-rollout 플래그를 그대로 붙인다, --strategy.type=base 전용):
    python tools/rollout/rollout_handoff.py --end.pose=final ...물따르기 플래그...
    python tools/rollout/rollout_handoff.py --end.pose=pour_start ...컵배치 플래그...
    python tools/rollout/rollout_handoff.py --pour_hook --end.pose=final ...물따르기 플래그...
    python tools/rollout/rollout_handoff.py --start.pose=pour --end.pose=final ...물따르기 플래그...
    python tools/rollout/rollout_handoff.py --end.pose=place_end ...컵배치 플래그...
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rollout_pour_hook import PourHook, fill_camera_indices, install, parse_args as parse_hook_args  # noqa: E402

logger = logging.getLogger("rollout_handoff")

POSE_DIR = Path(__file__).resolve().parents[2] / "config" / "rollout_poses"
# 시작 자세 이동이 실패하면 끝 자세 처리(_teardown)가 팔을 더 움직이지 않게 막는 표시
RUN_STATE = {"start_failed": False}

GRIPPER_ID = 6
OVERLOAD_BIT = 32

# water-pour-position-jitter-gravel-tabletop-handoff-v2 40개 에피소드 시작 state 평균 (관절별 std 0.1~4.6)
POUR_START = {
    "left_shoulder_pan.pos": 0.63, "left_shoulder_lift.pos": -0.47, "left_elbow_flex.pos": 95.61,
    "left_wrist_flex.pos": -53.77, "left_wrist_roll.pos": -90.68, "left_gripper.pos": 1.01,
    "right_shoulder_pan.pos": -2.26, "right_shoulder_lift.pos": -1.35, "right_elbow_flex.pos": 95.58,
    "right_wrist_flex.pos": -80.33, "right_wrist_roll.pos": 0.74, "right_gripper.pos": 1.65,
}
END_POSES = ("final", "pour_start", "initial")

# cup-place-position-jitter-gravel-tabletop-handoff-v2 40개 시작 state 평균 = 물따르기 끝(상판 적재) 자세
CHASSIS_HOLD = {
    "left_shoulder_pan.pos": 66.66, "left_shoulder_lift.pos": 29.00, "left_elbow_flex.pos": 81.53,
    "left_wrist_flex.pos": -95.47, "left_wrist_roll.pos": -94.54,
}
LEFT_5 = tuple(CHASSIS_HOLD)
GRIP_OPEN = 40.0
GRIP_CLOSED = 10.0
DISTANCE_LOG_S = 5.0


class DoneDetector:
    def __init__(self, mode: str, tol: float, settle_s: float, place_tol: float,
                 release_s: float = 0.3, retract_deg: float = 10.0):
        self.mode, self.tol, self.settle_s, self.place_tol = mode, tol, settle_s, place_tol
        self.release_s, self.retract_deg = release_s, retract_deg
        self.bottle_held = self.bottle_released = False
        self.t_in: float | None = None
        self.t_open: float | None = None
        self.cup_released = self.retracted = False
        self.release_pose: dict[str, float] = {}
        self.t_log = 0.0

    def _log_distance(self, now: float, obs: dict) -> None:
        """끝 감지가 안 날 때 어느 관절이 얼마나 벗어났는지 남긴다. 판정은 바꾸지 않는다."""
        if now < self.t_log:
            return
        self.t_log = now + DISTANCE_LOG_S
        off = {k.removeprefix("left_").removesuffix(".pos"): round(float(obs[k]) - CHASSIS_HOLD[k], 1) for k in LEFT_5}
        print(f"[auto] 상판 적재 자세와 차이(허용 {self.tol:.0f}도) {off} "
              f"left_gripper={float(obs['left_gripper.pos']):.1f} right_gripper={float(obs['right_gripper.pos']):.1f}")

    def _settled(self, now: float, obs: dict, pose: dict[str, float], tol: float) -> bool:
        if max(abs(float(obs[k]) - pose[k]) for k in LEFT_5) >= tol:
            self.t_in = None
            return False
        if self.t_in is None:
            self.t_in = now
        return now - self.t_in >= self.settle_s

    def update(self, now: float, obs: dict, action: dict | None = None) -> bool:
        if self.mode == "pour":
            self._log_distance(now, obs)
            rg = float(obs["right_gripper.pos"])
            if rg > 20:
                self.bottle_held = True
            elif self.bottle_held and rg < 10 and not self.bottle_released:
                self.bottle_released = True
                print(f"[auto] 오른손 병 놓고 빈손으로 닫음 감지 (right_gripper={rg:.1f})")
            return self.bottle_released and self._settled(now, obs, CHASSIS_HOLD, self.tol)

        # 그리퍼 값은 클수록 열림. 쥔 컵 10~24 / 놓을 때 62~96 / 빈손으로 닫으면 ~1.
        lg = float(obs["left_gripper.pos"])
        cmd = float(action["left_gripper.pos"]) if action and "left_gripper.pos" in action else None
        if not self.cup_released:
            if not (lg > GRIP_OPEN and cmd is not None and cmd > GRIP_OPEN):
                self.t_open = None
                return False
            if self.t_open is None:
                self.t_open = now
            if now - self.t_open < self.release_s:
                return False
            self.cup_released = True
            self.release_pose = {k: float(obs[k]) for k in LEFT_5}
            print(f"[auto] 컵 놓음 감지 (left_gripper 측정 {lg:.1f}, 명령 {cmd:.1f})")
        if not self.retracted:
            moved = max(abs(float(obs[k]) - self.release_pose[k]) for k in LEFT_5)
            if moved < self.retract_deg:
                return False
            self.retracted = True
            print(f"[auto] 놓은 자리에서 빠짐 감지 ({moved:.1f}도)")
        closed = lg < GRIP_CLOSED and (cmd is None or cmd < GRIP_CLOSED)
        if not closed:
            self.t_in = None
            return False
        return self._settled(now, obs, POUR_START, self.place_tol)


def install_done(det: DoneDetector) -> None:
    from lerobot.rollout.strategies import base as base_strategy

    inner = base_strategy.send_next_action
    last_action: dict[str, float] = {}

    def send_next_action(obs_processed, obs_raw, ctx, interpolator):
        out = inner(obs_processed, obs_raw, ctx, interpolator)
        if out is not None:
            last_action.update(out)
        if not ctx.runtime.shutdown_event.is_set() and det.update(time.perf_counter(), obs_raw, last_action):
            print(f"[auto] {det.mode} 작업 끝 감지 -> 롤아웃 종료")
            ctx.runtime.shutdown_event.set()
        return out

    base_strategy.send_next_action = send_next_action


def relax_gripper(ph, pt, ctrl_table, tag: str, margin: int, clear_overload: bool) -> None:
    """그리퍼 목표가 실제 위치를 margin 넘게 밀고 있으면 목표를 실제 위치 쪽으로 당긴다."""
    a_te, a_goal, a_pres = (ctrl_table[k][0] for k in ("Torque_Enable", "Goal_Position", "Present_Position"))
    te, c, _ = ph.read1ByteTxRx(pt, GRIPPER_ID, a_te)
    if c != 0:
        print(f"[grip] {tag}: 그리퍼 응답 없음(comm={c}) - 건너뜀")
        return
    if te == 0:
        print(f"[grip] {tag}: 토크 꺼짐 - 할 일 없음")
        return
    goal, _, _ = ph.read2ByteTxRx(pt, GRIPPER_ID, a_goal)
    pres, c, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
    if c != 0:
        print(f"[grip] {tag}: 위치 읽기 실패(comm={c}) - 건너뜀")
        return
    diff = goal - pres
    if abs(diff) > margin:
        new_goal = pres + (margin if diff > 0 else -margin)
        ph.write2ByteTxRx(pt, GRIPPER_ID, a_goal, new_goal)
        print(f"[grip] {tag}: 미는 힘 줄임 goal {goal} -> {new_goal} (present {pres}, 차이 {diff})")
    else:
        print(f"[grip] {tag}: 정상 (goal {goal}, present {pres}, 차이 {diff})")

    if clear_overload and err & OVERLOAD_BIT:
        time.sleep(1.0)
        _, _, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
        if err & OVERLOAD_BIT:
            ph.write1ByteTxRx(pt, GRIPPER_ID, a_te, 0)
            ph.write1ByteTxRx(pt, GRIPPER_ID, a_te, 1)
            time.sleep(0.5)
            _, _, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
            print(f"[grip] {tag}: 과부하 지속 -> 토크 off/on 즉시 재설정, 결과 err={err}")
        else:
            print(f"[grip] {tag}: 과부하 해제됨")


def pre_relax(ports: dict[str, str], margin: int) -> None:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    for side, port in ports.items():
        bus = FeetechMotorsBus(port, {"gripper": Motor(GRIPPER_ID, "sts3215", MotorNormMode.RANGE_0_100)})
        try:
            bus.connect(handshake=False)
            relax_gripper(bus.packet_handler, bus.port_handler, bus.model_ctrl_table["sts3215"],
                          f"{side}({port}) 실행 전", margin, clear_overload=True)
        except Exception as e:  # noqa: BLE001
            print(f"[grip] {side}({port}) 실행 전 점검 실패: {e} - 그대로 진행")
        finally:
            if bus.port_handler.is_open:
                bus.port_handler.closePort()


def make_pose_mover(target: dict[str, float], move_s: float, label: str = "물따르기 시작 자세",
                    max_speed: float | None = None, stop_event=None):
    from lerobot.utils.robot_utils import precise_sleep

    def _move(hw, duration_s: float = 3.0, fps: int = 50) -> None:
        robot = hw.robot_wrapper
        try:
            cur = robot.get_observation()
            start = {k: float(cur[k]) for k in target if k in cur}
            dur = move_s
            if max_speed:
                biggest = max(abs(target[k] - start[k]) for k in start)
                dur = max(move_s, biggest / max_speed)
                print(f"[end] {label}로 {dur:.1f}s 동안 이동 (가장 큰 차이 {biggest:.1f}도)")
            steps = max(int(dur * fps), 1)
            for i in range(1, steps + 1):
                if stop_event is not None and stop_event.is_set():
                    now = robot.get_observation()
                    robot.send_action({k: float(now[k]) for k in start})
                    print(f"[end] {label} 이동 중단 - 현재 위치에서 정지")
                    return
                t = i / steps
                t = t * t * (3 - 2 * t)
                robot.send_action({k: start[k] * (1 - t) + target[k] * t for k in start})
                precise_sleep(1 / fps)
            time.sleep(0.5)
            now = robot.get_observation()
            off = {k: round(float(now[k]) - target[k], 1) for k in start if abs(float(now[k]) - target[k]) > 10}
            if off:
                # 목표에 못 간 관절은 더 밀지 않게 현재 위치를 목표로 고정한다
                robot.send_action({k: float(now[k]) for k in start})
                print(f"[end] {label} 도달 실패(10도 초과) {off} - 현재 위치에서 정지")
            else:
                print(f"[end] {label} 도달")
        except Exception as e:  # noqa: BLE001
            logger.warning("%s 이동 실패: %s", label, e)

    return _move


def load_start_pose(arg: str, grippers: bool) -> tuple[Path, dict[str, float]]:
    path = Path(arg) if arg.endswith(".json") else POSE_DIR / f"{arg}.json"
    if not path.is_file():
        sys.exit(f"자세 파일 없음: {path} (save_start_pose.py --name 으로 먼저 저장)")
    pose = {k: float(v) for k, v in json.loads(path.read_text(encoding="utf-8"))["pose"].items()}
    if not grippers:
        pose = {k: v for k, v in pose.items() if not k.endswith("gripper.pos")}
    return path, pose


def move_to_start_pose(robot, target: dict[str, float], move_s: float, max_speed: float, tol: float,
                       stop_event=None) -> bool:
    """저장 자세로 부드럽게 옮긴다. 도착 후 tol도 넘게 벗어난 관절이 있으면 그 자리에 고정하고 False.

    stop_event가 켜지면 이동을 끊고 그 자리에 고정한 뒤 False.
    """
    from lerobot.utils.robot_utils import precise_sleep

    fps = 50
    try:
        cur = robot.get_observation()
        start = {k: float(cur[k]) for k in target}
        biggest = max(abs(target[k] - start[k]) for k in target)
        dur = max(move_s, biggest / max_speed)
        print(f"[start] 시작 자세로 {dur:.1f}s 동안 이동 (가장 큰 차이 {biggest:.1f}도)")
        steps = max(int(dur * fps), 1)
        for i in range(1, steps + 1):
            if stop_event is not None and stop_event.is_set():
                now = robot.get_observation()
                robot.send_action({k: float(now[k]) for k in target})
                print("[start] 시작 자세 이동 중단 - 그 자리에 멈추고 추론하지 않음")
                return False
            t = i / steps
            t = t * t * (3 - 2 * t)
            robot.send_action({k: start[k] * (1 - t) + target[k] * t for k in start})
            precise_sleep(1 / fps)
        time.sleep(0.5)
        now = robot.get_observation()
        off = {k: round(float(now[k]) - target[k], 1) for k in target if abs(float(now[k]) - target[k]) > tol}
    except Exception as e:  # noqa: BLE001
        now, off = None, {"error": str(e)}
    if off:
        if now is not None:
            # 목표에 못 간 관절은 더 밀지 않게 현재 위치를 목표로 고정한다
            robot.send_action({k: float(now[k]) for k in target})
        print(f"[start] 시작 자세 도달 실패({tol:.0f}도 초과) {off} - 그 자리에 멈추고 추론하지 않음")
        return False
    print("[start] 시작 자세 도착 -> 추론 시작")
    return True


def install_start_pose(target: dict[str, float], move_s: float, max_speed: float, tol: float) -> None:
    from lerobot.rollout.strategies import base as base_strategy

    orig_run = base_strategy.BaseStrategy.run

    def run(self, ctx) -> None:
        if not move_to_start_pose(ctx.hardware.robot_wrapper, target, move_s, max_speed, tol):
            RUN_STATE["start_failed"] = True
            ctx.runtime.shutdown_event.set()
            return
        return orig_run(self, ctx)

    base_strategy.BaseStrategy.run = run


def relax_grippers(robot, margin: int, when: str) -> None:
    """양팔 그리퍼가 물체를 계속 밀지 않게 목표를 실제 위치 쪽으로 당긴다(토크는 유지)."""
    for side in ("left_arm", "right_arm"):
        try:
            bus = getattr(robot, side).bus
            relax_gripper(bus.packet_handler, bus.port_handler, bus.model_ctrl_table["sts3215"],
                          f"{side} {when}", margin, clear_overload=False)
        except Exception as e:  # noqa: BLE001
            print(f"[grip] {side} {when} 처리 실패: {e}")


def install_end_pose(pose: str, margin: int, move_s: float, saved: dict[str, float] | None = None,
                     max_speed: float | None = None) -> None:
    from lerobot.rollout.strategies.core import RolloutStrategy

    orig_teardown = RolloutStrategy._teardown_hardware

    def _teardown(self, hw, return_to_initial_position: bool = True) -> None:
        robot = hw.robot_wrapper.inner
        if pose == "final" or RUN_STATE["start_failed"]:
            if robot.is_connected:
                relax_grippers(robot, margin, "종료 시")
            print("[end] 마지막 자세 유지")
            return orig_teardown(self, hw, return_to_initial_position=False)
        if pose == "pour_start":
            print(f"[end] 물따르기 시작 자세로 {move_s:.0f}s 동안 이동")
            return orig_teardown(self, hw, return_to_initial_position=True)
        if saved is not None:
            return orig_teardown(self, hw, return_to_initial_position=True)
        return orig_teardown(self, hw, return_to_initial_position=return_to_initial_position)

    RolloutStrategy._teardown_hardware = _teardown
    if pose == "pour_start":
        RolloutStrategy._return_to_initial_position = staticmethod(make_pose_mover(POUR_START, move_s))
    elif saved is not None:
        RolloutStrategy._return_to_initial_position = staticmethod(
            make_pose_mover(saved, move_s, f"저장 자세 '{pose}'", max_speed))


def is_groot_policy(policy_path: str | None) -> bool:
    if not policy_path:
        return False
    cfg = Path(policy_path) / "config.json"
    if not cfg.is_file():
        try:
            from huggingface_hub import hf_hub_download
            cfg = Path(hf_hub_download(policy_path, "config.json"))
        except Exception:
            return False
    return json.loads(cfg.read_text(encoding="utf-8")).get("type") == "groot"


def install_groot_bf16() -> None:
    """GrootPolicy.from_pretrained를 감싸 CPU에서 불러오고 bf16으로 바꾼 뒤 원래 장치로 옮긴다(6번)."""
    import torch
    from lerobot.policies.groot.modeling_groot import GrootPolicy

    original = GrootPolicy.from_pretrained.__func__

    def from_pretrained(cls, pretrained_name_or_path, *, config=None, **kwargs):
        if config is None:
            return original(cls, pretrained_name_or_path, **kwargs)
        device = config.device
        config.model_params_fp32 = False
        config.device = "cpu"
        try:
            policy = original(cls, pretrained_name_or_path, config=config, **kwargs)
        finally:
            config.device = device
        policy.to(torch.bfloat16).to(device)
        print(f"[groot] bf16으로 불러옴 (GPU {torch.cuda.memory_allocated() / 1e9:.2f}GB)")
        return policy

    def load_streamed(cls, model, model_file, map_location, strict):
        # 원래는 fp32 모델(12.6GB)과 가중치 전체(11.7GB)를 한꺼번에 올려 커밋 메모리 부족(os error 1455)으로 죽었다.
        # 모델을 먼저 bf16으로 줄이고 가중치를 한 텐서씩 복사해 순간 메모리를 모델 크기 수준으로 낮춘다.
        from safetensors import safe_open
        from lerobot.policies.utils import log_model_loading_keys

        model.to(torch.bfloat16)
        state = model.state_dict()
        missing, unexpected = set(state), []
        with torch.no_grad(), safe_open(model_file, framework="pt", device="cpu") as f:
            for key in f.keys():
                if key in state:
                    state[key].copy_(f.get_tensor(key))
                    missing.discard(key)
                else:
                    unexpected.append(key)
        # embed_tokens처럼 lm_head와 같은 텐서를 공유하는 키는 파일에 한 번만 있어도 채워진다
        loaded = {state[k].data_ptr() for k in state if k not in missing}
        missing = {k for k in missing if state[k].data_ptr() not in loaded}
        log_model_loading_keys(sorted(missing), unexpected)
        return model

    GrootPolicy.from_pretrained = classmethod(from_pretrained)
    GrootPolicy._load_as_safetensor = classmethod(load_streamed)


def arg_value(args: list[str], key: str) -> str | None:
    for a in args:
        if a.startswith(key + "="):
            return a.split("=", 1)[1]
    return None


def main() -> None:
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--end.pose", dest="end_pose", default="final", help=f"{END_POSES} 또는 저장 자세 이름")
    ap.add_argument("--end.grippers", dest="end_grippers", action="store_true")
    ap.add_argument("--end.move_s", dest="move_s", type=float, default=5.0)
    ap.add_argument("--grip.margin", dest="margin", type=int, default=10)
    ap.add_argument("--grip.skip_pre", dest="skip_pre", action="store_true")
    ap.add_argument("--pour_hook", dest="pour_hook", action="store_true")
    ap.add_argument("--end.auto", dest="auto", choices=("off", "pour", "place"), default="off")
    ap.add_argument("--end.tol", dest="tol", type=float, default=10.0)
    ap.add_argument("--end.settle_s", dest="settle_s", type=float, default=1.0)
    ap.add_argument("--end.place_tol", dest="place_tol", type=float, default=20.0)
    ap.add_argument("--start.pose", dest="start_pose", default=None)
    ap.add_argument("--start.grippers", dest="start_grippers", action="store_true")
    ap.add_argument("--start.move_s", dest="start_move_s", type=float, default=5.0)
    ap.add_argument("--start.max_speed", dest="start_max_speed", type=float, default=20.0)
    ap.add_argument("--start.tol", dest="start_tol", type=float, default=10.0)
    ap.add_argument("--groot.fp32", dest="groot_fp32", action="store_true")
    ns, rest = ap.parse_known_args(sys.argv[1:])
    hook_params, _, rest = parse_hook_args(rest)
    rest = fill_camera_indices(rest)
    if "--strategy.type=base" not in rest:
        sys.exit("--strategy.type=base 로만 동작한다.")
    if arg_value(rest, "--return_to_initial_position") is not None:
        sys.exit("--return_to_initial_position 대신 --end.pose 를 쓴다.")

    if not ns.skip_pre:
        ports = {s: p for s, p in (("left", arg_value(rest, "--robot.left_arm_config.port")),
                                    ("right", arg_value(rest, "--robot.right_arm_config.port"))) if p}
        pre_relax(ports, ns.margin)

    if ns.start_pose:
        path, target = load_start_pose(ns.start_pose, ns.start_grippers)
        install_start_pose(target, ns.start_move_s, ns.start_max_speed, ns.start_tol)
        print(f"[start] 시작 자세 = {path} ({len(target)}관절)")
    saved_end = None
    if ns.end_pose not in END_POSES:
        path, saved_end = load_start_pose(ns.end_pose, ns.end_grippers)
        print(f"[end] 끝 자세 파일 = {path} ({len(saved_end)}관절)")
    install_end_pose(ns.end_pose, ns.margin, ns.move_s, saved_end, ns.start_max_speed)
    hook = None
    if ns.pour_hook:
        hook = PourHook(hook_params)
        install(hook)
        print(f"[pour_hook] 활성: {hook_params}")
    if ns.auto != "off":
        install_done(DoneDetector(ns.auto, ns.tol, ns.settle_s, ns.place_tol))
    print(f"[end] 끝 자세 = {ns.end_pose}, 자동 종료 = {ns.auto}")

    if not ns.groot_fp32 and is_groot_policy(arg_value(rest, "--policy.path")):
        install_groot_bf16()
    sys.argv = ["lerobot-rollout", *rest]
    from lerobot.scripts.lerobot_rollout import main as rollout_main

    try:
        rollout_main()
    finally:
        if hook is not None:
            print(f"\n[pour_hook] 최종 상태: {hook.state}")
            for t, state, msg in hook.events:
                print(f"  t={t:6.1f}s [{state}] {msg}")


if __name__ == "__main__":
    main()
