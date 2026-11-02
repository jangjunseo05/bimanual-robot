"""상주형 조작 실행기: 시작할 때 물따르기·컵배치 정책을 모두 준비해 두고, 호출이 오면 바로 실행한다.

lerobot-rollout을 작업마다 새로 띄우면 라이브러리·모델 로드, 팔·카메라 연결 때문에 1분 넘게 걸린다.
이 실행기는 충전소 출발과 동시에 한 번만 띄운다.

    준비 (시작 직후, 이동 중):
        그리퍼 사전 이완 -> 물따르기 정책 로드 -> 팔·카메라 연결(1번) -> 컵배치 정책 로드
        -> 두 정책 모두 현재 관측으로 추론 1회씩 예열(팔은 움직이지 않음) -> 대기(현재 자세 유지)
    호출 pour  (주방 정렬 완료): 시작 자세 이동 -> 물따르기 정책 -> 작업 끝 감지 -> 끝 자세 -> 대기
    호출 place (테이블 정렬 완료): 컵배치 정책 -> 작업 끝 감지 -> 끝 자세 -> 대기
    다음 주문이 오면 다시 pour부터. 끝낼 때 shutdown.

이동 자세 (--stow.pose=place_end): 차가 움직이는 동안 팔이 테이블에 걸리지 않게 팔을 접어 든 자세.
    호출 stow  (출발 전, 바퀴 정지): 양팔(그리퍼 제외)을 이동 자세로 옮긴다.
    pour 이 작업 끝 감지로 끝나면: 왼팔은 컵을 쥔 상판 적재 자세 그대로 두고 오른팔만 이동 자세로 옮긴다.
    place 는 끝 자세(place_end)가 곧 이동 자세다.
    /status 의 arms: stowed(양팔 이동 자세) / carrying_cup(왼팔 컵 유지, 오른팔 이동 자세) / busy / hold(실패·중단으로
    그 자리 유지, 사람이 확인) / unknown(시작 직후).

팔·카메라 포트는 한 프로그램만 열 수 있어서, 두 정책이 같은 연결을 나눠 쓴다.
작업별 부가 기능(시작·끝 자세, 붓기 훅, 작업 끝 자동 감지)은 rollout_handoff.py와 같은 코드를 쓴다.

호출 방법 (같은 PC):
    HTTP   POST http://127.0.0.1:8770/run/pour      작업이 끝날 때까지 기다렸다 결과 JSON을 돌려준다
           POST http://127.0.0.1:8770/run/place     (?wait=0 이면 바로 202를 돌려주고 /status로 확인)
           GET  http://127.0.0.1:8770/status        준비 상태·현재 작업·최근 결과·소요 시간
           POST http://127.0.0.1:8770/stow          양팔을 이동 자세로(--stow.pose를 준 경우만, 아니면 404)
           POST http://127.0.0.1:8770/stop          진행 중인 작업 중단(팔은 그 자리에서 유지)
           POST http://127.0.0.1:8770/shutdown      팔 연결을 끊고 종료(토크는 연결 설정대로)
    명령줄 python tools/rollout/rollout_service.py call pour   (stow | status | stop | shutdown 도 같음)

실행 (평소 lerobot-rollout의 로봇·카메라 인자를 그대로 붙이고, 정책·task·duration만 작업별로 준다):
    python tools/rollout/rollout_service.py \\
        --pour.policy=<물따르기 pretrained_model> --place.policy=<컵배치 pretrained_model> \\
        --policy.device=cuda --robot.type=bi_so_follower ... (카메라 {left} {right} {depth} 자동 채움)

작업별 기본값 (rollout_handoff.py에서 쓰던 조합):
    pour : --pour.start_pose=pour  --pour.end_pose=final      --pour.auto=pour   --pour.duration=100
    place: --place.start_pose=none --place.end_pose=place_end --place.auto=place --place.duration=50
    붓기 훅은 --pour.hook 으로 켠다(--hook.* 값은 rollout_pour_hook.py와 같다).
    이동 자세: --stow.pose=place_end (기본 none = 이동 자세 기능 끔)
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import urllib.request
from dataclasses import replace
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rollout_handoff import (  # noqa: E402
    END_POSES,
    POUR_START,
    DoneDetector,
    arg_value,
    install_done,
    load_start_pose,
    make_pose_mover,
    move_to_start_pose,
    pre_relax,
    relax_grippers,
)
from rollout_pour_hook import PourHook, fill_camera_indices, install, parse_args as parse_hook_args  # noqa: E402

TASKS = ("pour", "place")
DEFAULT_URL = "http://127.0.0.1:8770"
# 학습 데이터셋 meta/tasks.parquet과 글자 단위까지 같아야 한다
TASK_TEXT = {
    "pour": ("Pick up the cup with the left arm, pick up the water bottle with the right arm, pour water from the "
             "bottle into the cup, set the bottle down to the right of where it was picked up, move the cup onto "
             "the robot chassis with the left arm, and keep holding the cup."),
    "place": "While holding the cup already loaded on the robot chassis with the left arm, move it to the table and place it down.",
}
DEFAULTS = {
    "pour": {"duration": 100.0, "start_pose": "pour", "end_pose": "final", "auto": "pour"},
    "place": {"duration": 50.0, "start_pose": "none", "end_pose": "place_end", "auto": "place"},
}
RESERVED = ("--policy.path", "--task", "--duration", "--strategy.type", "--return_to_initial_position",
            "--use_torch_compile")


def log(msg: str) -> None:
    print(f"[service {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---------------------------------------------------------------- 이동 자세
def send_arm_joints(robot, action: dict[str, float]) -> None:
    """키가 있는 팔에만 명령한다. 오른팔만 옮길 때 컵을 쥔 왼팔에는 아무것도 보내지 않기 위해서다."""
    for side in ("left", "right"):
        part = {k.removeprefix(f"{side}_"): v for k, v in action.items() if k.startswith(f"{side}_")}
        if part:
            getattr(robot.inner, f"{side}_arm").send_action(part)


def read_joints(robot) -> dict[str, float]:
    """관절 위치만 모터 버스에서 읽는다(카메라는 읽지 않는다).

    팔 접기·자세 확인은 관절만 있으면 된다. get_observation()은 카메라까지 읽어서, 손목캠 USB가 빠지면
    접기가 실패하고 arms_ready=false로 주행이 막혔다(2026-10-30 통합 시험). 정책 추론은 그대로 카메라를 쓴다.
    """
    joints: dict[str, float] = {}
    for side in ("left", "right"):
        arm = getattr(robot.inner, f"{side}_arm", None)
        bus = getattr(arm, "bus", None)
        if bus is None:  # 버스가 없는 대체 객체(테스트용 가짜 팔 등)는 관측에서 관절만 고른다
            return {k: float(v) for k, v in robot.get_observation().items() if k.endswith(".pos")}
        retries = getattr(getattr(arm, "config", None), "num_read_retries", 3)
        for motor, value in bus.sync_read("Present_Position", num_retry=retries).items():
            joints[f"{side}_{motor}.pos"] = float(value)
    return joints


def move_joints(robot, target: dict[str, float], move_s: float, max_speed: float, tol: float,
                label: str, fps: int = 50, stop_event: Event | None = None) -> tuple[bool, dict]:
    """target 관절만 부드럽게 옮긴다. 이미 1도 안이면 움직이지 않는다.

    도착 후 tol도 넘게 벗어난 관절이 있으면 더 밀지 않게 현재 위치를 목표로 고정하고 (False, 벗어난 관절)을 돌려준다.
    stop_event가 켜지면 이동을 끊고 현재 위치를 목표로 고정한다.
    """
    cur = read_joints(robot)
    start = {k: float(cur[k]) for k in target}
    biggest = max(abs(target[k] - start[k]) for k in target)
    if biggest < 1.0:
        log(f"{label}: 이미 도착(가장 큰 차이 {biggest:.1f}도)")
        return True, {}
    dur = max(move_s, biggest / max_speed)
    log(f"{label}: {dur:.1f}s 동안 이동 (가장 큰 차이 {biggest:.1f}도)")
    steps = max(int(dur * fps), 1)
    for i in range(1, steps + 1):
        if stop_event is not None and stop_event.is_set():
            now = read_joints(robot)
            send_arm_joints(robot, {k: float(now[k]) for k in target})
            log(f"{label}: 중단 - 현재 위치에서 정지")
            return False, {"stopped": True}
        t = i / steps
        t = t * t * (3 - 2 * t)
        send_arm_joints(robot, {k: start[k] * (1 - t) + target[k] * t for k in target})
        time.sleep(1 / fps)
    time.sleep(0.5)
    now = read_joints(robot)
    off = {k: round(float(now[k]) - target[k], 1) for k in target if abs(float(now[k]) - target[k]) > tol}
    if off:
        send_arm_joints(robot, {k: float(now[k]) for k in target})
        log(f"{label}: 도달 실패({tol:.0f}도 초과) {off} - 현재 위치에서 정지")
        return False, off
    log(f"{label}: 도착")
    return True, {}


# ---------------------------------------------------------------- 인자
def parse_service_args(argv: list[str]):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--listen", default="127.0.0.1:8770")
    ap.add_argument("--no_camera_fallback", action="store_true",
                    help="카메라 끊김 보호(camera_fallback.py)를 끈다. 끄면 프레임이 0.5초 넘게 끊길 때 관측 읽기가 실패한다")
    for t in TASKS:
        d = DEFAULTS[t]
        ap.add_argument(f"--{t}.policy", dest=f"{t}_policy", required=True)
        ap.add_argument(f"--{t}.task", dest=f"{t}_task", default=TASK_TEXT[t])
        ap.add_argument(f"--{t}.duration", dest=f"{t}_duration", type=float, default=d["duration"])
        ap.add_argument(f"--{t}.start_pose", dest=f"{t}_start_pose", default=d["start_pose"])
        ap.add_argument(f"--{t}.end_pose", dest=f"{t}_end_pose", default=d["end_pose"])
        ap.add_argument(f"--{t}.auto", dest=f"{t}_auto", choices=("off", "pour", "place"), default=d["auto"])
    ap.add_argument("--pour.hook", dest="pour_hook", action="store_true")
    ap.add_argument("--grip.margin", dest="margin", type=int, default=10)
    ap.add_argument("--grip.skip_pre", dest="skip_pre", action="store_true")
    ap.add_argument("--start.grippers", dest="start_grippers", action="store_true")
    ap.add_argument("--start.move_s", dest="start_move_s", type=float, default=5.0)
    ap.add_argument("--start.max_speed", dest="start_max_speed", type=float, default=20.0)
    ap.add_argument("--start.tol", dest="start_tol", type=float, default=10.0)
    ap.add_argument("--end.grippers", dest="end_grippers", action="store_true")
    ap.add_argument("--end.move_s", dest="move_s", type=float, default=5.0)
    ap.add_argument("--end.tol", dest="tol", type=float, default=10.0)
    ap.add_argument("--end.settle_s", dest="settle_s", type=float, default=1.0)
    ap.add_argument("--end.place_tol", dest="place_tol", type=float, default=20.0)
    ap.add_argument("--stow.pose", dest="stow_pose", default="none",
                    help="이동 중 유지할 저장 자세(예: place_end). none이면 이동 자세 기능을 끈다")
    ap.add_argument("--idle_check_s", type=float, default=1.0, help="대기 중 팔·카메라 상태를 읽는 간격")
    ns, rest = ap.parse_known_args(argv)
    hook_params, _, rest = parse_hook_args(rest)
    for key in RESERVED:
        if any(a == key or a.startswith(key + "=") for a in rest):
            sys.exit(f"{key} 는 이 실행기가 작업별로 채운다. --pour.* / --place.* 로 준다.")
    return ns, hook_params, rest


def task_settings(ns, task: str) -> dict:
    s = {k: getattr(ns, f"{task}_{k}") for k in ("policy", "task", "duration", "start_pose", "end_pose", "auto")}
    s["hook"] = task == "pour" and ns.pour_hook
    s["start_target"] = None
    if s["start_pose"] != "none":
        path, s["start_target"] = load_start_pose(s["start_pose"], ns.start_grippers)
        log(f"{task} 시작 자세 = {path} ({len(s['start_target'])}관절)")
    s["end_target"] = None
    if s["end_pose"] not in END_POSES:
        path, s["end_target"] = load_start_pose(s["end_pose"], ns.end_grippers)
        log(f"{task} 끝 자세 = {path} ({len(s['end_target'])}관절)")
    return s


def parse_rollout_config(args: list[str]):
    """lerobot-rollout과 같은 방식으로 RolloutConfig를 만든다(--policy.path를 sys.argv에서 읽는다)."""
    import lerobot.scripts.lerobot_rollout  # noqa: F401  로봇·카메라·텔레옵 설정 클래스를 등록한다
    from lerobot.configs import parser
    from lerobot.rollout import RolloutConfig
    from lerobot.utils.import_utils import register_third_party_plugins

    register_third_party_plugins()

    def _parse(cfg):
        return cfg

    # parser.wrap은 첫 인자 주석에서 설정 클래스를 읽는다(이 파일은 annotations가 문자열이라 직접 넣는다)
    _parse.__annotations__["cfg"] = RolloutConfig
    _parse = parser.wrap()(_parse)
    saved = sys.argv
    sys.argv = ["lerobot-rollout", *args]
    try:
        return _parse()
    finally:
        sys.argv = saved


# ---------------------------------------------------------------- 정책 준비
def attach_policy(cfg, base):
    """이미 연결된 팔·카메라(base 컨텍스트)를 함께 쓰는 두 번째 정책 컨텍스트를 만든다.

    lerobot.rollout.context.build_rollout_context의 정책·전후처리·추론 엔진 단계와 같다(하드웨어 연결만 생략).
    """
    from lerobot.configs import FeatureType
    from lerobot.policies import make_pre_post_processors
    from lerobot.rollout.context import (
        DatasetContext,
        PolicyContext,
        RolloutContext,
        RuntimeContext,
        _load_pretrained_policy,
        _resolve_action_key_order,
    )
    from lerobot.rollout.inference import create_inference_engine

    policy = _load_pretrained_policy(cfg.policy).to(cfg.device)
    policy.eval()

    robot = base.hardware.robot_wrapper.inner
    if not cfg.rename_map:
        expected = {k for k, v in cfg.policy.input_features.items() if v.type == FeatureType.VISUAL}
        provided = {f"observation.images.{k}" for k, v in robot.observation_features.items() if isinstance(v, tuple)}
        if not (expected <= provided or provided <= expected):
            raise ValueError(f"카메라 이름 불일치: 정책 {expected} / 로봇 {provided}")

    pre, post = make_pre_post_processors(
        policy_cfg=cfg.policy,
        pretrained_path=cfg.policy.pretrained_path,
        pretrained_revision=cfg.policy.pretrained_revision,
        dataset_stats=None,
        preprocessor_overrides={
            "device_processor": {"device": cfg.device},
            "rename_observations_processor": {"rename_map": cfg.rename_map},
        },
    )
    raw_keys = [k for k in robot.action_features if k.endswith((".pos", ".vel"))]
    names = getattr(cfg.policy, "action_feature_names", None)
    keys = _resolve_action_key_order(list(names) if names else None, raw_keys)
    engine = create_inference_engine(
        cfg.inference,
        policy=policy,
        preprocessor=pre,
        postprocessor=post,
        robot_wrapper=base.hardware.robot_wrapper,
        hw_features=base.data.hw_features,
        dataset_features=base.data.dataset_features,
        ordered_action_keys=keys,
        task=cfg.task,
        fps=cfg.fps,
        device=cfg.device,
        use_torch_compile=False,
        compile_warmup_inferences=cfg.compile_warmup_inferences,
        shutdown_event=Event(),
    )
    return RolloutContext(
        runtime=RuntimeContext(cfg=cfg, shutdown_event=Event()),
        hardware=base.hardware,
        policy=PolicyContext(policy=policy, preprocessor=pre, postprocessor=post, inference=engine),
        processors=base.processors,
        data=DatasetContext(dataset=None, dataset_features=base.data.dataset_features,
                            hw_features=base.data.hw_features, ordered_action_keys=keys),
    )


def read_observation(robot, tries: int = 10) -> dict:
    """카메라를 연 직후 첫 프레임이 늦는 경우가 있어 짧게 재시도한다."""
    for i in range(tries):
        try:
            return robot.get_observation()
        except TimeoutError:
            if i == tries - 1:
                raise
            time.sleep(0.3)
    raise RuntimeError("unreachable")


def warm_up(ctx) -> tuple[float, float]:
    """현재 관측으로 추론만 2번 돌린다(팔에는 아무것도 보내지 않는다). 첫 호출·두 번째 호출 시간을 돌려준다."""
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    engine = ctx.policy.inference
    times = []
    for _ in range(2):
        obs = read_observation(ctx.hardware.robot_wrapper)
        frame = build_dataset_frame(ctx.data.dataset_features, ctx.processors.robot_observation_processor(obs),
                                    prefix=OBS_STR)
        engine.reset()
        t = time.perf_counter()
        engine.get_action(frame)
        times.append(time.perf_counter() - t)
    engine.reset()
    return times[0], times[1]


# ---------------------------------------------------------------- 실행기
class Service:
    def __init__(self, ns, hook_params, contexts: dict, settings: dict, stow_target: dict | None = None):
        self.ns, self.hook_params = ns, hook_params
        self.contexts, self.settings = contexts, settings
        self.stow_target = stow_target
        self.arms = "unknown"
        self.jobs: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.state = "loading"
        self.current: str | None = None
        self.current_event: Event | None = None
        self.stop_requested = False
        self.results: list[dict] = []
        self.timings: dict = {}
        self.idle_failures = 0
        self.closing = False
        from lerobot.rollout.strategies import base as base_strategy

        self._base_strategy = base_strategy
        self._orig_send = base_strategy.send_next_action

    # -- HTTP 쪽에서 부른다
    def status(self) -> dict:
        with self.lock:
            return {"state": self.state, "current": self.current, "timings": self.timings,
                    "last_result": self.results[-1] if self.results else None,
                    "recent_results": self.results[-10:], "idle_read_failures": self.idle_failures,
                    "stow_pose": self.ns.stow_pose if self.stow_target else None, "arms": self.arms}

    def submit(self, task: str) -> tuple[int, dict, dict | None]:
        if task in ("stow", "stow_right") and self.stow_target is None:
            return HTTPStatus.NOT_FOUND, {"error": "stow_not_configured"}, None
        if task not in (*TASKS, "stow", "stow_right"):
            return HTTPStatus.NOT_FOUND, {"error": "unknown_task", "tasks": [*TASKS, "stow", "stow_right"]}, None
        with self.lock:
            if self.state != "ready":
                return HTTPStatus.CONFLICT, {"error": "not_ready", "state": self.state, "current": self.current}, None
            self.state = "running"
            self.current = task
        job = {"task": task, "done": Event(), "result": None, "submitted": time.perf_counter()}
        self.jobs.put(job)
        return HTTPStatus.ACCEPTED, {"accepted": task}, job

    def stop(self) -> dict:
        with self.lock:
            if self.current_event is None:
                return {"stopped": False, "reason": "no task running"}
            self.stop_requested = True
            self.current_event.set()
            return {"stopped": True, "task": self.current}

    def shutdown(self) -> dict:
        self.stop()
        self.jobs.put(None)
        return {"shutdown": True}

    # -- 메인 스레드
    def loop(self) -> None:
        while True:
            try:
                job = self.jobs.get(timeout=self.ns.idle_check_s)
            except queue.Empty:
                self._idle_check()
                continue
            if job is None:
                return
            result = self._run(job)
            with self.lock:
                self.results.append(result)
                self.state, self.current, self.current_event = "ready", None, None
            job["result"] = result
            job["done"].set()

    def _idle_check(self) -> None:
        # 대기 중에도 관측을 읽어 카메라 버퍼를 비우고 연결이 살아 있는지 확인한다(팔에는 명령을 보내지 않는다)
        try:
            next(iter(self.contexts.values())).hardware.robot_wrapper.get_observation()
            self.idle_failures = 0
        except Exception as e:  # noqa: BLE001
            self.idle_failures += 1
            if self.idle_failures in (1, 10) or self.idle_failures % 60 == 0:
                log(f"대기 중 관측 읽기 실패 {self.idle_failures}회: {e}")

    def _install_hooks(self, task: str, s: dict, timing: dict) -> None:
        bs = self._base_strategy
        bs.send_next_action = self._orig_send
        hook = None
        if s["hook"]:
            hook = PourHook(self.hook_params)
            install(hook)
        if s["auto"] != "off":
            install_done(DoneDetector(s["auto"], self.ns.tol, self.ns.settle_s, self.ns.place_tol))
        inner = bs.send_next_action

        def timed(obs_processed, obs_raw, ctx, interpolator):
            out = inner(obs_processed, obs_raw, ctx, interpolator)
            if out is not None and "first_action" not in timing:
                timing["first_action"] = time.perf_counter()
            if ctx.runtime.shutdown_event.is_set() and not self.stop_requested:
                timing.setdefault("auto_done", time.perf_counter())
            return out

        bs.send_next_action = timed
        timing["hook"] = hook

    def _run(self, job: dict) -> dict:
        from lerobot.rollout.context import RuntimeContext
        from lerobot.rollout.strategies.base import BaseStrategy

        task = job["task"]
        if task == "stow":
            return self._run_stow(job)
        if task == "stow_right":
            return self._run_stow_right(job)
        s = self.settings[task]
        base_ctx = self.contexts[task]
        event = Event()
        ctx = replace(base_ctx, runtime=RuntimeContext(cfg=base_ctx.runtime.cfg, shutdown_event=event))
        robot = ctx.hardware.robot_wrapper
        timing: dict = {}
        with self.lock:
            self.current_event, self.stop_requested = event, False
            self.arms = "busy"
        t0 = job["submitted"]
        log(f"호출 {task} -> 실행")
        result = {"task": task, "ok": False, "reason": "", "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        try:
            if s["start_target"] is not None:
                if not move_to_start_pose(robot, s["start_target"], self.ns.start_move_s,
                                          self.ns.start_max_speed, self.ns.start_tol, stop_event=event):
                    result["reason"] = "stopped" if event.is_set() else "start_pose_failed"
                    return result
            result["start_pose_s"] = round(time.perf_counter() - t0, 2)
            if event.is_set():
                result["reason"] = "stopped"
                return result
            self._install_hooks(task, s, timing)
            strategy = BaseStrategy(ctx.runtime.cfg.strategy)
            strategy.setup(ctx)
            t_run = time.perf_counter()
            try:
                strategy.run(ctx)
            finally:
                ctx.policy.inference.stop()
            run_s = time.perf_counter() - t_run
            hook = timing.get("hook")
            if self.stop_requested:
                result["reason"] = "stopped"
            elif hook is not None and hook.state == PourHook.FAULT:
                result["reason"] = "pour_hook_fault"
            elif "auto_done" in timing:
                result["ok"], result["reason"] = True, "auto_done"
            elif s["duration"] > 0 and run_s >= s["duration"] - 0.5:
                # 작업 끝을 감지하지 못한 채 최대 시간이 지났다. 성공 여부는 사람이 확인한다.
                result["reason"] = "duration"
            else:
                result["reason"] = "ended"
            result["run_s"] = round(run_s, 2)
            if "first_action" in timing:
                result["call_to_first_action_s"] = round(timing["first_action"] - t0, 2)
                result["policy_first_action_s"] = round(timing["first_action"] - t_run, 2)
            if hook is not None:
                result["pour_hook_state"] = hook.state
        except Exception as e:  # noqa: BLE001
            result["reason"] = f"error: {e}"
        finally:
            self._base_strategy.send_next_action = self._orig_send
            self._finish_pose(task, s, result)
            log(f"{task} 결과: {json.dumps(result, ensure_ascii=False)}")
        return result

    def _run_stow(self, job: dict) -> dict:
        """양팔(그리퍼 제외)을 이동 자세로 옮긴다. 차가 서 있을 때만 부른다(호출하는 쪽 책임)."""
        robot = next(iter(self.contexts.values())).hardware.robot_wrapper
        event = Event()
        with self.lock:
            self.current_event, self.stop_requested = event, False
            self.arms = "busy"
        log("호출 stow -> 양팔 이동 자세")
        result = {"task": "stow", "ok": False, "reason": "", "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        try:
            ok, off = move_joints(robot, self.stow_target, self.ns.start_move_s, self.ns.start_max_speed,
                                  self.ns.start_tol, f"이동 자세 '{self.ns.stow_pose}'", stop_event=event)
            result["ok"], result["reason"] = ok, "stowed" if ok else "stopped" if event.is_set() else "stow_failed"
            if off:
                result["off_joints"] = off
        except Exception as e:  # noqa: BLE001
            result["reason"] = f"error: {e}"
        result["stow_s"] = round(time.perf_counter() - job["submitted"], 2)
        with self.lock:
            self.arms = "stowed" if result["ok"] else "hold"
        log(f"stow 결과: {json.dumps(result, ensure_ascii=False)}")
        return result

    def _run_stow_right(self, job: dict) -> dict:
        """복구용: pour 직후 자동 오른팔 접기가 실패했을 때(예: 손목캠 끊김) 오른팔만 다시 접는다.

        왼팔(컵을 쥔 상판 적재 자세)은 움직이지 않는다. 성공하면 arms=carrying_cup이 되어 테이블 출발이 풀린다.
        사람이 병을 놓은 것을 확인하고 차가 서 있을 때만 부른다(호출하는 쪽 책임).
        """
        hw = self.contexts["pour"].hardware
        with self.lock:
            self.current_event, self.stop_requested = Event(), False
            self.arms = "busy"
        log("호출 stow_right -> 오른팔만 이동 자세(복구)")
        result = {"task": "stow_right", "ok": False, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        arms = self._stow_right_after_pour(hw, result)
        result["ok"] = arms == "carrying_cup"
        result["stow_s"] = round(time.perf_counter() - job["submitted"], 2)
        with self.lock:
            self.arms = arms
        log(f"stow_right 결과: {json.dumps(result, ensure_ascii=False)}")
        return result

    def _finish_pose(self, task: str, s: dict, result: dict) -> None:
        hw = self.contexts[task].hardware
        robot = hw.robot_wrapper.inner
        if not robot.is_connected:
            # 연결이 끊겨 끝 자세를 못 만들어도 "busy"로 남기지 않는다(관제·주행 쪽이 자세를 모른다는 뜻으로 hold)
            # 그리퍼는 모터 버스로 풀므로 카메라가 끊겨도 쥔 힘을 뺄 수 있다
            relax_grippers(robot, self.ns.margin, f"{task} 끝")
            with self.lock:
                self.arms = "hold"
            return
        # 작업 끝 감지도 정책을 끝내려고 current_event를 켠다. 끝 자세 이동은 새 이벤트로 /stop에만 반응한다
        with self.lock:
            self.current_event = Event()
            if self.stop_requested:
                self.current_event.set()
        # 중단·실패면 팔을 더 움직이지 않는다. 작업 끝 감지나 최대 시간 도달이면 rollout_handoff.py처럼 끝 자세로 간다.
        reason = result.get("reason", "")
        hold = s["end_pose"] == "final" or reason not in ("auto_done", "duration", "ended")
        arms = "hold"
        if hold:
            # 그 자리에서 유지하되 그리퍼가 계속 밀지 않게 한다(다음 호출까지 과부하 방지)
            relax_grippers(robot, self.ns.margin, f"{task} 끝")
            log(f"{task} 끝: 마지막 자세 유지")
            if task == "pour" and reason == "auto_done" and self.stow_target is not None:
                arms = self._stow_right_after_pour(hw, result)
        elif s["end_pose"] == "pour_start":
            make_pose_mover(POUR_START, self.ns.move_s)(hw)
        elif s["end_pose"] == "initial":
            make_pose_mover(hw.initial_position, self.ns.move_s, "연결 시 자세", self.ns.start_max_speed)(hw)
        else:
            make_pose_mover(s["end_target"], self.ns.move_s, f"저장 자세 '{s['end_pose']}'", self.ns.start_max_speed,
                            stop_event=self.current_event)(hw)
            if s["end_pose"] == self.ns.stow_pose and reason == "auto_done":
                arms = "stowed" if self._at(hw.robot_wrapper, s["end_target"]) else "hold"
        with self.lock:
            self.arms = arms

    def _stow_right_after_pour(self, hw, result: dict) -> str:
        """작업 끝 감지(오른손이 병을 놓고 왼팔이 상판 적재 자세)로 끝난 pour 뒤에 오른팔만 이동 자세로 옮긴다.

        왼팔은 컵을 쥔 채 상판 적재 자세를 유지한다(컵배치 정책의 시작 자세라 바꾸면 안 된다).
        최대 시간 도달(duration)은 병을 아직 쥐고 있을 수 있어 옮기지 않는다.
        """
        right = {k: v for k, v in self.stow_target.items() if k.startswith("right_")}
        try:
            ok, off = move_joints(hw.robot_wrapper, right, self.ns.start_move_s, self.ns.start_max_speed,
                                  self.ns.start_tol, f"오른팔 이동 자세 '{self.ns.stow_pose}'",
                                  stop_event=self.current_event)
        except Exception as e:  # noqa: BLE001
            ok, off = False, {"error": str(e)}
        result["right_stow"] = "ok" if ok else off
        return "carrying_cup" if ok else "hold"

    def _at(self, robot, target: dict[str, float]) -> bool:
        try:
            now = read_joints(robot)
            return all(abs(float(now[k]) - v) <= self.ns.start_tol for k, v in target.items())
        except Exception:  # noqa: BLE001
            return False

    def close(self) -> None:
        robot = next(iter(self.contexts.values())).hardware.robot_wrapper.inner
        if robot.is_connected:
            relax_grippers(robot, self.ns.margin, "종료 시")
            log("팔 연결 해제")
            robot.disconnect()


# ---------------------------------------------------------------- HTTP
def handler_factory(service: Service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args) -> None:
            if not self.path.startswith("/status"):
                log(f"HTTP {self.command} {self.path}")

        def _json(self, code: int, body: dict) -> None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            if urlparse(self.path).path == "/status":
                self._json(HTTPStatus.OK, service.status())
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            parts = url.path.strip("/").split("/")
            if len(parts) == 2 and parts[0] == "run":
                code, body, job = service.submit(parts[1])
                wait = parse_qs(url.query).get("wait", ["1"])[0] != "0"
                if job is not None and wait:
                    job["done"].wait()
                    code, body = HTTPStatus.OK, job["result"]
                self._json(code, body)
            elif url.path in ("/stow", "/stow_right"):
                code, body, job = service.submit(url.path.strip("/"))
                if job is not None:
                    job["done"].wait()
                    code, body = HTTPStatus.OK, job["result"]
                self._json(code, body)
            elif url.path == "/stop":
                self._json(HTTPStatus.OK, service.stop())
            elif url.path == "/shutdown":
                self._json(HTTPStatus.OK, service.shutdown())
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    return Handler


# ---------------------------------------------------------------- 명령줄 호출
def client(argv: list[str]) -> None:
    ap = argparse.ArgumentParser(prog="rollout_service.py call|stow|stow_right|status|stop|shutdown")
    ap.add_argument("command", choices=("call", "stow", "stow_right", "status", "stop", "shutdown"))
    ap.add_argument("task", nargs="?", choices=TASKS)
    ap.add_argument("--url", default=DEFAULT_URL)
    a = ap.parse_args(argv)
    if a.command == "call" and not a.task:
        sys.exit("call 다음에 pour 또는 place 를 준다.")
    path = {"call": f"/run/{a.task}", "stow": "/stow", "stow_right": "/stow_right", "status": "/status", "stop": "/stop",
            "shutdown": "/shutdown"}[a.command]
    req = urllib.request.Request(a.url + path, method="GET" if a.command == "status" else "POST", data=None)
    try:
        with urllib.request.urlopen(req, timeout=None) as r:
            print(json.dumps(json.loads(r.read()), ensure_ascii=False, indent=2))
    except urllib.error.HTTPError as e:
        print(json.dumps(json.loads(e.read()), ensure_ascii=False, indent=2))
        sys.exit(1)


# ---------------------------------------------------------------- 시작
def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] in ("call", "stow", "stow_right", "status", "stop", "shutdown"):
        client(sys.argv[1:])
        return
    ns, hook_params, rest = parse_service_args(sys.argv[1:])
    t_start = time.perf_counter()
    # lerobot(pyarrow)을 카메라 번호 찾기(Windows COM)보다 먼저 불러온다.
    # 순서가 반대면 pyarrow.dataset import에서 access violation으로 꺼진다(2026-10-29 실차 연결 때 재현).
    import lerobot.motors  # noqa: F401
    import lerobot.rollout  # noqa: F401
    rest = fill_camera_indices(rest)
    if not ns.no_camera_fallback:
        import camera_fallback

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
        from resolve_camera_indices import resolve

        camera_fallback.install({index: role for role, index in resolve().items()})
        log("카메라 끊김 보호 켬: 끊기면 마지막 프레임으로 계속하고 뒤에서 다시 연결")
    settings = {t: task_settings(ns, t) for t in TASKS}
    stow_target = None
    if ns.stow_pose != "none":
        path, stow_target = load_start_pose(ns.stow_pose, False)
        log(f"이동 자세 = {path} ({len(stow_target)}관절, 그리퍼 제외)")

    if not ns.skip_pre:
        ports = {s: p for s, p in (("left", arg_value(rest, "--robot.left_arm_config.port")),
                                    ("right", arg_value(rest, "--robot.right_arm_config.port"))) if p}
        pre_relax(ports, ns.margin)

    from lerobot.rollout import build_rollout_context
    from lerobot.rollout.inference import SyncInferenceConfig
    from lerobot.utils.utils import init_logging

    init_logging()
    cfgs = {}
    for t in TASKS:
        s = settings[t]
        cfgs[t] = parse_rollout_config([*rest, "--strategy.type=base", f"--policy.path={s['policy']}",
                                        f"--task={s['task']}", f"--duration={s['duration']}"])
        if not isinstance(cfgs[t].inference, SyncInferenceConfig):
            sys.exit("이 실행기는 --inference.type=sync(기본)만 지원한다.")

    host, port = ns.listen.rsplit(":", 1)
    timings: dict = {}
    log("물따르기 정책 로드 + 팔·카메라 연결")
    t = time.perf_counter()
    contexts = {"pour": build_rollout_context(cfgs["pour"], Event())}
    timings["pour_load_connect_s"] = round(time.perf_counter() - t, 1)
    service = Service(ns, hook_params, contexts, settings, stow_target)
    server = ThreadingHTTPServer((host, int(port)), handler_factory(service))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log(f"호출 주소 http://{host}:{port} (준비 중에는 /run 이 409 not_ready)")
    try:
        log("컵배치 정책 로드")
        t = time.perf_counter()
        contexts["place"] = attach_policy(cfgs["place"], contexts["pour"])
        timings["place_load_s"] = round(time.perf_counter() - t, 1)
        for name in TASKS:
            first, second = warm_up(contexts[name])
            timings[f"{name}_warmup_first_s"], timings[f"{name}_infer_s"] = round(first, 2), round(second, 2)
        timings["ready_total_s"] = round(time.perf_counter() - t_start, 1)
        with service.lock:
            service.timings, service.state = timings, "ready"
        log(f"준비 완료 {timings['ready_total_s']}s -> 호출 대기 {json.dumps(timings)}")
        service.loop()
    except KeyboardInterrupt:
        log("Ctrl+C -> 종료")
        service.stop()
    finally:
        with service.lock:
            service.state = "closing"
        server.shutdown()
        service.close()
        log("종료")


if __name__ == "__main__":
    main()
