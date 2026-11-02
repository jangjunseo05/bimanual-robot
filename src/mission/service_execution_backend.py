#!/usr/bin/env python3
"""Execution backends for service mission commands.

The default backend preserves the CPU-only dry-run.  The ROS 2 backend sends
only manipulation commands to ExecuteManipulationSkill; navigation and charge
remain explicit immediate mocks until their adapters are implemented.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import threading
from typing import Protocol


def _planned_ipc_module():
    """Load ROS-workspace IPC only when the roundtrip backend is selected."""
    from hold_flow_mission import planned_ipc

    return planned_ipc


def call_executor(*args, **kwargs):
    return _planned_ipc_module().call_executor(*args, **kwargs)


def execute_phase(*args, **kwargs):
    return _planned_ipc_module().execute_phase(*args, **kwargs)


def cancel_session(*args, **kwargs):
    return _planned_ipc_module().cancel_session(*args, **kwargs)


@dataclass(frozen=True)
class BackendResult:
    success: bool
    failure_code: str = ""
    adapter: str = "simulation_immediate"
    action_result: dict | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class CommandBackend(Protocol):
    mode: str

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult: ...

    def cancel_active(self) -> bool: ...

    def info(self) -> dict: ...

    def close(self) -> None: ...


class ImmediateBackend:
    """Marks every command successful without accessing ROS or hardware."""

    mode = "simulation_dry_run"

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult:
        del command, mission_id, phase_attempt
        return BackendResult(success=True)

    def cancel_active(self) -> bool:
        return False

    def info(self) -> dict:
        return {
            "mode": self.mode,
            "manipulation": "immediate_success",
            "navigation": "immediate_success",
            "charge": "immediate_success",
            "active_goal": False,
            "hardware_accessed": False,
        }

    def close(self) -> None:
        return None


class Ros2ManipulationBackend:
    """Routes manipulation to ROS 2 and keeps other commands as mocks."""

    mode = "ros2_manipulation_mock"

    def __init__(self, *, timeout_sec: float = 30.0) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be a positive finite number")
        try:
            import rclpy
            from rclpy.context import Context

            from hold_flow_mission.action_client import ManipulationActionClient
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 manipulation backend requires sourced Jazzy and install/setup.bash"
            ) from exc
        self.timeout_sec = float(timeout_sec)
        self._rclpy = rclpy
        self._context = Context()
        rclpy.init(context=self._context)
        self._client = ManipulationActionClient(context=self._context)

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult:
        if command.get("kind") != "manipulate":
            return BackendResult(
                success=True,
                adapter="simulation_immediate_non_manipulation",
            )
        if not mission_id:
            return BackendResult(False, "MISSION_ID_MISSING", self.mode)

        from hold_flow_mission.contract import ManipulationGoalSpec

        phase = str(command.get("phase", "UNKNOWN"))
        request_id = f"{mission_id}:{phase}:{phase_attempt}"
        try:
            spec = ManipulationGoalSpec.from_mission_command(
                command,
                request_id=request_id,
                mission_id=mission_id,
                timeout_sec=self.timeout_sec,
            )
            action_result = self._client.execute(spec)
        except RuntimeError as exc:
            return BackendResult(
                False,
                "ACTION_SERVER_UNAVAILABLE",
                self.mode,
                {"message": str(exc)},
            )
        except Exception as exc:
            return BackendResult(
                False,
                "BACKEND_EXCEPTION",
                self.mode,
                {"message": f"{type(exc).__name__}: {exc}"},
            )
        if not action_result.get("accepted"):
            return BackendResult(False, "GOAL_REJECTED", self.mode, action_result)
        if (
            action_result.get("mission_id") != mission_id
            or action_result.get("order_id") != command.get("order_id")
            or action_result.get("request_id") != request_id
        ):
            return BackendResult(False, "RESULT_ID_MISMATCH", self.mode, action_result)
        if action_result.get("success"):
            return BackendResult(True, adapter=self.mode, action_result=action_result)
        failure_code = str(action_result.get("failure_code") or "ACTION_ABORTED")
        return BackendResult(False, failure_code, self.mode, action_result)

    def cancel_active(self) -> bool:
        return self._client.cancel_active()

    def info(self) -> dict:
        return {
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill",
            "navigation": "immediate_success_mock",
            "charge": "immediate_success_mock",
            "timeout_sec": self.timeout_sec,
            "active_goal": self._client.has_active_goal(),
            "hardware_accessed": False,
        }

    def close(self) -> None:
        self._client.destroy_node()
        if self._context.ok():
            self._context.shutdown()


class Ros2PlannedArtifactBackend(Ros2ManipulationBackend):
    """Labels the shared Action client as verified artifact replay."""

    mode = "ros2_planned_artifact_replay"

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill (artifact replay)",
            "simulator_accessed": False,
        })
        return info


class Ros2PlannedSessionBackend(Ros2ManipulationBackend):
    """Labels the shared Action client as a persistent executor session."""

    mode = "ros2_planned_session"

    def execute(self, command: dict, *, mission_id: str | None, phase_attempt: int) -> BackendResult:
        result = super().execute(command, mission_id=mission_id, phase_attempt=phase_attempt)
        if result.action_result:
            try:
                evidence = json.loads(result.action_result.get('message', ''))
            except (TypeError, ValueError):
                evidence = None
            if isinstance(evidence, dict):
                self._executor_evidence = evidence
        return result

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill (persistent IPC session)",
            "executor_transport": "unix_socket_json",
            "hardware_accessed": False,
            "simulator_accessed": getattr(self, '_executor_evidence', {}).get('simulator_accessed'),
            "executor_kind": getattr(self, '_executor_evidence', {}).get('executor_kind', 'unconfirmed'),
            "scope": "table_1 cold water; dock navigation and charging are mocks",
        })
        return info


class Ros2PlannedRoundtripBackend(Ros2PlannedSessionBackend):
    """Runs navigation in the same Isaac world as ROS Action manipulation."""

    mode = "ros2_planned_roundtrip"
    scope = "dock_roundtrip"

    def __init__(self, *, executor_socket: Path, timeout_sec: float = 30.0) -> None:
        self.executor_socket = Path(executor_socket).expanduser()
        self._identity: dict[str, str] | None = None
        self._identity_lock = threading.Lock()
        self._executor_evidence = self._validate_executor(timeout_sec)
        self._executor_id = self._executor_evidence["executor_id"]
        super().__init__(timeout_sec=timeout_sec)

    def _validate_executor(self, timeout_sec: float) -> dict:
        try:
            status = call_executor(
                self.executor_socket,
                {"op": "status"},
                timeout_sec=min(float(timeout_sec), 5.0),
            )
        except (OSError, RuntimeError) as exc:
            code = getattr(exc, "code", "IPC_UNAVAILABLE")
            raise RuntimeError(f"roundtrip executor status failed ({code}): {exc}") from exc
        expected = {
            "success": True,
            "executor_kind": "isaac_physics",
            "scope": self.scope,
            "simulator_accessed": True,
            "hardware_accessed": False,
        }
        mismatches = [key for key, value in expected.items() if status.get(key) != value]
        if mismatches:
            raise RuntimeError(
                "roundtrip executor provenance mismatch: " + ", ".join(mismatches)
            )
        if status.get("mission_id") is not None or status.get("session_state") != "WAITING":
            raise RuntimeError("roundtrip executor world is not fresh and waiting")
        if not isinstance(status.get("executor_id"), str) or not status["executor_id"]:
            raise RuntimeError("roundtrip executor provenance mismatch: executor_id")
        return status

    def validate_order(self, payload: dict, *, existing_order_ids: set[str]) -> None:
        order_id = payload.get("order_id")
        if payload.get("table_id") != "table_1" or payload.get("drink") != "COLD_WATER":
            raise ValueError("ros2-planned-roundtrip supports table_1/COLD_WATER only")
        if existing_order_ids and order_id not in existing_order_ids:
            raise ValueError("ros2-planned-roundtrip allows one order per Isaac world")

    def _remember_identity(self, command: dict, mission_id: str | None) -> dict[str, str] | None:
        with self._identity_lock:
            if mission_id:
                candidate = {
                    "mission_id": mission_id,
                    "order_id": str(command.get("order_id") or ""),
                    "table_id": str(command.get("table_id") or ""),
                    "drink": str(command.get("drink") or ""),
                }
                if all(candidate.values()):
                    if self._identity is not None and self._identity != candidate:
                        raise RuntimeError("roundtrip world identity changed")
                    self._identity = candidate
            return dict(self._identity) if self._identity is not None else None

    def execute(self, command: dict, *, mission_id: str | None, phase_attempt: int) -> BackendResult:
        try:
            identity = self._remember_identity(command, mission_id)
        except RuntimeError as exc:
            return BackendResult(False, "WORLD_IDENTITY_MISMATCH", self.mode, {"message": str(exc)})
        if command.get("kind") == "navigate":
            phase = str(command.get("phase") or "")
            if phase not in {"NAVIGATE_KITCHEN", "NAVIGATE_TABLE", "NAVIGATE_DOCK"}:
                return BackendResult(False, "NAVIGATION_PHASE_UNSUPPORTED", self.mode)
            if identity is None:
                return BackendResult(False, "MISSION_IDENTITY_MISSING", self.mode)
            request_id = f"{identity['mission_id']}:{phase}:{phase_attempt}"
            try:
                result = execute_phase(
                    self.executor_socket,
                    request_id=request_id,
                    mission_id=identity["mission_id"],
                    order_id=identity["order_id"],
                    phase_id=phase,
                    table_id=identity["table_id"],
                    drink=identity["drink"],
                    executor_id=self._executor_id,
                    timeout_sec=self.timeout_sec,
                )
            except Exception as exc:
                planned_ipc_error = _planned_ipc_module().PlannedIpcError
                if not isinstance(exc, planned_ipc_error):
                    raise
                detail = {"message": str(exc), "result_state": "unknown"}
                if exc.code == "IPC_TIMEOUT":
                    detail["cancel_requested"] = self._cancel_identity(identity)
                    return BackendResult(False, "IPC_TIMEOUT_RESULT_UNKNOWN", self.mode, detail)
                return BackendResult(False, exc.code, self.mode, detail)
            self._executor_evidence = result
            if result.get("success"):
                return BackendResult(True, adapter=self.mode, action_result=result)
            return BackendResult(
                False,
                str(result.get("failure_code") or "EXECUTOR_FAILED"),
                self.mode,
                result,
            )
        if command.get("kind") == "charge":
            return BackendResult(
                True,
                adapter="battery_model_only",
                action_result={"message": "charge is controller-model-only; no physical charging"},
            )
        if command.get("kind") == "manipulate":
            return super().execute(command, mission_id=mission_id, phase_attempt=phase_attempt)
        return BackendResult(False, "COMMAND_UNSUPPORTED", self.mode)

    def _cancel_identity(self, identity: dict[str, str]) -> bool:
        try:
            return cancel_session(
                self.executor_socket,
                identity["mission_id"],
                executor_id=self._executor_id,
                timeout_sec=1.0,
            )
        except (OSError, RuntimeError):
            return False

    def cancel_active(self) -> bool:
        action_requested = super().cancel_active()
        with self._identity_lock:
            identity = dict(self._identity) if self._identity is not None else None
        ipc_requested = self._cancel_identity(identity) if identity is not None else False
        return action_requested or ipc_requested

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "navigation": "Isaac physics via planned executor IPC",
            "charge": "controller_model_only (not physical charging)",
            "executor_socket": str(self.executor_socket),
            "executor_kind": self._executor_evidence.get("executor_kind", "unconfirmed"),
            "executor_id": self._executor_id,
            "simulator_accessed": self._executor_evidence.get("simulator_accessed"),
            "scope": self.scope,
            "single_order_only": True,
            "supported_order": {"table_id": "table_1", "drink": "COLD_WATER"},
            "world_resume_supported": False,
        })
        return info


class RolloutServiceBackend:
    """Runs the two manipulation policies through the resident rollout service (tools/rollout/rollout_service.py).

    The service keeps both policies loaded and the arms and cameras connected for the whole scenario,
    so a call only waits for the start-pose move and the policy itself.  One policy run covers several
    mission phases, and each policy runs at most once per mission:

        ALIGN_KITCHEN / ALIGN_TABLE   wait for the "alignment done" signal (operator button or navigation)
        GRASP_CUP                     run the pour policy (grasp, pour, return bottle, load cup on deck)
        GRASP_BOTTLE .. PLACE_DECK    report the cached pour result
        REGRASP_CUP                   run the place policy (regrasp cup, place it on the guest table)
        SERVE                         report the cached place result

    A retried phase reports the cached result instead of moving the arms again.  Navigation and charge
    stay immediate mocks until the real-vehicle adapter exists.

    Driving pose: when the service runs with --stow.pose, NAVIGATE_KITCHEN first asks it to fold both arms
    into that pose (POST /stow) while the vehicle is still at the dock.  The service itself folds the right
    arm after a pour that ended normally (the left arm keeps holding the cup) and ends the place run in the
    same pose.  info()["drive"] tells the navigation side where to go next and whether the arms are folded.
    """

    mode = "rollout_service"
    POLICY_PHASES = {
        "GRASP_CUP": ("pour", True), "GRASP_BOTTLE": ("pour", False), "POUR": ("pour", False),
        "RETURN_BOTTLE": ("pour", False), "PLACE_DECK": ("pour", False),
        "REGRASP_CUP": ("place", True), "SERVE": ("place", False),
    }
    ALIGN_PHASES = {"ALIGN_KITCHEN": "kitchen", "ALIGN_TABLE": "table"}
    FAILURE_BY_REASON = {
        "duration": "END_NOT_DETECTED",
        "stopped": "STOPPED",
        "start_pose_failed": "START_POSE_FAILED",
        "pour_hook_fault": "POUR_HOOK_FAULT",
    }

    def __init__(
        self,
        *,
        url: str = "http://127.0.0.1:8770",
        require_alignment: bool = True,
        accept_duration: bool = False,
        run_timeout_s: float = 600.0,
        alignment_timeout_s: float = 3600.0,
    ) -> None:
        for name, value in (("run_timeout_s", run_timeout_s), ("alignment_timeout_s", alignment_timeout_s)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a positive finite number")
        self.url = url.rstrip("/")
        self.require_alignment = require_alignment
        self.accept_duration = accept_duration
        self.run_timeout_s = float(run_timeout_s)
        self.alignment_timeout_s = float(alignment_timeout_s)
        self._lock = threading.Lock()
        self._aligned = threading.Event()
        self._cancel = threading.Event()
        self._halted = threading.Event()
        self._waiting: dict | None = None
        self._running: str | None = None
        self._results: dict[tuple[str, str], dict] = {}
        self._stowed_missions: set[str] = set()
        self._next_drive: str | None = None
        self._status_cache: tuple[float, dict] = (0.0, {})

    def _request(self, method: str, path: str, timeout: float) -> tuple[int, dict]:
        import urllib.error
        import urllib.request

        request = urllib.request.Request(self.url + path, method=method)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return response.status, json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, json.loads(exc.read() or b"{}")
            except ValueError:
                return exc.code, {}

    def service_status(self, max_age_s: float = 2.0) -> dict:
        """Rollout service /status, cached briefly because the web dashboard polls every second."""
        import time

        checked, cached = self._status_cache
        if time.monotonic() - checked < max_age_s:
            return cached
        try:
            status = self._request("GET", "/status", timeout=0.5)[1]
        except OSError as exc:
            status = {"state": "unreachable", "error": str(exc)}
        self._status_cache = (time.monotonic(), status)
        return status

    def confirm_alignment(self, station: str | None = None) -> dict:
        """Release the waiting ALIGN_* phase (operator button now, navigation signal later)."""
        with self._lock:
            waiting = self._waiting
            if waiting is None:
                return {"confirmed": False, "reason": "not_waiting"}
            if station and station != waiting["station"]:
                return {"confirmed": False, "reason": "station_mismatch", "waiting_for": waiting["station"]}
            self._aligned.set()
            return {"confirmed": True, "station": waiting["station"]}

    def _wait_alignment(self, station: str, phase: str, order_id: str | None) -> BackendResult:
        with self._lock:
            self._aligned.clear()
            self._cancel.clear()
            self._waiting = {"station": station, "phase": phase, "order_id": order_id}
        try:
            remaining = self.alignment_timeout_s
            while remaining > 0:
                if self._aligned.wait(0.2):
                    return BackendResult(True, adapter=self.mode, action_result={"alignment": station})
                if self._cancel.is_set():
                    return BackendResult(False, "CANCELED_WHILE_WAITING_ALIGNMENT", self.mode)
                remaining -= 0.2
            return BackendResult(False, "ALIGNMENT_TIMEOUT", self.mode)
        finally:
            with self._lock:
                self._waiting = None

    def _run_policy(self, task: str, path: str | None = None) -> dict:
        import time

        with self._lock:
            self._running = task
            self._cancel.clear()
        if self._halted.is_set():
            with self._lock:
                self._running = None
            return {"ok": False, "failure_code": "EMERGENCY_STOP"}
        try:
            # Right after scenario start the service may still be loading policies (409, state=loading): wait.
            deadline = time.monotonic() + self.run_timeout_s
            while True:
                status, body = self._request("POST", path or f"/run/{task}", timeout=self.run_timeout_s)
                loading = status == 409 and body.get("error") == "not_ready" and body.get("state") == "loading"
                if not loading or time.monotonic() > deadline or self._cancel.wait(1.0):
                    break
        except OSError as exc:
            return {"ok": False, "failure_code": "ROLLOUT_SERVICE_UNREACHABLE", "message": str(exc)}
        finally:
            with self._lock:
                self._running = None
            # the arms just moved: do not show the cached pre-call status (arms, last result) to the dashboard
            self._status_cache = (0.0, {})
        if status != 200:
            error = str(body.get("error", status)).upper()
            return {"ok": False, "failure_code": f"ROLLOUT_SERVICE_{error}", "service": body}
        reason = str(body.get("reason", ""))
        if body.get("ok") or (reason == "duration" and self.accept_duration):
            return {"ok": True, "result": body}
        code = self.FAILURE_BY_REASON.get(reason, "POLICY_ERROR")
        return {"ok": False, "failure_code": f"{task.upper()}_{code}", "result": body}

    def _stow(self, mission_key: str) -> BackendResult:
        """Fold both arms into the driving pose once per mission, before the first drive."""
        with self._lock:
            if mission_key in self._stowed_missions:
                return BackendResult(True, adapter=self.mode, action_result={"stow": "already_done"})
        status, body = None, {}
        try:
            status = self._request("GET", "/status", timeout=2.0)[1]
        except OSError as exc:
            return BackendResult(False, "ROLLOUT_SERVICE_UNREACHABLE", self.mode, {"message": str(exc)})
        if status.get("state") not in ("loading", "ready", "running") or (
                status.get("state") != "loading" and not status.get("stow_pose")):
            # Service without --stow.pose: keep the old behavior (arms are not moved before driving).
            return BackendResult(True, adapter=self.mode, action_result={"stow": "not_configured",
                                                                         "service_state": status.get("state")})
        outcome = self._run_policy("stow", "/stow")
        if outcome.get("failure_code") == "ROLLOUT_SERVICE_STOW_NOT_CONFIGURED":
            return BackendResult(True, adapter=self.mode, action_result={"stow": "not_configured"})
        if not outcome["ok"]:
            code = outcome.get("failure_code") or "STOW_FAILED"
            return BackendResult(False, code if code.startswith("ROLLOUT_SERVICE_") else "STOW_FAILED",
                                 self.mode, outcome)
        with self._lock:
            self._stowed_missions.add(mission_key)
        return BackendResult(True, adapter=self.mode, action_result={"stow": outcome.get("result")})

    def execute(self, command: dict, *, mission_id: str | None, phase_attempt: int) -> BackendResult:
        del phase_attempt
        if self._halted.is_set():
            return BackendResult(False, "EMERGENCY_STOP", self.mode)
        if command.get("kind") == "navigate":
            destination = str(command.get("destination") or "")
            with self._lock:
                self._next_drive = destination or None
            if destination == "kitchen":
                return self._stow(str(mission_id or command.get("order_id") or ""))
            return BackendResult(True, adapter="simulation_immediate_non_manipulation")
        if command.get("kind") != "manipulate":
            return BackendResult(True, adapter="simulation_immediate_non_manipulation")
        phase = str(command.get("phase", ""))
        if phase in self.ALIGN_PHASES:
            with self._lock:
                self._next_drive = self.ALIGN_PHASES[phase]
            if not self.require_alignment:
                return BackendResult(True, adapter=self.mode)
            return self._wait_alignment(self.ALIGN_PHASES[phase], phase, command.get("order_id"))
        if phase not in self.POLICY_PHASES:
            return BackendResult(False, "UNSUPPORTED_PHASE", self.mode, {"phase": phase})
        task, runs_policy = self.POLICY_PHASES[phase]
        key = (str(mission_id or command.get("order_id") or ""), task)
        with self._lock:
            cached = self._results.get(key)
        if cached is None:
            if not runs_policy:
                return BackendResult(False, f"{task.upper()}_NOT_RUN", self.mode)
            with self._lock:
                self._next_drive = None
            cached = self._run_policy(task)
            with self._lock:
                self._results[key] = cached
                if task == "place" and cached["ok"]:
                    self._next_drive = "dock"
        detail = {"policy": task, "covered_phase": phase, **{k: v for k, v in cached.items() if k != "ok"}}
        if cached["ok"]:
            return BackendResult(True, adapter=self.mode, action_result=detail)
        return BackendResult(False, cached["failure_code"], self.mode, detail)

    def cancel_active(self) -> bool:
        with self._lock:
            waiting, running = self._waiting, self._running
        if waiting is not None:
            self._cancel.set()
            return True
        if running is not None:
            self._cancel.set()
            try:
                self._request("POST", "/stop", timeout=3.0)
                return True
            except OSError:
                return False
        return False

    def emergency_stop(self) -> bool:
        """Hold every later arm command and stop the running one.

        The second /stop catches a run request that was already on its way when the first one arrived.
        """
        import time

        self._halted.set()
        self._cancel.set()
        sent = False
        for delay_s in (0.0, 0.3):
            time.sleep(delay_s)
            try:
                self._request("POST", "/stop", timeout=1.0)
                sent = True
            except OSError:
                pass
        return sent

    def release_emergency_stop(self) -> None:
        self._halted.clear()

    def info(self) -> dict:
        with self._lock:
            waiting = dict(self._waiting) if self._waiting else None
            running = self._running
        status = self.service_status()
        with self._lock:
            next_drive = None if running is not None else self._next_drive
        arms = status.get("arms")
        return {
            "mode": self.mode,
            "manipulation": f"rollout_service {self.url}",
            "navigation": "immediate_success_mock",
            "charge": "immediate_success_mock",
            "active_goal": running is not None,
            "running_policy": running,
            "waiting_alignment": waiting,
            "require_alignment": self.require_alignment,
            "manipulation_service_state": status.get("state"),
            "manipulation_service_timings": status.get("timings"),
            "manipulation_service_current": status.get("current"),
            "manipulation_last_result": status.get("last_result"),
            "manipulation_idle_read_failures": status.get("idle_read_failures"),
            "stow_pose": status.get("stow_pose"),
            "arms": arms,
            # For the navigation side: where the vehicle should be heading now and whether the arms are folded
            # for driving.  Never drive while running_policy is set or arms is busy/hold/unknown.
            "drive": {
                "next": next_drive,
                "arms_ready": running is None and arms in ("stowed", "carrying_cup"),
            },
            "hardware_accessed": True,
        }

    def close(self) -> None:
        self._cancel.set()
