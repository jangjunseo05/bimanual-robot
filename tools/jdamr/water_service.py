#!/usr/bin/env python3
"""Coordinate one water-service mission across the AMR and Windows order server host.

This module deliberately delegates all robot movement to ``jdamr_depart.py``.  It
uses only the Windows order server: bind one order, wait for arm-ready drive
permission, confirm each parked station, and resume after manipulation finishes.
"""

from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, quote
from urllib.request import Request, urlopen
from uuid import uuid4


DEPART_HELPER = Path(__file__).with_name("jdamr_depart.py")
DEFAULT_MAP_POINTER = Path.home() / "jdamr_data/maps/current_manual_mapping.json"


class _Stopped(RuntimeError):
    pass


class WaterServiceController:
    """Own one non-overlapping navigation or navigation-plus-rollout mission."""

    def __init__(
        self,
        *,
        inference_url: str | None,
        state_directory: Path,
        map_session: Path | None = None,
        inference_timeout_s: float = 300.0,
        run_inference: bool = True,
    ) -> None:
        self.inference_url = inference_url.rstrip("/") if inference_url else None
        self.run_inference = bool(run_inference)
        self.state_directory = Path(state_directory).expanduser().resolve()
        self.map_session = Path(map_session or DEFAULT_MAP_POINTER).expanduser().resolve()
        self.inference_timeout_s = float(inference_timeout_s)
        if self.inference_timeout_s <= 0:
            raise ValueError("inference_timeout_s must be positive")

        self.state_directory.mkdir(parents=True, exist_ok=True)
        self._state_path = self.state_directory / "state.json"
        self._lock = threading.RLock()
        self._inference_gate = threading.Lock()
        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._process: subprocess.Popen[str] | None = None
        self._log_path: Path | None = None
        self._state = self._load_state()
        self._announced: set[str] = set()
        if self._state.get("state") == "running":
            self._state.update(
                state="failed",
                phase="interrupted",
                error="controller restarted while a mission was running",
                finished_unix_s=time.time(),
            )
            self._record_request_terminal()
            self._save_state()

    def start(self, request_id: str | None = None) -> dict[str, Any]:
        """Accept a mission and run it on a background thread."""
        if self.run_inference and not self.inference_url:
            raise ValueError("order_url is not configured; refusing to move the robot")
        if not self.map_session.is_file():
            raise ValueError(f"map session pointer is missing: {self.map_session}")

        with self._lock:
            request_id = request_id or uuid4().hex
            previous = self._state.setdefault("requests", {}).get(request_id)
            if previous is not None:
                response = deepcopy(previous)
                response.update(accepted=False, duplicate=True, request_id=request_id)
                return response
            if self._worker_thread is not None and self._worker_thread.is_alive():
                raise RuntimeError("another water-service mission is already running")

            mission_id = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:8]
            mission_dir = self.state_directory / "runs" / mission_id
            mission_dir.mkdir(parents=True, exist_ok=False)
            self._log_path = mission_dir / "controller.log"
            self._stop_event.clear()
            mode = "inference" if self.run_inference else "navigation_only"
            initial_phase = "checking_inference" if self.run_inference else "navigation_cycle"
            self._state.update(
                state="running",
                phase=initial_phase,
                mode=mode,
                mission_id=mission_id,
                request_id=request_id,
                inference_url=self.inference_url,
                configured=bool(self.inference_url) if self.run_inference else True,
                error=None,
                started_unix_s=time.time(),
                finished_unix_s=None,
                mission_directory=str(mission_dir),
                parking_run=None,
                order_id=None,
                windows_result=None,
                table_parking_run=None,
                table_parking_log_on_pi=None,
                windows_place_result=None,
            )
            self._state["requests"][request_id] = {
                "mission_id": mission_id,
                "state": "running",
                "phase": initial_phase,
                "mode": mode,
                "error": None,
            }
            self._save_state()
            self._worker_thread = threading.Thread(
                target=self._run_mission,
                args=(mission_dir,),
                name=f"water-service-{mission_id}",
                daemon=True,
            )
            self._worker_thread.start()
            return {
                "accepted": True,
                "duplicate": False,
                "request_id": request_id,
                "mission_id": mission_id,
                "state": "running",
                "phase": initial_phase,
                "mode": mode,
            }

    def status(self) -> dict[str, Any]:
        with self._lock:
            view = deepcopy(self._state)
            view["last_mission_mode"] = view.get("mode")
            view["mode"] = "inference" if self.run_inference else "navigation_only"
            view["inference_url"] = self.inference_url
            view["order_url"] = self.inference_url
            view["configured"] = bool(self.inference_url) if self.run_inference else True
            return view

    def stop(self) -> dict[str, Any]:
        """Cancel only the active stage; cancellation never starts dock return."""
        with self._lock:
            active = self._worker_thread is not None and self._worker_thread.is_alive()
            if not active:
                return self.status()
            phase = str(self._state.get("phase"))
            self._stop_event.set()
            self._state.update(state="stopped", phase="stopping", error="stop requested")
            self._record_request_terminal()
            self._save_state()

        errors: list[str] = []
        if phase in ("parking", "table_one_and_dock", "parking_table_one",
                     "returning_to_dock", "navigation_cycle"):
            try:
                self._run_stop_helper()
            except Exception as error:
                errors.append(f"AMR stop: {error}")

        if self.run_inference:
            try:
                # Order cancellation after an in-flight alignment/order request.
                with self._inference_gate:
                    if self._state.get("order_id"):
                        self._http_json("POST", "/api/orders/" + quote(str(self._state["order_id"]), safe="")
                                        + "/cancel", timeout_s=10.0, body={})
            except Exception as error:  # the local stop still remains authoritative
                errors.append(f"Windows stop: {error}")

        with self._lock:
            if errors:
                self._state["stop_errors"] = errors
                self._save_state()
            return self.status()

    def read_web_state(self) -> dict[str, Any]:
        return self._http_json("GET", "/api/state", timeout_s=2.0)

    def report_web(self, *, dry_run: bool) -> None:
        """Tell the control page that this PC follows it and what the mission is doing."""
        with self._lock:
            body = {key: self._state.get(key) for key in ("state", "phase", "mission_id", "error")}
        self._http_json("POST", "/api/drive/heartbeat", timeout_s=2.0, body={**body, "dry_run": dry_run})

    def follow_web(self, state: dict[str, Any], *, dry_run: bool = False) -> str | None:
        """Act once on the order server state: STOP first, otherwise depart for a newly started order.

        The arms-ready kitchen request is the departure signal, so nothing moves before the
        control page Start has folded the arms.  With dry_run only the decision is reported.
        """
        with self._lock:
            running = self._worker_thread is not None and self._worker_thread.is_alive()
        emergency = state.get("emergency_stop") or {}
        if emergency.get("active"):
            if dry_run:
                return self._announce_once(f"stop-{emergency.get('at')}", "would_stop")
            if running:
                self.stop()
                return "stop"
            return None
        if running:
            return None
        backend = state.get("execution_backend") or {}
        drive = backend.get("drive") or {}
        waiting = backend.get("waiting_alignment") or {}
        order_id = waiting.get("order_id")
        if (not order_id or waiting.get("station") != "kitchen" or drive.get("next") != "kitchen"
                or drive.get("arms_ready") is not True):
            return None
        request_id = f"web-{order_id}"
        if dry_run:
            return self._announce_once(request_id, f"would_start {order_id}")
        with self._lock:
            if request_id in self._state.setdefault("requests", {}):
                return None
        self.start(request_id)
        return f"start {order_id}"

    def _announce_once(self, key: str, message: str) -> str | None:
        if key in self._announced:
            return None
        self._announced.add(key)
        return message

    def _run_mission(self, mission_dir: Path) -> None:
        try:
            if not self.run_inference:
                self._set_phase("navigation_cycle")
                self._run_helper(["go", "table_01", "--dwell-s", "0"])
                self._check_stopped()
                self._finish("succeeded", "complete", None)
                return
            self._check_stopped()
            self._bind_order()
            self._wait_drive("kitchen")

            self._check_stopped()
            self._set_phase("parking")
            before = self._read_navigation_state()
            parking_started = time.time()
            self._run_helper(["go", "water_station", "--park-only"])
            self._check_stopped()
            parking_run = self._new_parking_run(before, parking_started)
            remote_log = self._remote_cycle_log(parking_run)
            with self._lock:
                self._state["parking_run"] = str(parking_run)
                self._state["parking_log_on_pi"] = remote_log
                self._save_state()

            self._check_stopped()
            self._set_phase("inferencing")
            self._confirm_alignment("kitchen")
            self._wait_drive("table")

            self._check_stopped()
            self._set_phase("parking_table_one")
            before = self._read_navigation_state()
            parking_started = time.time()
            self._run_helper([
                "go",
                "table_01",
                "--resume-parked-log",
                remote_log,
                "--park-only",
                "--dwell-s",
                "0",
            ])
            self._check_stopped()
            table_run = self._new_parking_run(before, parking_started, table_id="table_01")
            table_log = self._remote_cycle_log(table_run)
            with self._lock:
                self._state["table_parking_run"] = str(table_run)
                self._state["table_parking_log_on_pi"] = table_log
                self._save_state()
            self._set_phase("placing_cup")
            self._confirm_alignment("table")
            self._wait_drive("dock")
            self._check_stopped()
            self._set_phase("returning_to_dock")
            self._run_helper([
                "go", "table_01", "--skip-via", "--resume-parked-log",
                table_log, "--dwell-s", "0",
            ])
            self._check_stopped()
            self._finish("succeeded", "complete", None)
        except _Stopped:
            self._finish("stopped", "stopped", "stop requested")
        except Exception as error:
            if self._stop_event.is_set():
                self._finish("stopped", "stopped", "stop requested")
            else:
                self._finish("failed", "failed", str(error))
        finally:
            with self._lock:
                self._process = None

    def _run_helper(self, arguments: list[str]) -> None:
        command = [sys.executable, str(DEPART_HELPER), "--map-session", str(self.map_session)]
        command.extend(arguments)
        assert self._log_path is not None
        with self._log_path.open("a", encoding="utf-8") as output:
            output.write("COMMAND " + json.dumps(command, ensure_ascii=False) + "\n")
            output.flush()
            with self._lock:
                self._check_stopped()
                process = subprocess.Popen(
                    command,
                    cwd=str(DEPART_HELPER.parents[2]),
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                self._process = process
            checked_s = 0.0
            destination = {"parking": "kitchen", "parking_table_one": "table",
                           "returning_to_dock": "dock"}.get(self._state.get("phase"))
            while process.poll() is None:
                if self._stop_event.wait(0.25):
                    # stop() sends the maintained helper's cancellation command.
                    continue
                if self.run_inference and destination and time.monotonic() - checked_s >= 1.0:
                    checked_s = time.monotonic()
                    try:
                        state = self._http_json("GET", "/api/state", timeout_s=3.0)
                        if not self._drive_ready(state, destination):
                            raise RuntimeError("arm-ready drive permission withdrawn")
                    except Exception:
                        self._run_stop_helper()
                        raise
            with self._lock:
                if self._process is process:
                    self._process = None
            if process.returncode != 0:
                if self._stop_event.is_set():
                    raise _Stopped()
                raise RuntimeError(
                    f"jdamr_depart.py exited with code {process.returncode}; see {self._log_path}"
                )

    def _run_stop_helper(self) -> None:
        command = [
            sys.executable,
            str(DEPART_HELPER),
            "--map-session",
            str(self.map_session),
            "stop",
        ]
        if self._log_path is None:
            return
        with self._log_path.open("a", encoding="utf-8") as output:
            process = subprocess.Popen(
                command,
                cwd=str(DEPART_HELPER.parents[2]),
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                code = process.wait(timeout=30.0)
            except subprocess.TimeoutExpired as error:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
                raise RuntimeError("jdamr_depart.py stop did not confirm within 30 s") from error
            if code != 0:
                raise RuntimeError(f"jdamr_depart.py stop exited with code {code}")

    def _new_parking_run(self, before: dict[str, Any], started_s: float,
                         table_id: str = "water_station") -> Path:
        after = self._read_navigation_state()
        previous = before.get("last_run")
        current = after.get("last_run")
        if not current or current == previous:
            raise RuntimeError("parking completed without a new last_run")
        run = Path(str(current)).expanduser().resolve()
        session_dir = self._session_directory()
        expected_runs = (session_dir / "runtime/runs").resolve()
        if run.parent != expected_runs or not run.is_dir():
            raise RuntimeError(f"new last_run is outside the selected map session: {run}")
        command = run / "command.txt"
        if not command.is_file() or command.stat().st_mtime < started_s - 1.0:
            raise RuntimeError("new last_run has no command file matching this request")
        text = command.read_text(encoding="utf-8")
        if (f"--table-id {table_id}" not in text or "--search" not in text
                or "--return-home" in text):
            raise RuntimeError(f"new last_run is not this {table_id} parking-only request")
        return run

    def _read_navigation_state(self) -> dict[str, Any]:
        state_path = self._session_directory() / "runtime/state.json"
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise RuntimeError(f"cannot read navigation state: {state_path}: {error}") from error
        if not isinstance(value, dict):
            raise RuntimeError(f"navigation state is not an object: {state_path}")
        return value

    def _session_directory(self) -> Path:
        try:
            pointer = json.loads(self.map_session.read_text(encoding="utf-8"))
            directory = Path(os.path.expandvars(pointer["directory"])).expanduser()
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise RuntimeError(f"invalid map session pointer: {self.map_session}: {error}") from error
        if not directory.is_absolute():
            directory = self.map_session.parent / directory
        return directory.resolve()

    def _remote_cycle_log(self, run: Path) -> str:
        data_root = Path(os.environ.get("JDAMR_DATA_ROOT", Path.home() / "jdamr_data"))
        data_root = data_root.expanduser().resolve()
        try:
            relative = run.relative_to(data_root)
        except ValueError as error:
            raise RuntimeError(f"parking run is outside JDAMR_DATA_ROOT: {run}") from error
        pi_home = os.environ.get("JDAMR_PI_HOME")
        if not pi_home:
            user = os.environ.get("JDAMR_PI_USER")
            if not user:
                host = os.environ.get(
                    "JDAMR_PI_HOST", os.environ.get("JDAMR_HOST", "lim@192.168.0.159")
                )
                user = host.split("@", 1)[0] if "@" in host else "lim"
            pi_home = f"/home/{user}"
        return str(Path(pi_home) / "jdamr_data" / relative / "cycle_events.jsonl")

    def _http_json(
        self, method: str, path: str, *, timeout_s: float, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        _raw, value = self._http_json_raw(method, path, timeout_s=timeout_s, body=body)
        return value

    def _http_json_raw(
        self, method: str, path: str, *, timeout_s: float, body: dict[str, Any] | None = None
    ) -> tuple[bytes, dict[str, Any]]:
        assert self.inference_url is not None
        payload = None if body is None else json.dumps(body).encode("utf-8")
        request = Request(
            urljoin(self.inference_url + "/", path.lstrip("/")),
            data=payload,
            method=method,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        try:
            with urlopen(request, timeout=timeout_s) as response:
                raw = response.read()
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"Windows order server {method} {path} failed: {error}") from error
        try:
            value = json.loads(raw)
        except (UnicodeDecodeError, ValueError) as error:
            raise RuntimeError(f"Windows order server {method} {path} returned invalid JSON") from error
        if not isinstance(value, dict):
            raise RuntimeError(f"Windows order server {method} {path} returned a non-object JSON value")
        return raw, value

    def _bind_order(self) -> None:
        with self._inference_gate:
            self._check_stopped()
            self._bind_order_locked()

    def _bind_order_locked(self) -> None:
        state = self._http_json("GET", "/api/state", timeout_s=3.0)
        active = [order for order in state.get("orders", [])
                  if order.get("state") in ("RUNNING", "QUEUED")]
        if active:
            if len(active) != 1 or active[0].get("table_id") != "table_1":
                raise RuntimeError("active order is not the registered table_1 mission")
            order = active[0]
        else:
            # Only called by an explicit start request, never during readiness.
            # One submission; an ambiguous response is never retried automatically.
            result = self._http_json("POST", "/api/orders", timeout_s=5.0, body={
                "table_id": "table_1", "drink": "COLD_WATER",
                "client_ref": self._state["mission_id"],
            })
            order = result.get("order") or {}
        order_id = order.get("order_id")
        if not isinstance(order_id, str) or not order_id:
            raise RuntimeError("order server did not return an order_id")
        with self._lock:
            self._state["order_id"] = order_id
            self._save_state()

    def _drive_ready(self, state: dict[str, Any], destination: str) -> bool:
        if (state.get("emergency_stop") or {}).get("active"):
            raise RuntimeError("emergency stop is active on the order server")
        backend = state.get("execution_backend") or {}
        drive = backend.get("drive") or {}
        order_id = self._state.get("order_id")
        order = next((o for o in state.get("orders", [])
                      if o.get("order_id") == order_id), None)
        if order is None:
            raise RuntimeError("bound order disappeared from order server")
        if order.get("state") in ("FAILED", "CANCELED", "CANCELLED"):
            raise RuntimeError(f"order {order_id} ended: {order.get('failure') or order['state']}")
        if ("running_policy" not in backend or backend["running_policy"] is not None
                or drive.get("arms_ready") is not True or drive.get("next") != destination
                or backend.get("arms") not in ("stowed", "carrying_cup")):
            return False
        if destination == "dock":
            return order.get("state") == "SUCCEEDED" and backend.get("arms") == "stowed"
        waiting = backend.get("waiting_alignment") or {}
        return (order.get("state") == "RUNNING"
                and waiting.get("station") == destination
                and waiting.get("order_id") == order_id)

    def _wait_drive(self, destination: str) -> None:
        deadline = time.monotonic() + self.inference_timeout_s
        last_error = None
        while time.monotonic() < deadline:
            self._check_stopped()
            try:
                state = self._http_json(
                    "GET", "/api/state",
                    timeout_s=min(3.0, max(0.001, deadline - time.monotonic())))
            except RuntimeError as error:
                # No drive helper runs in this stage. Keep the same order while
                # waiting for communication to recover; never replay a POST.
                last_error = str(error)
                self._stop_event.wait(0.25)
                continue
            self._check_stopped()
            if self._drive_ready(state, destination):
                return
            last_error = None
            self._stop_event.wait(0.25)
        self._check_stopped()
        detail = f"; last read error: {last_error}" if last_error else ""
        raise RuntimeError(f"order server did not release drive to {destination}{detail}")

    def _confirm_alignment(self, station: str) -> None:
        # Validate the same order immediately before authorizing arm motion.
        # The base is parked: a transient read may recover in this same stage.
        self._wait_drive(station)
        with self._inference_gate:
            self._check_stopped()
            result = self._http_json("POST", "/api/align/confirm", timeout_s=5.0,
                                     body={"station": station})
        if result.get("confirmed") is not True or result.get("station") != station:
            raise RuntimeError(f"alignment confirmation rejected: {result}")

    def _check_stopped(self) -> None:
        if self._stop_event.is_set():
            raise _Stopped()

    def _set_phase(self, phase: str) -> None:
        with self._lock:
            self._check_stopped()
            self._state.update(state="running", phase=phase, error=None)
            request_id = self._state.get("request_id")
            if request_id:
                self._state["requests"][request_id].update(state="running", phase=phase)
            self._save_state()
            if self._log_path is not None:
                with self._log_path.open("a", encoding="utf-8") as output:
                    output.write(json.dumps({
                        "event": "water_service_phase",
                        "phase": phase,
                        "unix_s": time.time(),
                    }, ensure_ascii=False) + "\n")

    def _finish(self, state: str, phase: str, error: str | None) -> None:
        with self._lock:
            if self._stop_event.is_set() and state != "stopped":
                state, phase, error = "stopped", "stopped", "stop requested"
            self._state.update(
                state=state,
                phase=phase,
                error=error,
                finished_unix_s=time.time(),
            )
            self._record_request_terminal()
            self._save_state()

    def _record_request_terminal(self) -> None:
        request_id = self._state.get("request_id")
        if not request_id:
            return
        self._state.setdefault("requests", {})[request_id] = {
            "mission_id": self._state.get("mission_id"),
            "state": self._state.get("state"),
            "phase": self._state.get("phase"),
            "mode": self._state.get("mode"),
            "error": self._state.get("error"),
        }

    def _load_state(self) -> dict[str, Any]:
        if self._state_path.is_file():
            try:
                value = json.loads(self._state_path.read_text(encoding="utf-8"))
                if isinstance(value, dict):
                    value.setdefault("requests", {})
                    return value
            except (OSError, ValueError):
                pass
        return {
            "state": "idle",
            "phase": "idle",
            "mode": "inference" if self.run_inference else "navigation_only",
            "mission_id": None,
            "request_id": None,
            "inference_url": self.inference_url,
            "configured": bool(self.inference_url),
            "error": None,
            "requests": {},
        }

    def _save_state(self) -> None:
        temporary = self._state_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self._state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary.replace(self._state_path)
