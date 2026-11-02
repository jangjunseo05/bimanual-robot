#!/usr/bin/env python3
"""CPU-only order queue and mission controller for the water-service robot.

This module does not connect to motors, cameras, ROS 2, or Isaac Sim.  It emits
backend commands that can be consumed by a dry-run, Nav2, or simulator adapter.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from typing import Callable

from simulation.restaurant_layout import load_layout, plan_route


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config/simulation/service_mission_control.json"
DEFAULT_ORDERS = ROOT / "config/simulation/service_orders_demo.json"
DEFAULT_LAYOUT = ROOT / "config/simulation/restaurant_layout.json"

# PENDING(손님 주문, 운영자 확인 전) -> ACCEPTED(수락, Start 대기) -> QUEUED(Start로 출발 허용) -> RUNNING
ORDER_STATES = {"PENDING", "ACCEPTED", "QUEUED", "RUNNING", "SUCCEEDED", "FAILED", "CANCELED", "REJECTED"}
TERMINAL_ORDER_STATES = {"SUCCEEDED", "FAILED", "CANCELED", "REJECTED"}
# 로봇이 반드시 다룰 수 있어야 하는 기본 음료. config 의 supported_drinks 에 다른 병 음료를 더할 수 있다.
DRINKS = {"COLD_WATER", "HOT_WATER"}
ORDER_ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,64}$")
SERVICE_PHASES = (
    "NAVIGATE_KITCHEN",
    "ALIGN_KITCHEN",
    "GRASP_CUP",
    "GRASP_BOTTLE",
    "POUR",
    "RETURN_BOTTLE",
    "PLACE_DECK",
    "NAVIGATE_TABLE",
    "ALIGN_TABLE",
    "REGRASP_CUP",
    "SERVE",
)
NAVIGATION_PHASES = {"NAVIGATE_KITCHEN", "NAVIGATE_TABLE", "NAVIGATE_DOCK"}
SYSTEM_PHASES = {"IDLE_AT_DOCK", "NAVIGATE_DOCK", "CHARGING", "TERMINAL_HOLD"}
SKILL_BY_PHASE = {
    "ALIGN_KITCHEN": "align_at_kitchen",
    "GRASP_CUP": "grasp_cup",
    "GRASP_BOTTLE": "grasp_bottle",
    "POUR": "pour_water",
    "RETURN_BOTTLE": "return_bottle",
    "PLACE_DECK": "place_cup_on_deck",
    "ALIGN_TABLE": "align_at_guest_table",
    "REGRASP_CUP": "regrasp_cup",
    "SERVE": "place_cup_on_guest_table",
}


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _parse_timestamp(value: object) -> datetime:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("created_at must be a timezone-aware ISO-8601 string")
    normalized = value.strip().replace("Z", "+00:00")
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise ValueError("created_at must be a timezone-aware ISO-8601 string") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("created_at must include a timezone")
    return parsed.astimezone(timezone.utc)


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def route_length(path: list[list[float]]) -> float:
    return sum(math.dist(first, second) for first, second in zip(path, path[1:]))


def load_control_config(path: Path = DEFAULT_CONFIG) -> dict:
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if config.get("schema") != "service_mission_control_v1":
        raise ValueError("service mission control schema mismatch")
    drinks = config.get("supported_drinks")
    if not isinstance(drinks, dict) or not DRINKS <= set(drinks):
        raise ValueError("COLD_WATER and HOT_WATER drink definitions are required")
    for name, drink in drinks.items():
        if not isinstance(drink, dict) or drink.get("station") != "kitchen":
            raise ValueError(f"{name} must use the kitchen station")
        if not all(isinstance(drink.get(key), str) and drink[key] for key in ("label_ko", "bottle_id")):
            raise ValueError(f"{name} label and bottle_id are required")
    battery = config.get("battery", {})
    for key in ("reserve_percent", "route_percent_per_m", "charge_target_percent"):
        battery[key] = _finite_number(battery.get(key), f"battery.{key}")
    if not 0 <= battery["reserve_percent"] < battery["charge_target_percent"] <= 100:
        raise ValueError("battery reserve/target range is invalid")
    if battery["route_percent_per_m"] <= 0:
        raise ValueError("battery.route_percent_per_m must be positive")
    phase_energy = config.get("phase_energy_percent")
    if not isinstance(phase_energy, dict) or set(phase_energy) != set(SKILL_BY_PHASE):
        raise ValueError("phase_energy_percent must define every manipulation phase")
    for phase, value in phase_energy.items():
        phase_energy[phase] = _finite_number(value, f"phase_energy_percent.{phase}")
        if phase_energy[phase] < 0:
            raise ValueError("phase energy cannot be negative")
    retry = config.get("retry", {})
    attempts = retry.get("max_attempts_per_phase")
    if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 1:
        raise ValueError("retry.max_attempts_per_phase must be a positive integer")
    retryable = retry.get("retryable_phases")
    if not isinstance(retryable, list) or len(retryable) != len(set(retryable)):
        raise ValueError("retry.retryable_phases must be a unique list")
    if not set(retryable).issubset(SKILL_BY_PHASE):
        raise ValueError("retryable phase is not a manipulation phase")
    limit = config.get("event_history_limit")
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 10:
        raise ValueError("event_history_limit must be an integer of at least 10")
    return config


class OrderStateError(RuntimeError):
    """The operator action does not fit the current order or mission state."""


@dataclass
class ServiceOrder:
    order_id: str
    table_id: str
    drink: str
    created_at: str
    priority: int
    sequence: int
    state: str = "QUEUED"
    mission_id: str | None = None
    failure: str | None = None
    completed_at: str | None = None


class MissionController:
    """Deterministic high-level controller with one active mission."""

    def __init__(
        self,
        *,
        config: dict | None = None,
        layout: dict | None = None,
        battery_percent: float = 100.0,
        require_dock_return: bool = False,
        require_operator: bool = False,
        clock: Callable[[], datetime] = _utc_now,
    ) -> None:
        if not isinstance(require_dock_return, bool):
            raise ValueError("require_dock_return must be boolean")
        if not isinstance(require_operator, bool):
            raise ValueError("require_operator must be boolean")
        self.config = config if config is not None else load_control_config()
        self.layout = layout if layout is not None else load_layout()
        self.clock = clock
        self.require_dock_return = require_dock_return
        self.require_operator = require_operator
        self.table_ids = {table["id"] for table in self.layout["tables"]}
        self.orders: dict[str, ServiceOrder] = {}
        self.sequence = 0
        self.event_sequence = 0
        self.active_order_id: str | None = None
        self.phase = "IDLE_AT_DOCK"
        self.phase_attempt = 0
        self.current_location = "dock"
        self.battery_percent = _finite_number(battery_percent, "battery_percent")
        if not 0 <= self.battery_percent <= 100:
            raise ValueError("battery_percent must be between 0 and 100")
        self.events: list[dict] = []
        self._record("CONTROLLER_STARTED")

    @classmethod
    def from_snapshot(
        cls,
        snapshot: dict,
        *,
        config: dict | None = None,
        layout: dict | None = None,
        require_operator: bool = False,
        clock: Callable[[], datetime] = _utc_now,
    ) -> "MissionController":
        """Restore a validated controller snapshot after a server restart."""
        if not isinstance(snapshot, dict) or snapshot.get("schema") != "service_mission_snapshot_v1":
            raise ValueError("service mission snapshot schema mismatch")
        battery_percent = _finite_number(snapshot.get("battery_percent"), "battery_percent")
        controller = cls(
            config=config,
            layout=layout,
            battery_percent=battery_percent,
            require_operator=require_operator,
            clock=clock,
        )
        raw_orders = snapshot.get("orders")
        if not isinstance(raw_orders, list):
            raise ValueError("snapshot orders must be a list")
        restored: dict[str, ServiceOrder] = {}
        sequences: set[int] = set()
        for raw in raw_orders:
            if not isinstance(raw, dict):
                raise ValueError("snapshot order must be an object")
            try:
                order = ServiceOrder(**raw)
            except TypeError as exc:
                raise ValueError("snapshot order fields are invalid") from exc
            if not ORDER_ID_PATTERN.fullmatch(order.order_id) or order.order_id in restored:
                raise ValueError("snapshot order_id is invalid or duplicated")
            if order.table_id not in controller.table_ids or order.drink not in controller.config["supported_drinks"]:
                raise ValueError("snapshot order destination or drink is invalid")
            if order.state not in ORDER_STATES:
                raise ValueError("snapshot order state is invalid")
            if isinstance(order.priority, bool) or not isinstance(order.priority, int) or not 0 <= order.priority <= 9:
                raise ValueError("snapshot order priority is invalid")
            if isinstance(order.sequence, bool) or not isinstance(order.sequence, int) or order.sequence < 1:
                raise ValueError("snapshot order sequence is invalid")
            if order.sequence in sequences:
                raise ValueError("snapshot order sequence is duplicated")
            _parse_timestamp(order.created_at)
            if order.completed_at is not None:
                _parse_timestamp(order.completed_at)
            restored[order.order_id] = order
            sequences.add(order.sequence)

        phase = snapshot.get("phase")
        valid_phases = set(SERVICE_PHASES) | SYSTEM_PHASES
        if phase not in valid_phases:
            raise ValueError("snapshot phase is invalid")
        current_location = snapshot.get("current_location")
        if current_location not in controller.layout["waypoints"]:
            raise ValueError("snapshot current_location is invalid")
        active_order_id = snapshot.get("active_order_id")
        running_order_ids = [order.order_id for order in restored.values() if order.state == "RUNNING"]
        if active_order_id is not None:
            active = restored.get(active_order_id)
            if (active is None or active.state != "RUNNING" or phase not in SERVICE_PHASES
                    or running_order_ids != [active_order_id]):
                raise ValueError("snapshot active order is inconsistent")
        elif phase in SERVICE_PHASES or running_order_ids:
            raise ValueError("snapshot service phase requires an active order")
        phase_attempt = snapshot.get("phase_attempt")
        if isinstance(phase_attempt, bool) or not isinstance(phase_attempt, int) or phase_attempt < 0:
            raise ValueError("snapshot phase_attempt is invalid")
        raw_events = snapshot.get("events", [])
        if not isinstance(raw_events, list) or not all(isinstance(event, dict) for event in raw_events):
            raise ValueError("snapshot events must be a list of objects")
        event_sequences = [event.get("sequence_number") for event in raw_events]
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 1 for value in event_sequences):
            raise ValueError("snapshot event sequence is invalid")
        if event_sequences != sorted(set(event_sequences)):
            raise ValueError("snapshot event sequence is duplicated or unordered")

        controller.orders = restored
        controller.sequence = max(sequences, default=0)
        controller.event_sequence = max(event_sequences, default=0)
        controller.active_order_id = active_order_id
        controller.phase = phase
        controller.phase_attempt = phase_attempt
        controller.current_location = current_location
        controller.events = raw_events[-controller.config["event_history_limit"] :]
        controller._record("CONTROLLER_RESTORED")
        return controller

    @property
    def active_order(self) -> ServiceOrder | None:
        return self.orders.get(self.active_order_id) if self.active_order_id else None

    def _record(self, kind: str, **details: object) -> None:
        self.event_sequence += 1
        event = {
            "sequence_number": self.event_sequence,
            "timestamp": _iso(self.clock()),
            "kind": kind,
            "phase": self.phase,
            "active_order_id": self.active_order_id,
            "battery_percent": round(self.battery_percent, 6),
            **details,
        }
        self.events.append(event)
        self.events = self.events[-self.config["event_history_limit"] :]

    def _validate_new_order(self, payload: dict) -> ServiceOrder:
        if not isinstance(payload, dict):
            raise ValueError("order payload must be an object")
        order_id = payload.get("order_id")
        if not isinstance(order_id, str) or not ORDER_ID_PATTERN.fullmatch(order_id):
            raise ValueError("order_id must be 1-64 safe identifier characters")
        table_id = payload.get("table_id")
        if table_id not in self.table_ids:
            raise ValueError("unknown table_id")
        drink = payload.get("drink")
        if drink not in self.config["supported_drinks"]:
            raise ValueError("unsupported drink")
        priority = payload.get("priority", 0)
        if isinstance(priority, bool) or not isinstance(priority, int) or not 0 <= priority <= 9:
            raise ValueError("priority must be an integer from 0 to 9")
        created = _parse_timestamp(payload["created_at"]) if "created_at" in payload else self.clock()
        self.sequence += 1
        return ServiceOrder(
            order_id=order_id,
            table_id=table_id,
            drink=drink,
            created_at=_iso(created),
            priority=priority,
            sequence=self.sequence,
        )

    def submit(self, payload: dict) -> tuple[ServiceOrder, bool]:
        order_id = payload.get("order_id") if isinstance(payload, dict) else None
        if isinstance(order_id, str) and order_id in self.orders:
            existing = self.orders[order_id]
            for key in ("table_id", "drink", "priority"):
                if key in payload and payload[key] != getattr(existing, key):
                    raise ValueError(f"duplicate order_id conflicts on {key}")
            if "created_at" in payload and _iso(_parse_timestamp(payload["created_at"])) != existing.created_at:
                raise ValueError("duplicate order_id conflicts on created_at")
            self._record("ORDER_DUPLICATE_IGNORED", order_id=order_id)
            return existing, False
        order = self._validate_new_order(payload)
        self.orders[order.order_id] = order
        if self.require_operator:
            # 로봇은 운영자의 수락과 Start 뒤에만 움직인다. 운영자가 직접 넣은 주문은 수락된 채로 들어온다.
            order.state = "ACCEPTED" if payload.get("accepted") is True else "PENDING"
            self._record("ORDER_RECEIVED", order_id=order.order_id, table_id=order.table_id, drink=order.drink)
            return order, True
        self._record("ORDER_QUEUED", order_id=order.order_id, table_id=order.table_id, drink=order.drink)
        if self.phase == "IDLE_AT_DOCK" and self.active_order is None:
            self._dispatch_or_charge()
        elif self.phase == "NAVIGATE_DOCK" and self.active_order is None and self._can_dispatch(order):
            self._start_order(order)
        return order, True

    def cancel(self, order_id: str) -> ServiceOrder:
        if order_id not in self.orders:
            raise KeyError(order_id)
        order = self.orders[order_id]
        if order.state in TERMINAL_ORDER_STATES:
            return order
        order.state = "CANCELED"
        order.completed_at = _iso(self.clock())
        self._record("ORDER_CANCELED", order_id=order_id)
        if self.active_order_id == order_id:
            self.active_order_id = None
            if self.require_dock_return:
                self.phase = "TERMINAL_HOLD"
                self.phase_attempt = 0
                self._record("ROUNDTRIP_TERMINAL_HOLD", outcome="CANCELED")
            else:
                self._decide_after_terminal_order()
        return order

    def _operator_order(self, order_id: str, allowed: set[str]) -> ServiceOrder:
        if order_id not in self.orders:
            raise KeyError(order_id)
        order = self.orders[order_id]
        if order.state not in allowed:
            raise OrderStateError(f"order {order_id} is {order.state}")
        return order

    def accept(self, order_id: str) -> ServiceOrder:
        order = self._operator_order(order_id, {"PENDING", "ACCEPTED"})
        if order.state == "PENDING":
            order.state = "ACCEPTED"
            self._record("ORDER_ACCEPTED", order_id=order_id)
        return order

    def reject(self, order_id: str) -> ServiceOrder:
        order = self._operator_order(order_id, {"PENDING", "REJECTED"})
        if order.state == "PENDING":
            order.state = "REJECTED"
            order.completed_at = _iso(self.clock())
            self._record("ORDER_REJECTED", order_id=order_id)
        return order

    def accepted_orders(self) -> list[ServiceOrder]:
        return sorted(
            (order for order in self.orders.values() if order.state == "ACCEPTED"),
            key=lambda order: (-order.priority, order.created_at, order.sequence),
        )

    def start_next(self) -> ServiceOrder:
        """Operator Start: release the oldest accepted order to the dispatcher."""
        if self.active_order is not None:
            raise OrderStateError("a mission is already running")
        accepted = self.accepted_orders()
        if not accepted:
            raise OrderStateError("no accepted order")
        order = accepted[0]
        order.state = "QUEUED"
        self._record("ORDER_RELEASED", order_id=order.order_id)
        if self.phase == "IDLE_AT_DOCK":
            self._dispatch_or_charge()
        elif self.phase == "NAVIGATE_DOCK" and self._can_dispatch(order):
            self._start_order(order)
        return order

    def emergency_stop(self) -> ServiceOrder | None:
        """Operator STOP: end the active mission; released orders go back behind the Start button."""
        for order in self._queued():
            order.state = "ACCEPTED"
        self._record("EMERGENCY_STOP")
        order = self.active_order
        if order is None:
            return None
        self._fail_order(order, self.phase, "EMERGENCY_STOP")
        self._end_after_failure()
        return order

    def record_event(self, kind: str) -> None:
        self._record(kind)

    def _fail_order(self, order: ServiceOrder, phase: str, reason: str) -> None:
        order.state = "FAILED"
        order.failure = f"{phase}:{reason}"
        order.completed_at = _iso(self.clock())
        self._record("MISSION_FAILED", order_id=order.order_id, failed_phase=phase, failure=reason)
        self.active_order_id = None

    def _end_after_failure(self) -> None:
        if self.require_dock_return:
            self.phase = "TERMINAL_HOLD"
            self.phase_attempt = 0
            self._record("ROUNDTRIP_TERMINAL_HOLD", outcome="FAILED")
        else:
            self._decide_after_terminal_order()

    def _queued(self) -> list[ServiceOrder]:
        return sorted(
            (order for order in self.orders.values() if order.state == "QUEUED"),
            key=lambda order: (-order.priority, order.created_at, order.sequence),
        )

    def _path(self, start: str, goal: str) -> list[list[float]]:
        if start == goal:
            pose = self.layout["waypoints"][goal]
            return [[float(pose[0]), float(pose[1])]]
        return plan_route(self.layout, start, goal)

    def estimate_order_cost(self, order: ServiceOrder, start: str | None = None) -> float:
        start_name = start or self.current_location
        route_m = route_length(self._path(start_name, "kitchen"))
        route_m += route_length(self._path("kitchen", order.table_id))
        route_m += route_length(self._path(order.table_id, "dock"))
        manipulation = sum(self.config["phase_energy_percent"].values())
        return route_m * self.config["battery"]["route_percent_per_m"] + manipulation

    def _can_dispatch(self, order: ServiceOrder) -> bool:
        required = self.estimate_order_cost(order)
        return self.battery_percent - required >= self.config["battery"]["reserve_percent"]

    def _start_order(self, order: ServiceOrder) -> None:
        if self.active_order is not None:
            raise RuntimeError("only one active mission is allowed")
        order.state = "RUNNING"
        order.mission_id = f"MIS-{order.order_id}-{order.sequence:04d}"
        self.active_order_id = order.order_id
        self.phase = "NAVIGATE_KITCHEN"
        self.phase_attempt = 1
        self._record("MISSION_STARTED", order_id=order.order_id, mission_id=order.mission_id)

    def _dispatch_or_charge(self) -> None:
        queued = self._queued()
        if not queued:
            if self.current_location == "dock":
                self.phase = "IDLE_AT_DOCK"
            else:
                self.phase = "NAVIGATE_DOCK"
            self.phase_attempt = 0
            return
        candidate = queued[0]
        if self._can_dispatch(candidate):
            self._start_order(candidate)
        elif self.current_location == "dock":
            self.phase = "CHARGING"
            self.phase_attempt = 1
            self._record("CHARGE_REQUIRED", next_order_id=candidate.order_id)
        else:
            self.phase = "NAVIGATE_DOCK"
            self.phase_attempt = 1
            self._record("RETURN_REQUIRED_FOR_CHARGE", next_order_id=candidate.order_id)

    def _decide_after_terminal_order(self) -> None:
        self.phase_attempt = 0
        self._dispatch_or_charge()
        if self.phase == "NAVIGATE_DOCK":
            self.phase_attempt = 1
            self._record("RETURN_TO_DOCK_STARTED")

    def current_command(self) -> dict:
        order = self.active_order
        if self.phase == "IDLE_AT_DOCK":
            return {"kind": "hold", "name": "idle_at_dock", "hardware_accessed": False}
        if self.phase == "CHARGING":
            return {
                "kind": "charge",
                "name": "charge_robot",
                "target_percent": self.config["battery"]["charge_target_percent"],
                "hardware_accessed": False,
            }
        if self.phase == "TERMINAL_HOLD":
            return {
                "kind": "hold",
                "name": "roundtrip_terminal_hold",
                "hardware_accessed": False,
            }
        if self.phase in NAVIGATION_PHASES:
            if self.phase == "NAVIGATE_KITCHEN":
                destination = "kitchen"
            elif self.phase == "NAVIGATE_TABLE":
                if order is None:
                    raise RuntimeError("table navigation requires an active order")
                destination = order.table_id
            else:
                destination = "dock"
            path = self._path(self.current_location, destination)
            return {
                "kind": "navigate",
                "name": "navigate_to_destination",
                "start": self.current_location,
                "destination": destination,
                "path_xy_m": path,
                "route_length_m": route_length(path),
                "hardware_accessed": False,
            }
        if self.phase in SKILL_BY_PHASE:
            if order is None:
                raise RuntimeError("manipulation phase requires an active order")
            drink = self.config["supported_drinks"][order.drink]
            return {
                "kind": "manipulate",
                "name": SKILL_BY_PHASE[self.phase],
                "phase": self.phase,
                "policy_mode": "PLANNED_ALL",
                "order_id": order.order_id,
                "table_id": order.table_id,
                "drink": order.drink,
                "bottle_id": drink["bottle_id"],
                "hardware_accessed": False,
            }
        raise RuntimeError(f"unsupported phase: {self.phase}")

    def _consume_for_command(self, command: dict) -> None:
        if command["kind"] == "navigate":
            amount = command["route_length_m"] * self.config["battery"]["route_percent_per_m"]
        elif command["kind"] == "manipulate":
            amount = self.config["phase_energy_percent"][self.phase]
        else:
            amount = 0.0
        self.battery_percent = max(0.0, self.battery_percent - amount)

    def advance(self, *, success: bool = True, failure: str | None = None) -> dict:
        if not isinstance(success, bool):
            raise ValueError("success must be boolean")
        command = self.current_command()
        phase_before = self.phase
        if self.phase == "TERMINAL_HOLD":
            return command
        if self.phase == "IDLE_AT_DOCK":
            self._dispatch_or_charge()
            return command
        if not success:
            reason = failure or "backend_failed"
            retryable = self.phase in self.config["retry"]["retryable_phases"]
            if retryable and self.phase_attempt < self.config["retry"]["max_attempts_per_phase"]:
                self.phase_attempt += 1
                self._record("PHASE_RETRY", failed_phase=phase_before, failure=reason,
                             attempt=self.phase_attempt)
                return command
            order = self.active_order
            if order is not None:
                self._fail_order(order, phase_before, reason)
            else:
                self._record("SYSTEM_PHASE_FAILED", failed_phase=phase_before, failure=reason)
            self._end_after_failure()
            return command

        self._consume_for_command(command)
        if self.phase == "CHARGING":
            self.battery_percent = self.config["battery"]["charge_target_percent"]
            self._record("CHARGE_COMPLETED")
            self._dispatch_or_charge()
            return command
        if self.phase == "NAVIGATE_DOCK":
            self.current_location = "dock"
            order = self.active_order
            if self.require_dock_return and order is not None:
                self._record("PHASE_SUCCEEDED", completed_phase=phase_before, order_id=order.order_id)
                order.state = "SUCCEEDED"
                order.completed_at = _iso(self.clock())
                self._record("MISSION_SUCCEEDED", order_id=order.order_id, mission_id=order.mission_id)
                self.active_order_id = None
                self.phase = "IDLE_AT_DOCK"
                self.phase_attempt = 0
                return command
            self.phase = "CHARGING"
            self.phase_attempt = 1
            self._record("DOCK_ARRIVED")
            return command
        order = self.active_order
        if order is None:
            raise RuntimeError("service phase completed without an active order")
        index = SERVICE_PHASES.index(self.phase)
        if self.phase == "NAVIGATE_KITCHEN":
            self.current_location = "kitchen"
        elif self.phase == "NAVIGATE_TABLE":
            self.current_location = order.table_id
        self._record("PHASE_SUCCEEDED", completed_phase=phase_before, order_id=order.order_id)
        if index + 1 < len(SERVICE_PHASES):
            self.phase = SERVICE_PHASES[index + 1]
            self.phase_attempt = 1
        else:
            if self.require_dock_return:
                self.phase = "NAVIGATE_DOCK"
                self.phase_attempt = 1
                self._record("RETURN_TO_DOCK_STARTED", order_id=order.order_id)
            else:
                order.state = "SUCCEEDED"
                order.completed_at = _iso(self.clock())
                self._record("MISSION_SUCCEEDED", order_id=order.order_id, mission_id=order.mission_id)
                self.active_order_id = None
                self._decide_after_terminal_order()
        return command

    def snapshot(self) -> dict:
        orders = sorted(self.orders.values(), key=lambda item: item.sequence)
        queued = self._queued()
        command = self.current_command()
        return {
            "schema": "service_mission_snapshot_v1",
            "timestamp": _iso(self.clock()),
            "mission_state": (
                "RUNNING" if self.active_order is not None else
                "TERMINAL" if self.phase == "TERMINAL_HOLD" else
                "CHARGING" if self.phase == "CHARGING" else
                "IDLE" if self.phase == "IDLE_AT_DOCK" else "RUNNING"
            ),
            "phase": self.phase,
            "phase_attempt": self.phase_attempt,
            "active_order_id": self.active_order_id,
            "current_location": self.current_location,
            "battery_percent": round(self.battery_percent, 6),
            "queue": [order.order_id for order in queued],
            "orders": [asdict(order) for order in orders],
            "command": command,
            "events": list(self.events),
            "hardware_accessed": False,
            "simulator_accessed": False,
        }


class DryRunRuntime:
    """Executes emitted commands as immediate success/failure observations."""

    def __init__(self, controller: MissionController) -> None:
        self.controller = controller
        self.commands: list[dict] = []

    def step(self, *, success: bool = True, failure: str | None = None) -> dict:
        command = self.controller.current_command()
        self.commands.append(command)
        self.controller.advance(success=success, failure=failure)
        return command

    def run_until_idle(self, max_steps: int = 200) -> dict:
        for _ in range(max_steps):
            if (self.controller.phase == "IDLE_AT_DOCK"
                    and self.controller.active_order is None
                    and not self.controller._queued()):
                return self.controller.snapshot()
            self.step()
        raise RuntimeError("dry-run exceeded max_steps")


def run_batch(orders_path: Path, output_dir: Path, battery_percent: float) -> dict:
    batch = json.loads(Path(orders_path).read_text(encoding="utf-8"))
    if batch.get("schema") != "service_order_batch_v1" or not isinstance(batch.get("orders"), list):
        raise ValueError("service order batch schema mismatch")
    controller = MissionController(battery_percent=battery_percent)
    runtime = DryRunRuntime(controller)
    for order in batch["orders"]:
        controller.submit(order)
    snapshot = runtime.run_until_idle()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "snapshot.json").write_text(
        json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "commands.json").write_text(
        json.dumps(runtime.commands, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "events.jsonl").write_text(
        "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in controller.events),
        encoding="utf-8",
    )
    return snapshot


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orders", type=Path, default=DEFAULT_ORDERS)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/bimanual-service-mission"))
    parser.add_argument("--battery-percent", type=float, default=100.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    snapshot = run_batch(args.orders, args.output_dir, args.battery_percent)
    print(json.dumps({
        "phase": snapshot["phase"],
        "battery_percent": snapshot["battery_percent"],
        "orders": {order["order_id"]: order["state"] for order in snapshot["orders"]},
        "output_dir": str(args.output_dir),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
