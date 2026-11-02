#!/usr/bin/env python3
"""SQLite persistence for service orders, controller snapshots, events, and commands."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3


SCHEMA_VERSION = 1
DATABASE_NAME = "service_mission_control.sqlite3"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class ServiceOrderStore:
    """Durable append-friendly store owned by one service-order server process."""

    def __init__(self, state_dir: Path) -> None:
        self.state_dir = Path(state_dir).expanduser().resolve()
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.database_path = self.state_dir / DATABASE_NAME
        self.connection = sqlite3.connect(self.database_path, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        try:
            self._create_schema()
        except Exception:
            self.connection.close()
            raise

    def _create_schema(self) -> None:
        self.connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS controller_snapshot (
                singleton_id INTEGER PRIMARY KEY CHECK (singleton_id = 1),
                payload_json TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS orders (
                order_id TEXT PRIMARY KEY,
                table_id TEXT NOT NULL,
                drink TEXT NOT NULL,
                created_at TEXT NOT NULL,
                priority INTEGER NOT NULL,
                sequence INTEGER NOT NULL UNIQUE,
                state TEXT NOT NULL,
                mission_id TEXT,
                failure TEXT,
                completed_at TEXT,
                updated_at TEXT NOT NULL,
                payload_json TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_orders_state_sequence
                ON orders(state, sequence);
            CREATE TABLE IF NOT EXISTS events (
                sequence_number INTEGER PRIMARY KEY,
                timestamp TEXT NOT NULL,
                kind TEXT NOT NULL,
                phase TEXT NOT NULL,
                active_order_id TEXT,
                battery_percent REAL NOT NULL,
                payload_json TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS commands (
                command_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                recorded_at TEXT NOT NULL,
                order_id TEXT,
                phase TEXT NOT NULL,
                kind TEXT NOT NULL,
                name TEXT NOT NULL,
                payload_json TEXT NOT NULL
            );
            """
        )
        existing = self.connection.execute(
            "SELECT value FROM metadata WHERE key='schema_version'"
        ).fetchone()
        if existing is not None and existing["value"] != str(SCHEMA_VERSION):
            raise RuntimeError(
                f"unsupported service-order database schema: {existing['value']}"
            )
        self.connection.execute(
            "INSERT OR IGNORE INTO metadata(key, value) VALUES('schema_version', ?)",
            (str(SCHEMA_VERSION),),
        )
        self.connection.commit()

    def load_snapshot(self) -> dict | None:
        row = self.connection.execute(
            "SELECT payload_json FROM controller_snapshot WHERE singleton_id=1"
        ).fetchone()
        return json.loads(row["payload_json"]) if row is not None else None

    def persist(self, snapshot: dict, command: dict | None = None) -> None:
        updated_at = _utc_iso()
        with self.connection:
            self.connection.execute(
                """
                INSERT INTO controller_snapshot(singleton_id, payload_json, updated_at)
                VALUES(1, ?, ?)
                ON CONFLICT(singleton_id) DO UPDATE SET
                    payload_json=excluded.payload_json,
                    updated_at=excluded.updated_at
                """,
                (json.dumps(snapshot, ensure_ascii=False), updated_at),
            )
            for order in snapshot["orders"]:
                self.connection.execute(
                    """
                    INSERT INTO orders(
                        order_id, table_id, drink, created_at, priority, sequence,
                        state, mission_id, failure, completed_at, updated_at, payload_json
                    ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(order_id) DO UPDATE SET
                        table_id=excluded.table_id,
                        drink=excluded.drink,
                        created_at=excluded.created_at,
                        priority=excluded.priority,
                        sequence=excluded.sequence,
                        state=excluded.state,
                        mission_id=excluded.mission_id,
                        failure=excluded.failure,
                        completed_at=excluded.completed_at,
                        updated_at=excluded.updated_at,
                        payload_json=excluded.payload_json
                    """,
                    (
                        order["order_id"], order["table_id"], order["drink"],
                        order["created_at"], order["priority"], order["sequence"],
                        order["state"], order["mission_id"], order["failure"],
                        order["completed_at"], updated_at,
                        json.dumps(order, ensure_ascii=False),
                    ),
                )
            for event in snapshot["events"]:
                self.connection.execute(
                    """
                    INSERT OR IGNORE INTO events(
                        sequence_number, timestamp, kind, phase, active_order_id,
                        battery_percent, payload_json
                    ) VALUES(?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event["sequence_number"], event["timestamp"], event["kind"],
                        event["phase"], event.get("active_order_id"),
                        event["battery_percent"], json.dumps(event, ensure_ascii=False),
                    ),
                )
            if command is not None:
                self.connection.execute(
                    """
                    INSERT INTO commands(recorded_at, order_id, phase, kind, name, payload_json)
                    VALUES(?, ?, ?, ?, ?, ?)
                    """,
                    (
                        updated_at, command.get("order_id", snapshot.get("active_order_id")),
                        command.get("phase", snapshot["phase"]), command["kind"],
                        command["name"], json.dumps(command, ensure_ascii=False),
                    ),
                )

    def order_history(self, limit: int = 100) -> list[dict]:
        rows = self.connection.execute(
            "SELECT payload_json FROM orders ORDER BY sequence DESC LIMIT ?", (limit,)
        ).fetchall()
        return [json.loads(row["payload_json"]) for row in rows]

    def event_history(self, limit: int = 200) -> list[dict]:
        rows = self.connection.execute(
            "SELECT payload_json FROM events ORDER BY sequence_number DESC LIMIT ?", (limit,)
        ).fetchall()
        return [json.loads(row["payload_json"]) for row in reversed(rows)]

    def stats(self) -> dict:
        counts = {
            row["state"]: row["count"]
            for row in self.connection.execute(
                "SELECT state, COUNT(*) AS count FROM orders GROUP BY state"
            )
        }
        command_count = self.connection.execute("SELECT COUNT(*) FROM commands").fetchone()[0]
        event_count = self.connection.execute("SELECT COUNT(*) FROM events").fetchone()[0]
        return {
            "total": sum(counts.values()),
            "pending": counts.get("PENDING", 0),
            "accepted": counts.get("ACCEPTED", 0),
            "queued": counts.get("QUEUED", 0),
            "running": counts.get("RUNNING", 0),
            "succeeded": counts.get("SUCCEEDED", 0),
            "failed": counts.get("FAILED", 0),
            "canceled": counts.get("CANCELED", 0),
            "rejected": counts.get("REJECTED", 0),
            "events": event_count,
            "commands": command_count,
        }

    def info(self) -> dict:
        row = self.connection.execute(
            "SELECT updated_at FROM controller_snapshot WHERE singleton_id=1"
        ).fetchone()
        return {
            "enabled": True,
            "backend": "sqlite",
            "schema_version": SCHEMA_VERSION,
            "database_path": str(self.database_path),
            "updated_at": row["updated_at"] if row is not None else None,
            "database_bytes": self.database_path.stat().st_size if self.database_path.exists() else 0,
        }

    def close(self) -> None:
        self.connection.close()
