"""Stable file-based interface between the SUMO engine and a visualizer.

The engine publishes telemetry/events/state/route files and polls a JSONL
command inbox. The visualization layer can run in a separate process and does
not need to import the controller or TraCI.
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


class EngineInterface:
    """Publish a run's state and events; receive commands from a visualizer."""

    SCHEMA_VERSION = 1

    def __init__(self, run_dir: str | Path, metadata: Optional[Dict[str, Any]] = None):
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.events_path = self.run_dir / "events.jsonl"
        self.state_path = self.run_dir / "state.json"
        self.route_path = self.run_dir / "route.json"
        self.manifest_path = self.run_dir / "manifest.json"
        self.commands_path = self.run_dir / "commands.jsonl"
        self.summary_path = self.run_dir / "summary.json"

        self.events_path.touch(exist_ok=True)
        self.commands_path.touch(exist_ok=True)
        self._command_offset = self.commands_path.stat().st_size

        self.write_json_atomic(
            self.manifest_path,
            {
                "schema_version": self.SCHEMA_VERSION,
                "created_at_utc": self._now(),
                **(metadata or {}),
                "interface": {
                    "telemetry": "telemetry.csv",
                    "events": self.events_path.name,
                    "state_snapshot": self.state_path.name,
                    "active_route": self.route_path.name,
                    "command_inbox": self.commands_path.name,
                    "summary": self.summary_path.name,
                },
            },
        )
        self.log_event("engine_started", simulation_time=0.0, run_dir=str(self.run_dir))

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
        """Write a JSON snapshot atomically so readers never see partial JSON."""
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2, default=str)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp_name, path)
        except Exception:
            try:
                os.unlink(tmp_name)
            except OSError:
                pass
            raise

    def log_event(self, event: str, simulation_time: Optional[float] = None, **payload: Any) -> None:
        record = {
            "schema_version": self.SCHEMA_VERSION,
            "event": str(event),
            "simulation_time": simulation_time,
            "wall_time_utc": self._now(),
            "payload": payload,
        }
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
            handle.flush()

    def publish_state(self, state: Dict[str, Any]) -> None:
        self.write_json_atomic(
            self.state_path,
            {
                "schema_version": self.SCHEMA_VERSION,
                "updated_at_utc": self._now(),
                **state,
            },
        )

    def publish_route(self, route: Dict[str, Any]) -> None:
        self.write_json_atomic(
            self.route_path,
            {
                "schema_version": self.SCHEMA_VERSION,
                "updated_at_utc": self._now(),
                **route,
            },
        )

    def publish_summary(self, summary: Dict[str, Any]) -> None:
        self.write_json_atomic(
            self.summary_path,
            {
                "schema_version": self.SCHEMA_VERSION,
                "finished_at_utc": self._now(),
                **summary,
            },
        )

    def read_commands(self) -> List[Dict[str, Any]]:
        """Return complete, newly appended JSONL commands only.

        The visualizer appends one JSON object per line. A trailing partial
        line is retained for the next poll rather than being discarded.
        """
        commands: List[Dict[str, Any]] = []
        try:
            with self.commands_path.open("rb") as handle:
                size = self.commands_path.stat().st_size
                if size < self._command_offset:
                    # Inbox was rotated/truncated; start at the new beginning.
                    self._command_offset = 0
                handle.seek(self._command_offset)

                while True:
                    line_start = handle.tell()
                    line = handle.readline()
                    if not line:
                        self._command_offset = handle.tell()
                        break
                    if not line.endswith(b"\n"):
                        self._command_offset = line_start
                        break
                    self._command_offset = handle.tell()
                    if not line.strip():
                        continue
                    try:
                        item = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                        commands.append({"_parse_error": str(exc), "_raw": line.decode("utf-8", errors="replace")})
                        continue
                    if isinstance(item, dict):
                        commands.append(item)
                    else:
                        commands.append({"_parse_error": "Each command must be a JSON object", "_raw": item})
        except FileNotFoundError:
            self.commands_path.touch(exist_ok=True)
        return commands
