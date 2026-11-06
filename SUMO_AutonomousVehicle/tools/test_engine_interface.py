#!/usr/bin/env python3
"""Unit smoke test for the engine-to-visualizer file interface."""

import json
import tempfile
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))
from engine_interface import EngineInterface


def main():
    with tempfile.TemporaryDirectory() as temp_dir:
        bridge = EngineInterface(temp_dir, {"run_id": "test-run", "planner": "risk"})
        bridge.log_event("route_planned", 1.0, route_edges=["e0", "e1"])
        bridge.publish_route({"route_edges": ["e0", "e1"], "path_points": [[0, 0], [1, 1]]})
        bridge.publish_state({"status": "running", "position": {"x": 1.0, "y": 2.0}})

        command_file = Path(temp_dir) / "commands.jsonl"
        with command_file.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps({"type": "set_hazard", "edge_id": "e1", "severity": 3}) + "\n")
        commands = bridge.read_commands()
        assert len(commands) == 1 and commands[0]["type"] == "set_hazard"
        assert bridge.read_commands() == []

        assert json.loads((Path(temp_dir) / "state.json").read_text())["status"] == "running"
        assert json.loads((Path(temp_dir) / "route.json").read_text())["route_edges"] == ["e0", "e1"]
        events = [json.loads(line) for line in (Path(temp_dir) / "events.jsonl").read_text().splitlines()]
        assert any(item["event"] == "route_planned" for item in events)
        bridge.publish_summary({"goal_reached": True})
        assert json.loads((Path(temp_dir) / "summary.json").read_text())["goal_reached"] is True

    print("ENGINE INTERFACE TEST: PASS")


if __name__ == "__main__":
    main()
