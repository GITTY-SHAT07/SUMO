#!/usr/bin/env python3
"""Run the engine's offline acceptance checks before a live SUMO run.

This validates Python modules, configuration references, OSM route legality,
closure/hazard behaviour, the visualizer file interface and map geometry.
It does not replace a live SUMO + TraCI simulation smoke test.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run_step(label: str, command: list[str], env: dict[str, str]) -> None:
    print(f"\n{'=' * 72}\n{label}\n{'=' * 72}", flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-file", default=str(ROOT / "map.net.xml"))
    parser.add_argument("--start-edge", default="-1513939047")
    parser.add_argument("--goal-junction", default="2227588809")
    parser.add_argument("--vehicle-class", default="passenger")
    parser.add_argument(
        "--geometry-output", default=str(ROOT / "visuals" / "network_geometry.json")
    )
    args = parser.parse_args()

    network = Path(args.network_file).expanduser().resolve()
    geometry_output = Path(args.geometry_output).expanduser().resolve()
    if not network.is_file():
        parser.error(f"Network file does not exist: {network}")
    if not args.start_edge.strip() or not args.goal_junction.strip():
        parser.error("--start-edge and --goal-junction must be non-empty")

    env = os.environ.copy()
    env["SUMO_NETWORK_FILE"] = str(network)
    env["SUMO_VEHICLE_CLASS"] = args.vehicle_class

    run_step(
        "1/6 Python syntax/bytecode compilation",
        [sys.executable, "-m", "compileall", "-q", "controller", "tools"],
        env,
    )
    run_step(
        "2/6 Visualizer interface smoke test",
        [sys.executable, "tools/test_engine_interface.py"],
        env,
    )
    run_step(
        "3/6 SUMO config reference validation",
        [sys.executable, "tools/validate_configs.py"],
        env,
    )
    run_step(
        "4/6 OSM route/closure/no-route/hazard validation",
        [
            sys.executable,
            "tools/validate_engine.py",
            "--network-file",
            str(network),
            "--start-edge",
            args.start_edge,
            "--goal-junction",
            args.goal_junction,
        ],
        env,
    )
    run_step(
        "5/6 Graded hazard manager unit test",
        [sys.executable, "controller/hazard_manager.py"],
        env,
    )
    run_step(
        "6/6 Renderer geometry export",
        [
            sys.executable,
            "tools/export_network_geometry.py",
            "--network",
            str(network),
            "--output",
            str(geometry_output),
            "--vehicle-class",
            args.vehicle_class,
        ],
        env,
    )

    print("\nOFFLINE ENGINE PREFLIGHT: PASS")
    print("NOTE: Run a live SUMO/TraCI smoke test on the target machine before handoff.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
