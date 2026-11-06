#!/usr/bin/env python3
"""Run matched SUMO planner experiments while preserving the SUMO config.

Example from the SUMO project root:
    python tools/run_matched_experiment.py \
      --scenario corridor_congestion \
      --route-files routes/routes_large.rou.xml routes/background_corridor_test.rou.xml \
      --seeds 42 43 44

The script temporarily replaces <route-files value="..."> in
simulation/simulation_large.sumocfg, executes each planner/seed combination,
archives telemetry and console output into results/runs/<scenario>/, and
restores the original SUMO config byte-for-byte in a finally block.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--scenario", required=True, help="Short scenario label, e.g. corridor_congestion")
    parser.add_argument("--route-files", nargs="+", required=True, help="Project-root-relative SUMO route XML files; ego route first")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44])
    parser.add_argument("--planners", nargs="+", choices=["shortest", "traffic", "risk"], default=["shortest", "traffic", "risk"])
    parser.add_argument("--timeout", type=int, default=600)
    parser.add_argument("--diagnostic", choices=["on", "off"], default="on")
    args = parser.parse_args()

    root = args.root.resolve()
    config = root / "simulation" / "simulation_large.sumocfg"
    main_script = root / "controller" / "main.py"
    logs_dir = root / "logs"
    archive_dir = root / "results" / "runs" / args.scenario
    archive_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    if not config.is_file():
        raise FileNotFoundError(f"SUMO config not found: {config}")
    if not main_script.is_file():
        raise FileNotFoundError(f"Controller not found: {main_script}")

    route_paths = [(root / rel).resolve() for rel in args.route_files]
    for path in route_paths:
        if not path.is_file():
            raise FileNotFoundError(f"Route file not found: {path}")
    if len(route_paths) < 1:
        raise ValueError("At least the ego route file must be supplied.")

    original_config = config.read_bytes()
    config_text = original_config.decode("utf-8")
    route_value = ",".join(os.path.relpath(path, config.parent) for path in route_paths)
    updated_text, replacements = re.subn(
        r'(<route-files\b[^>]*\bvalue=")[^"]*(")',
        lambda match: match.group(1) + route_value + match.group(2),
        config_text,
        count=1,
    )
    if replacements != 1:
        raise RuntimeError(
            "Could not safely find one <route-files value=...> entry in "
            f"{config}. No configuration changes were made."
        )

    status_rows: list[dict[str, object]] = []
    telemetry_live = logs_dir / "telemetry_large_{planner}_seed{seed}.csv"

    try:
        config.write_text(updated_text, encoding="utf-8")
        print(f"Scenario: {args.scenario}")
        print(f"Route files: {route_value}")
        print(f"Output: {archive_dir}")

        for seed in args.seeds:
            for planner in args.planners:
                telemetry_path = logs_dir / f"telemetry_large_{planner}_seed{seed}.csv"
                console_path = archive_dir / f"experiment_console_{planner}_seed{seed}.txt"
                archived_telemetry = archive_dir / telemetry_path.name

                # Remove the previous live file so a failed run cannot be
                # mistaken for a newly generated telemetry file.
                if telemetry_path.exists():
                    telemetry_path.unlink()

                env = os.environ.copy()
                env.update({
                    "SUMO_SCENARIO": "large",
                    "SUMO_PLANNER": planner,
                    "SUMO_SEED": str(seed),
                    "SUMO_DIAGNOSE_ROUTE_COSTS": "1" if args.diagnostic == "on" else "0",
                })
                env.pop("SUMO_TEST_CLOSURE", None)
                env.pop("SUMO_TEST_HAZARD", None)

                command = [sys.executable, str(main_script)]
                started = time.perf_counter()
                print(f"\n[{planner} | seed {seed}] starting")
                with console_path.open("w", encoding="utf-8") as console:
                    try:
                        result = subprocess.run(
                            command,
                            cwd=root,
                            env=env,
                            stdout=console,
                            stderr=subprocess.STDOUT,
                            timeout=args.timeout,
                            check=False,
                        )
                        return_code = result.returncode
                        timed_out = False
                    except subprocess.TimeoutExpired:
                        return_code = -1
                        timed_out = True
                        console.write(f"\nRUNNER TIMEOUT after {args.timeout} seconds.\n")

                wall_time = time.perf_counter() - started
                console_text = console_path.read_text(encoding="utf-8", errors="replace")
                goal_reached = "GOAL REACHED" in console_text
                telemetry_created = telemetry_path.is_file() and telemetry_path.stat().st_size > 0

                if telemetry_created:
                    shutil.move(str(telemetry_path), str(archived_telemetry))

                status = "PASS" if return_code == 0 and goal_reached and telemetry_created else "FAIL"
                if timed_out:
                    status = "TIMEOUT"

                status_rows.append({
                    "scenario": args.scenario,
                    "planner": planner,
                    "seed": seed,
                    "return_code": return_code,
                    "timed_out": timed_out,
                    "goal_reached": goal_reached,
                    "telemetry_created": telemetry_created,
                    "wall_time_seconds": round(wall_time, 3),
                    "status": status,
                    "telemetry_file": archived_telemetry.name if telemetry_created else "",
                    "console_file": console_path.name,
                })
                print(f"[{planner} | seed {seed}] {status} | goal={goal_reached} | telemetry={telemetry_created} | wall={wall_time:.2f}s")

                with (archive_dir / "experiment_status.csv").open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(status_rows[0].keys()))
                    writer.writeheader()
                    writer.writerows(status_rows)

    finally:
        # Restore original XML exactly, even if a process fails.
        config.write_bytes(original_config)
        print("\nOriginal simulation_large.sumocfg restored byte-for-byte.")

    passed = sum(row["status"] == "PASS" for row in status_rows)
    print(f"\nCompleted: {passed}/{len(status_rows)} runs passed.")
    print(f"Status CSV: {archive_dir / 'experiment_status.csv'}")
    return 0 if passed == len(status_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
