
"""
Automated baseline experiment runner.

Runs:
    3 planners x 5 SUMO random seeds = 15 experiments.

All experiments use:
    - The same OSM network
    - The same SUMO configuration
    - The same destination
    - Matched random seeds
    - No injected road closures or hazards

Run:
    python run_baseline_experiments.py
"""

import csv
import os
import shutil
import subprocess
import sys
import time

from pathlib import Path
from datetime import datetime


# ============================================================
# 1. PROJECT CONFIGURATION
# ============================================================

PROJECT_DIR = Path(__file__).resolve().parent

MAIN_FILE = PROJECT_DIR / "controller" / "main.py"

LOG_DIR = PROJECT_DIR / "logs"

PLANNERS = [
    "shortest",
    "traffic",
    "risk"
]

SEEDS = [42, 43, 44, 45, 46]


# ============================================================
# 2. VALIDATE PROJECT
# ============================================================

if not MAIN_FILE.is_file():

    raise FileNotFoundError(
        f"Could not find controller: {MAIN_FILE}\n"
        "Save this script in the project root."
    )

LOG_DIR.mkdir(
    parents=True,
    exist_ok=True
)


# ============================================================
# 3. PRESERVE PREVIOUS EXPERIMENT FILES
# ============================================================

timestamp = datetime.now().strftime(
    "%Y%m%d_%H%M%S"
)

ARCHIVE_DIR = LOG_DIR / f"previous_runs_{timestamp}"

for planner in PLANNERS:

    for seed in SEEDS:

        telemetry_file = LOG_DIR / (
            f"telemetry_large_{planner}_seed{seed}.csv"
        )

        console_file = LOG_DIR / (
            f"experiment_console_{planner}_seed{seed}.txt"
        )

        for old_file in (telemetry_file, console_file):

            if old_file.exists():

                ARCHIVE_DIR.mkdir(
                    parents=True,
                    exist_ok=True
                )

                shutil.copy2(
                    old_file,
                    ARCHIVE_DIR / old_file.name
                )

print("Previous experiment logs preserved.")


# ============================================================
# 4. EXPERIMENT INITIALIZATION
# ============================================================

status_rows = []

total_runs = len(PLANNERS) * len(SEEDS)

run_number = 0

print("\n" + "=" * 65)
print("AUTOMATED BASELINE EXPERIMENTS")
print("=" * 65)

print("Network: OSM large map")
print("Planners:", PLANNERS)
print("Random seeds:", SEEDS)
print("Total runs:", total_runs)
print("Hazard injection: Disabled")
print("Road closure: Disabled")

print("=" * 65)


# ============================================================
# 5. EXECUTE EXPERIMENTS
# ============================================================

for seed in SEEDS:

    for planner in PLANNERS:

        run_number += 1

        run_name = f"{planner}_seed{seed}"

        console_file = (
            LOG_DIR /
            f"experiment_console_{run_name}.txt"
        )

        telemetry_file = (
            LOG_DIR /
            f"telemetry_large_{planner}_seed{seed}.csv"
        )

        # ----------------------------------------------------
        # Configure environment
        # ----------------------------------------------------

        env = os.environ.copy()

        env["SUMO_SCENARIO"] = "large"
        env["SUMO_PLANNER"] = planner
        env["SUMO_SEED"] = str(seed)

        # Ensure this is a normal-traffic baseline experiment.
        env.pop("SUMO_TEST_HAZARD", None)
        env.pop("SUMO_TEST_CLOSURE", None)

        print("\n" + "-" * 65)

        print(
            f"RUN {run_number}/{total_runs}"
        )

        print("Planner:", planner)
        print("Seed:", seed)

        # ----------------------------------------------------
        # Execute main.py
        # ----------------------------------------------------

        start_time = time.time()

        wall_start = time.perf_counter()

        with console_file.open(
            "w",
            encoding="utf-8"
        ) as output:

            result = subprocess.run(
                [
                    sys.executable,
                    str(MAIN_FILE)
                ],
                cwd=str(PROJECT_DIR),
                env=env,
                stdout=output,
                stderr=subprocess.STDOUT,
                check=False
            )

        wall_time = (
            time.perf_counter() - wall_start
        )

        # ----------------------------------------------------
        # Inspect run output
        # ----------------------------------------------------

        log_text = console_file.read_text(
            encoding="utf-8",
            errors="replace"
        )

        goal_reached = (
            "GOAL REACHED" in log_text
        )

        telemetry_created = (
            telemetry_file.is_file()
            and telemetry_file.stat().st_mtime
            >= start_time - 1.0
        )

        run_passed = (
            result.returncode == 0
            and goal_reached
            and telemetry_created
        )

        status = (
            "PASS"
            if run_passed
            else "CHECK_LOG"
        )

        # ----------------------------------------------------
        # Store results
        # ----------------------------------------------------

        status_rows.append({

            "planner": planner,

            "seed": seed,

            "return_code": result.returncode,

            "goal_reached": goal_reached,

            "telemetry_created": telemetry_created,

            "wall_time_seconds": round(
                wall_time,
                3
            ),

            "status": status,

            "telemetry_file": str(
                telemetry_file.relative_to(PROJECT_DIR)
            ),

            "console_file": str(
                console_file.relative_to(PROJECT_DIR)
            )
        })

        print("\nRun result:", status)

        print(
            "Return code:",
            result.returncode
        )

        print(
            "Goal reached:",
            goal_reached
        )

        print(
            "Telemetry saved:",
            telemetry_created
        )

        print(
            f"Wall-clock execution time: {wall_time:.2f}s"
        )


# ============================================================
# 6. SAVE EXPERIMENT STATUS
# ============================================================

status_file = (
    LOG_DIR /
    "baseline_experiment_status.csv"
)

fieldnames = [
    "planner",
    "seed",
    "return_code",
    "goal_reached",
    "telemetry_created",
    "wall_time_seconds",
    "status",
    "telemetry_file",
    "console_file"
]

with status_file.open(
    "w",
    newline="",
    encoding="utf-8"
) as csv_file:

    writer = csv.DictWriter(
        csv_file,
        fieldnames=fieldnames
    )

    writer.writeheader()

    writer.writerows(status_rows)


# ============================================================
# 7. FINAL REPORT
# ============================================================

passed_runs = sum(
    row["status"] == "PASS"
    for row in status_rows
)

failed_runs = total_runs - passed_runs

print("\n" + "=" * 65)

print("BASELINE EXPERIMENT BATCH COMPLETED")

print("=" * 65)

print("Total runs:", total_runs)

print("Passed:", passed_runs)

print("Need inspection:", failed_runs)

print(
    "\nStatus report:",
    status_file
)

print(
    "\nTelemetry directory:",
    LOG_DIR
)

if ARCHIVE_DIR.exists():

    print(
        "Previous logs archived in:",
        ARCHIVE_DIR
    )

print("=" * 65)

if failed_runs > 0:

    print(
        "Some experiments need inspection. "
        "Check their individual console logs."
    )

else:

    print(
        "All baseline experiments passed basic checks."
    )
