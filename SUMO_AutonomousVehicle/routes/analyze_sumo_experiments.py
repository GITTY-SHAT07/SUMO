#!/usr/bin/env python3
"""Analyze SUMO autonomous-navigation experiment telemetry and console logs.

Standard-library only. Run from the SUMO project root:
    python tools/analyze_sumo_experiments.py --root .

Outputs:
    results/experiment_run_metrics.csv
    results/experiment_planner_summary.csv
    results/experiment_diagnostic_summary.csv

The script does not assume that missing collision or route-change data is zero.
Blank fields mean the log did not provide that metric.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

FLOAT_FIELDS = {
    "simulation_time", "model_x", "model_y", "model_heading", "model_speed",
    "sumo_x", "sumo_y", "sumo_speed", "cross_track_error", "heading_error",
    "steering_angle", "target_index",
}


def as_float(value: Any) -> float | None:
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def first_existing(paths: list[Path]) -> Path | None:
    return next((path for path in paths if path.is_file()), None)


def parse_console(path: Path | None) -> dict[str, Any]:
    data: dict[str, Any] = {
        "console_file": str(path) if path else "",
        "goal_reached_console": "",
        "return_code": "",
        "dynamic_reroute_count": "",
        "replan_check_count": "",
        "emergency_braking_warnings": "",
        "traci_implied_speed_warnings": "",
        "collision_events": "",
        "initial_main_route_cost_s": "",
        "initial_alternative_route_cost_s": "",
        "initial_selected_route_traffic_cost_s": "",
        "initial_route_preference": "",
        "initial_route_selected_edges": "",
        "initial_legal_main_route": "",
        "initial_legal_alternative_route": "",
    }
    if not path or not path.is_file():
        return data

    text = path.read_text(encoding="utf-8", errors="replace")
    lower = text.lower()
    data["goal_reached_console"] = "GOAL REACHED" in text
    data["dynamic_reroute_count"] = len(re.findall(r"Dynamic reroute applied:", text))
    data["replan_check_count"] = len(re.findall(r"Checking for a better route", text))
    data["emergency_braking_warnings"] = len(
        re.findall(r"performs emergency braking", text, flags=re.IGNORECASE)
    )
    data["traci_implied_speed_warnings"] = len(
        re.findall(r"implied speed of .*?exceeding maximum speed", text, flags=re.IGNORECASE)
    )

    # Count explicit collision records only. If no collision indication is in
    # the console, retain blank rather than asserting zero collisions.
    collision_matches = re.findall(
        r"(?:collision(?:s)?(?: detected)?\s*[:=]\s*)(\d+)",
        text,
        flags=re.IGNORECASE,
    )
    collision_lines = [
        line for line in text.splitlines()
        if re.search(r"collision", line, flags=re.IGNORECASE)
        and not re.search(r"no collision", line, flags=re.IGNORECASE)
    ]
    if collision_matches:
        data["collision_events"] = max(int(value) for value in collision_matches)
    elif collision_lines:
        data["collision_events"] = len(collision_lines)

    def capture_float(pattern: str) -> float | str:
        match = re.search(pattern, text, flags=re.IGNORECASE)
        return as_float(match.group(1)) if match else ""

    data["initial_main_route_cost_s"] = capture_float(
        r"MAIN ROUTE(?: \(K-ROUTE \d+\))? traffic cost:\s*([0-9.]+)"
    )
    data["initial_alternative_route_cost_s"] = capture_float(
        r"ALTERNATIVE ROUTE(?: \(K-ROUTE \d+\))? traffic cost:\s*([0-9.]+)"
    )
    data["initial_selected_route_traffic_cost_s"] = capture_float(
        r"A\* selected path traffic cost:\s*([0-9.]+)"
    )

    if "ALTERNATIVE ROUTE IS CHEAPER" in text:
        data["initial_route_preference"] = "alternative"
    elif "MAIN ROUTE IS CHEAPER" in text:
        data["initial_route_preference"] = "main"
    elif "Both routes have equal traffic cost" in text:
        data["initial_route_preference"] = "equal"

    selected_match = re.search(
        r"\[ROUTE DIAGNOSTIC\] A\* selected remaining edges:\s*(\[[^\n]*\])",
        text,
    )
    if selected_match:
        data["initial_route_selected_edges"] = selected_match.group(1)

    if "Legal successor sequence from current edge: YES" in text:
        legal_yes = len(re.findall(r"Legal successor sequence from current edge: YES", text))
        legal_no = len(re.findall(r"Legal successor sequence from current edge: NO", text))
        if legal_yes >= 2 and legal_no == 0:
            data["initial_legal_main_route"] = True
            data["initial_legal_alternative_route"] = True
        elif legal_yes == 1 and legal_no == 0:
            data["initial_legal_main_route"] = True
    if "Legal successor sequence from current edge: NO" in text:
        legal_lines = re.findall(r"Legal successor sequence from current edge: (YES|NO)", text)
        if len(legal_lines) >= 2:
            data["initial_legal_main_route"] = legal_lines[0] == "YES"
            data["initial_legal_alternative_route"] = legal_lines[1] == "YES"

    return data


def parse_run_name(path: Path) -> tuple[str, str]:
    match = re.search(r"telemetry(?:_large)?_(shortest|traffic|risk)_seed(\d+)\.csv$", path.name)
    if match:
        return match.group(1), match.group(2)
    return "unknown", "unknown"


def route_sequence(rows: list[dict[str, str]]) -> list[str]:
    sequence: list[str] = []
    for row in rows:
        edge = (row.get("current_edge") or "").strip()
        if edge and (not sequence or sequence[-1] != edge):
            sequence.append(edge)
    return sequence


def summarize_telemetry(path: Path) -> dict[str, Any]:
    rows = read_csv_rows(path)
    if not rows:
        raise ValueError(f"Telemetry file is empty: {path}")

    columns = set(rows[0])
    parsed: dict[str, list[float]] = {}
    for field in FLOAT_FIELDS & columns:
        values = [as_float(row.get(field)) for row in rows]
        parsed[field] = [value for value in values if value is not None]

    times = parsed.get("simulation_time", [])
    xs = parsed.get("model_x", [])
    ys = parsed.get("model_y", [])
    cte = parsed.get("cross_track_error", [])
    heading = parsed.get("heading_error", [])
    model_speed = parsed.get("model_speed", [])
    sumo_speed = parsed.get("sumo_speed", [])

    path_distance = ""
    if len(xs) == len(ys) and len(xs) >= 2:
        path_distance = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))

    sync_errors: list[float] = []
    if {"model_x", "model_y", "sumo_x", "sumo_y"}.issubset(columns):
        for row in rows:
            mx, my = as_float(row.get("model_x")), as_float(row.get("model_y"))
            sx, sy = as_float(row.get("sumo_x")), as_float(row.get("sumo_y"))
            if None not in (mx, my, sx, sy):
                sync_errors.append(math.hypot(mx - sx, my - sy))

    seq = route_sequence(rows)
    normal_edges = [edge for edge in seq if not edge.startswith(":")]

    return {
        "telemetry_file": str(path),
        "samples": len(rows),
        "simulation_start_s": min(times) if times else "",
        "simulation_end_s": max(times) if times else "",
        "simulated_duration_s": max(times) - min(times) if times else "",
        "model_path_distance_m": path_distance,
        "mean_abs_cte_m": statistics.fmean(abs(v) for v in cte) if cte else "",
        "p95_abs_cte_m": percentile([abs(v) for v in cte], 95) if cte else "",
        "max_abs_cte_m": max((abs(v) for v in cte), default=""),
        "mean_abs_heading_error_deg": statistics.fmean(abs(v) for v in heading) if heading else "",
        "max_abs_heading_error_deg": max((abs(v) for v in heading), default=""),
        "mean_model_speed_mps": statistics.fmean(model_speed) if model_speed else "",
        "max_model_speed_mps": max(model_speed, default=""),
        "max_sumo_speed_mps": max(sumo_speed, default=""),
        "mean_model_sumo_position_error_m": statistics.fmean(sync_errors) if sync_errors else "",
        "max_model_sumo_position_error_m": max(sync_errors, default=""),
        "unique_edge_ids": len(set(seq)),
        "unique_non_internal_edges": len(set(normal_edges)),
        "route_edge_sequence": " -> ".join(seq),
    }


def percentile(values: list[float], p: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * p / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def load_status(root: Path) -> dict[str, dict[str, str]]:
    candidates = [
        root / "logs" / "baseline_experiment_status.csv",
        root / "baseline_experiment_status.csv",
        root / "results" / "baseline_experiment_status.csv",
    ]
    status_paths = [path for path in candidates if path.is_file()]
    status_paths.extend(sorted((root / "results" / "runs").glob("*/experiment_status.csv")))
    combined: dict[str, dict[str, str]] = {}
    for status_path in status_paths:
        for row in read_csv_rows(status_path):
            telemetry_name = row.get("telemetry_file", "")
            if telemetry_name:
                combined[telemetry_name] = row
                combined[str(Path("logs") / telemetry_name)] = row
    return combined


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="SUMO project root")
    parser.add_argument("--telemetry-dir", type=Path, default=None, help="Defaults to ROOT/logs")
    parser.add_argument("--console-dir", type=Path, default=None, help="Defaults to ROOT/logs")
    parser.add_argument("--output-dir", type=Path, default=None, help="Defaults to ROOT/results/analysis")
    args = parser.parse_args()

    root = args.root.resolve()
    telemetry_dir = (args.telemetry_dir or root / "logs").resolve()
    console_dir = (args.console_dir or root / "logs").resolve()
    output_dir = (args.output_dir or root / "results" / "analysis").resolve()

    telemetry_files = sorted(telemetry_dir.glob("telemetry*.csv"))
    if not telemetry_files:
        print(f"No telemetry*.csv files found in {telemetry_dir}")
        return 2

    statuses = load_status(root)
    run_rows: list[dict[str, Any]] = []

    for telemetry_path in telemetry_files:
        planner, seed = parse_run_name(telemetry_path)
        metrics = summarize_telemetry(telemetry_path)
        status_key_candidates = [
            str(Path("logs") / telemetry_path.name),
            telemetry_path.name,
            str(telemetry_path),
        ]
        status = next((statuses[key] for key in status_key_candidates if key in statuses), {})

        console_candidates = []
        if status.get("console_file"):
            console_candidates.append(root / status["console_file"])
        console_candidates.extend([
            console_dir / f"experiment_console_{planner}_seed{seed}.txt",
            console_dir / f"{planner}_seed{seed}.txt",
            console_dir / f"{planner}_{seed}.txt",
        ])
        console_path = first_existing(console_candidates)
        console = parse_console(console_path)

        row: dict[str, Any] = {
            "planner": planner,
            "seed": seed,
            **metrics,
            **console,
            "status_return_code": status.get("return_code", ""),
            "status_goal_reached": status.get("goal_reached", ""),
            "status": status.get("status", ""),
        }
        if row["goal_reached_console"] == "" and status.get("goal_reached") != "":
            row["goal_reached_console"] = status.get("goal_reached", "")
        run_rows.append(row)

    run_rows.sort(key=lambda row: (row["planner"], str(row["seed"])))
    write_csv(output_dir / "experiment_run_metrics.csv", run_rows)

    numeric_metrics = [
        "simulated_duration_s", "model_path_distance_m", "mean_abs_cte_m",
        "p95_abs_cte_m", "max_abs_cte_m", "mean_abs_heading_error_deg",
        "max_abs_heading_error_deg", "max_sumo_speed_mps",
        "mean_model_sumo_position_error_m", "max_model_sumo_position_error_m",
    ]
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in run_rows:
        grouped[row["planner"]].append(row)

    summary_rows = []
    for planner, rows in sorted(grouped.items()):
        summary: dict[str, Any] = {"planner": planner, "runs": len(rows)}
        for metric in numeric_metrics:
            values = [as_float(row.get(metric)) for row in rows]
            values = [value for value in values if value is not None]
            summary[f"{metric}_mean"] = statistics.fmean(values) if values else ""
            summary[f"{metric}_sample_sd"] = statistics.stdev(values) if len(values) > 1 else (0.0 if values else "")
        for field in ["dynamic_reroute_count", "replan_check_count", "emergency_braking_warnings", "traci_implied_speed_warnings"]:
            values = [as_float(row.get(field)) for row in rows]
            values = [value for value in values if value is not None]
            summary[f"{field}_mean"] = statistics.fmean(values) if values else ""
        known_goals = [str(row.get("goal_reached_console", "")).lower() for row in rows]
        known_goals = [value for value in known_goals if value in {"true", "false"}]
        summary["goal_success_rate"] = (
            sum(value == "true" for value in known_goals) / len(known_goals)
            if known_goals else ""
        )
        summary_rows.append(summary)

    write_csv(output_dir / "experiment_planner_summary.csv", summary_rows)

    diagnostic_rows = []
    for row in run_rows:
        if row.get("initial_main_route_cost_s") != "" or row.get("initial_alternative_route_cost_s") != "":
            diagnostic_rows.append({
                "planner": row["planner"],
                "seed": row["seed"],
                "main_route_cost_s": row.get("initial_main_route_cost_s", ""),
                "alternative_route_cost_s": row.get("initial_alternative_route_cost_s", ""),
                "selected_route_cost_s": row.get("initial_selected_route_traffic_cost_s", ""),
                "initial_preference": row.get("initial_route_preference", ""),
                "selected_edges": row.get("initial_route_selected_edges", ""),
                "main_legal": row.get("initial_legal_main_route", ""),
                "alternative_legal": row.get("initial_legal_alternative_route", ""),
            })
    write_csv(output_dir / "experiment_diagnostic_summary.csv", diagnostic_rows)

    print(f"Analyzed {len(run_rows)} telemetry files.")
    print(f"Run metrics: {output_dir / 'experiment_run_metrics.csv'}")
    print(f"Planner summary: {output_dir / 'experiment_planner_summary.csv'}")
    print(f"Route diagnostics: {output_dir / 'experiment_diagnostic_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
