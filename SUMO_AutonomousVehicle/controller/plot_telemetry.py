
"""
plot_telemetry.py

Generate performance plots from the telemetry CSV.
Run from the project root:
    python3 controller/plot_telemetry.py
"""

import csv
from pathlib import Path

import matplotlib.pyplot as plt


# Resolve paths from the project directory, not the terminal location.
PROJECT_DIR = Path(__file__).resolve().parent.parent
CSV_FILE = PROJECT_DIR / "logs" / "telemetry.csv"
OUTPUT_DIR = PROJECT_DIR / "results" / "telemetry_plots"


def load_telemetry():
    """Read numeric telemetry values from the CSV file."""

    if not CSV_FILE.exists():
        raise FileNotFoundError(
            f"Telemetry file not found: {CSV_FILE}"
        )

    with CSV_FILE.open("r", newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("Telemetry CSV contains no data rows.")

    required = [
        "simulation_time",
        "cross_track_error",
        "heading_error",
        "steering_angle",
    ]

    for column in required:
        if column not in rows[0]:
            raise ValueError(f"Missing CSV column: {column}")

    data = {}

    for column in required:
        data[column] = [
            float(row[column])
            for row in rows
            if row[column] != ""
        ]

    return data


def save_plot(times, values, title, ylabel, filename, zero_line=False):
    """Create and save one time-series plot."""

    if len(times) != len(values):
        raise ValueError(f"Time/value length mismatch: {title}")

    plt.figure(figsize=(11, 5))
    plt.plot(times, values, linewidth=1.5, label=ylabel)

    if zero_line:
        plt.axhline(0, linestyle="--", linewidth=1)

    plt.title(title)
    plt.xlabel("Simulation time (s)")
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    output_path = OUTPUT_DIR / filename
    plt.savefig(output_path, dpi=160)
    plt.close()

    print(f"Saved: {output_path}")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    data = load_telemetry()

    times = data["simulation_time"]

    save_plot(
        times,
        data["cross_track_error"],
        "Signed Cross-Track Error",
        "Cross-track error (m)",
        "cross_track_error.png",
        zero_line=True,
    )

    save_plot(
        times,
        data["heading_error"],
        "Heading Tracking Error",
        "Heading error (degrees)",
        "heading_error.png",
        zero_line=True,
    )

    save_plot(
        times,
        data["steering_angle"],
        "Steering Controller Output",
        "Steering angle (degrees)",
        "steering_angle.png",
        zero_line=True,
    )

    print("\nAll plots generated successfully.")


if __name__ == "__main__":
    main()
