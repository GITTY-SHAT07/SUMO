"""
telemetry_logger.py

Logs autonomous vehicle state and controller outputs
to a CSV file for performance evaluation.
"""

import csv
from pathlib import Path


class TelemetryLogger:
    """Write simulation telemetry to a CSV file."""

    FIELDS = [
        "simulation_time",
        "model_x",
        "model_y",
        "model_heading",
        "model_speed",
        "sumo_x",
        "sumo_y",
        "sumo_speed",
        "cross_track_error",
        "heading_error",
        "steering_angle",
        "target_index",
        "current_edge",
    ]

    def __init__(self, output_path):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        self.file = self.output_path.open(
            "w",
            newline="",
            encoding="utf-8"
        )

        self.writer = csv.DictWriter(
            self.file,
            fieldnames=self.FIELDS
        )

        self.writer.writeheader()
        self.file.flush()

    def log(self, **values):
        """Write one simulation timestep to the CSV."""

        row = {
            field: values.get(field, "")
            for field in self.FIELDS
        }

        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        """Safely close the CSV file."""

        if not self.file.closed:
            self.file.close()