#!/usr/bin/env python3
"""Convert an OpenStreetMap .osm extract into a SUMO .net.xml network."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path


def locate_netconvert() -> str:
    found = shutil.which("netconvert")
    if found:
        return found
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidate = Path(sumo_home) / "bin" / "netconvert"
        if candidate.is_file():
            return str(candidate)
    raise SystemExit(
        "netconvert was not found. Add SUMO's bin directory to PATH or set SUMO_HOME."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("osm_file", type=Path, help="Input OSM XML/PBF extract supported by netconvert")
    parser.add_argument("output_net", type=Path, help="Output SUMO network .net.xml")
    parser.add_argument("--netconvert", default=None, help="Explicit netconvert executable")
    parser.add_argument("--keep-vehicle-class", default="passenger", help="Vehicle class retained by netconvert (default: passenger)")
    args = parser.parse_args()

    osm_file = args.osm_file.expanduser().resolve()
    output_net = args.output_net.expanduser().resolve()
    if not osm_file.is_file():
        raise SystemExit(f"OSM input not found: {osm_file}")
    output_net.parent.mkdir(parents=True, exist_ok=True)

    executable = args.netconvert or locate_netconvert()
    command = [
        executable,
        "--osm-files", str(osm_file),
        "--output-file", str(output_net),
        "--keep-edges.by-vclass", args.keep_vehicle_class,
        "--junctions.join",
        # Preserve lane/edge geometry: the controller builds its reference path
        # from SUMO lane shapes, and the visualizer consumes those same shapes.
        "--roundabouts.guess",
        "--ramps.guess",
        "--tls.guess-signals",
        "--tls.discard-simple",
    ]
    print("Running:", " ".join(command))
    subprocess.run(command, check=True)
    print(f"Created SUMO network: {output_net}")
    print("Next: run tools/validate_engine.py --network-file", output_net)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
