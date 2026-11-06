#!/usr/bin/env python3
"""Validate that every committed SUMO config references existing network/routes."""
from __future__ import annotations
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "simulation"


def resolve(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def main() -> int:
    failed = False
    configs = sorted(CONFIG_DIR.glob("*.sumocfg"))
    if not configs:
        print("FAIL: no .sumocfg files found")
        return 1
    for config in configs:
        root = ET.parse(config).getroot()
        net_node = root.find("./input/net-file")
        if net_node is None or not net_node.get("value"):
            print(f"FAIL: {config.name}: no input/net-file")
            failed = True
            continue
        net_path = resolve(config.parent, net_node.get("value"))
        if not net_path.is_file():
            print(f"FAIL: {config.name}: missing network {net_path}")
            failed = True
            continue
        route_node = root.find("./input/route-files")
        routes = [] if route_node is None else [x.strip() for x in route_node.get("value", "").split(",") if x.strip()]
        missing = [str(resolve(config.parent, value)) for value in routes if not resolve(config.parent, value).is_file()]
        if missing:
            print(f"FAIL: {config.name}: missing route files: {missing}")
            failed = True
        else:
            print(f"PASS: {config.name} -> {net_path.name} ({len(routes)} route files)")
    if failed:
        return 1
    print(f"SUMO CONFIG VALIDATION: PASS ({len(configs)} configs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
