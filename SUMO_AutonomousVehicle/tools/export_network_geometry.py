#!/usr/bin/env python3
"""Export a SUMO network into a renderer-friendly JSON geometry file.

Coordinates remain in SUMO's projected Cartesian coordinate system. The
`location` metadata is retained so a visualizer can show the map's projection
and origin information without guessing.
"""
from __future__ import annotations

import argparse
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path


def allows_class(attrs: dict[str, str], vehicle_class: str) -> bool:
    allow = attrs.get("allow")
    disallow = attrs.get("disallow")
    if allow is not None:
        return vehicle_class in allow.split()
    if disallow is not None:
        return vehicle_class not in disallow.split()
    return True


def parse_shape(value: str | None) -> list[list[float]]:
    points: list[list[float]] = []
    if not value:
        return points
    for token in value.split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        try:
            points.append([float(parts[0]), float(parts[1])])
        except ValueError:
            continue
    return points


def main() -> int:
    root_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default=str(root_dir / "map.net.xml"))
    parser.add_argument("--output", default=str(root_dir / "visuals" / "network_geometry.json"))
    parser.add_argument("--vehicle-class", default="passenger")
    args = parser.parse_args()

    net_path = Path(args.network).expanduser().resolve()
    out_path = Path(args.output).expanduser().resolve()
    if not net_path.is_file():
        raise SystemExit(f"Network not found: {net_path}")

    xml_root = ET.parse(net_path).getroot()
    location_node = xml_root.find("location")
    location = dict(location_node.attrib) if location_node is not None else {}

    junctions = []
    for node in xml_root.findall("junction"):
        node_id = node.get("id", "")
        if node_id.startswith(":"):
            continue
        try:
            x, y = float(node.get("x", "nan")), float(node.get("y", "nan"))
        except ValueError:
            continue
        if not (x == x and y == y):
            continue
        junctions.append({"id": node_id, "x": x, "y": y, "type": node.get("type")})

    edges = []
    allowed_lane_indices: dict[str, set[int]] = {}
    normal_edge_ids: set[str] = set()
    for edge in xml_root.findall("edge"):
        edge_id = edge.get("id", "")
        if edge.get("function") == "internal" or edge_id.startswith(":"):
            continue
        lanes = []
        allowed_indices: set[int] = set()
        edge_allowed = allows_class(edge.attrib, args.vehicle_class)
        for lane in edge.findall("lane"):
            allowed = edge_allowed and allows_class(lane.attrib, args.vehicle_class)
            lane_index = int(lane.get("index", "0"))
            if allowed:
                allowed_indices.add(lane_index)
            try:
                speed = float(lane.get("speed", "0"))
            except ValueError:
                speed = 0.0
            try:
                length = float(lane.get("length", "0"))
            except ValueError:
                length = 0.0
            try:
                width = float(lane.get("width", "3.2"))
            except ValueError:
                width = 3.2
            lanes.append({
                "id": lane.get("id"),
                "index": lane_index,
                "allowed": allowed,
                "speed_mps": speed,
                "length_m": length,
                "width_m": width,
                "shape": parse_shape(lane.get("shape")),
            })
        allowed_lanes = [lane for lane in lanes if lane["allowed"]]
        normal_edge_ids.add(edge_id)
        allowed_lane_indices[edge_id] = allowed_indices
        edges.append({
            "id": edge_id,
            "from": edge.get("from"),
            "to": edge.get("to"),
            "type": edge.get("type"),
            "name": edge.get("name"),
            "priority": int(edge.get("priority", "0")),
            "passenger_allowed": bool(allowed_lanes),
            "lanes": lanes,
        })

    transitions = []
    seen = set()
    for connection in xml_root.findall("connection"):
        from_edge, to_edge = connection.get("from"), connection.get("to")
        from_lane, to_lane = connection.get("fromLane"), connection.get("toLane")
        if not all((from_edge, to_edge, from_lane is not None, to_lane is not None)):
            continue
        if from_edge not in normal_edge_ids or to_edge not in normal_edge_ids:
            continue
        try:
            from_lane_index = int(from_lane)
            to_lane_index = int(to_lane)
        except ValueError:
            continue
        if (
            from_lane_index not in allowed_lane_indices.get(from_edge, set())
            or to_lane_index not in allowed_lane_indices.get(to_edge, set())
        ):
            continue
        key = (from_edge, to_edge, from_lane, to_lane)
        if key in seen:
            continue
        seen.add(key)
        transitions.append({
            "from_edge": from_edge,
            "to_edge": to_edge,
            "from_lane": from_lane_index,
            "to_lane": to_lane_index,
            "direction": connection.get("dir"),
        })

    payload = {
        "schema_version": 1,
        "source_network": net_path.name,
        "vehicle_class": args.vehicle_class,
        "coordinate_system": "SUMO projected Cartesian x/y; not longitude/latitude",
        "location": location,
        "counts": {
            "junctions": len(junctions),
            "normal_edges": len(edges),
            "lanes": sum(len(edge["lanes"]) for edge in edges),
            "connection_records": len(transitions),
        },
        "junctions": junctions,
        "edges": edges,
        "transitions": transitions,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, separators=(",", ":"))
        handle.write("\n")

    print(f"Exported {len(edges)} edges and {len(junctions)} junctions to {out_path}")
    print("Location metadata:", json.dumps(location, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
