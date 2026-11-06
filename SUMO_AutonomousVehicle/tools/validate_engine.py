#!/usr/bin/env python3
"""Static OSM/A* validation. Does not require a running SUMO simulation."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "controller"
if str(CONTROLLER) not in sys.path:
    sys.path.insert(0, str(CONTROLLER))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-file", default=str(ROOT / "map.net.xml"))
    parser.add_argument("--start-edge", default="-1513939047")
    parser.add_argument("--goal-junction", default="2227588809")
    args = parser.parse_args()

    network_file = str(Path(args.network_file).expanduser().resolve())
    os.environ["SUMO_NETWORK_FILE"] = network_file
    os.environ.setdefault("SUMO_VEHICLE_CLASS", "passenger")

    import network_reader as network
    from path_planner import plan_path
    from obstacle_manager import block_edge, unblock_edge, get_blocked_edges
    from hazard_manager import set_hazard, clear_hazard, get_hazard_cost

    if args.start_edge not in network.edge_data:
        raise SystemExit(f"Start edge not found: {args.start_edge}")
    if args.goal_junction not in network.positions:
        raise SystemExit(f"Goal junction not found: {args.goal_junction}")

    start_junction = network.edge_data[args.start_edge]["to_node"]
    route, route_cost, edges = plan_path(
        start_junction,
        args.goal_junction,
        return_edges=True,
        start_edge=args.start_edge,
    )
    if not route or not edges:
        raise SystemExit("FAIL: no route found for the requested OSM endpoints")

    full_edges = [args.start_edge] + list(edges)
    invalid = [
        (a, b)
        for a, b in zip(full_edges, full_edges[1:])
        if not network.is_legal_edge_transition(a, b)
    ]
    if invalid:
        raise SystemExit(f"FAIL: illegal edge transitions detected: {invalid[:5]}")

    print(f"PASS: loaded {len(network.edge_data)} OSM road edges")
    print(f"PASS: loaded {len(network.positions)} junction positions")
    print(f"PASS: built {len(network.edge_transition_lanes)} legal edge transitions")
    print(f"PASS: A* found {len(full_edges)}-edge route, cost={route_cost:.2f}")
    print("PASS: every consecutive road pair is a legal passenger-vehicle transition")

    # A closure test must find a valid detour for at least one edge on the route.
    detour_found = False
    tested_closure = None
    for edge_id in edges[:-1]:
        block_edge(edge_id)
        try:
            alt_nodes, _, alt_edges = plan_path(
                start_junction,
                args.goal_junction,
                return_edges=True,
                start_edge=args.start_edge,
            )
            if alt_nodes and edge_id not in alt_edges:
                detour_found = True
                tested_closure = edge_id
                break
        finally:
            unblock_edge(edge_id)
    if not detour_found:
        raise SystemExit("FAIL: no tested route edge had a legal detour on this network")
    print(f"PASS: closure of {tested_closure} produced an alternate legal route")

    # If every legal successor of the starting edge is blocked, no path should be returned.
    successors = list(network.get_edge_successors(args.start_edge))
    if not successors:
        raise SystemExit("FAIL: start edge has no legal outgoing successor")
    for edge_id in successors:
        block_edge(edge_id)
    try:
        no_path, _, _ = plan_path(
            start_junction,
            args.goal_junction,
            return_edges=True,
            start_edge=args.start_edge,
        )
        if no_path:
            raise SystemExit("FAIL: planner returned a route although every first successor was blocked")
    finally:
        for edge_id in successors:
            unblock_edge(edge_id)
    print("PASS: planner reports no route when all legal first successors are blocked")

    # Hazard severity must affect the planner cost continuously while keeping the edge open.
    hazard_edge = full_edges[min(1, len(full_edges) - 1)]
    set_hazard(hazard_edge, 3, "validator test")
    try:
        penalty = get_hazard_cost(hazard_edge)
        if penalty <= 0:
            raise SystemExit("FAIL: severity-3 hazard did not add a route penalty")
        print(f"PASS: severity-3 hazard adds {penalty:.2f} cost units and leaves the edge traversable")
    finally:
        clear_hazard(hazard_edge)

    if get_blocked_edges():
        raise SystemExit("FAIL: blocked-edge state was not clean after validation")
    print("PASS: temporary test state cleaned up")
    print("ENGINE VALIDATION: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
