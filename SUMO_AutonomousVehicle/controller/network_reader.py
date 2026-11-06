"""Read a SUMO road network and expose junction- and edge-level graph data.

The original synthetic network remains the default. To load another network
for an isolated test, set SUMO_NETWORK_FILE to the desired .net.xml path.

In addition to the legacy junction graph, this module builds a legal
edge-to-edge successor graph from SUMO <connection> elements. Connections are
kept only when the connected fromLane and toLane both permit the requested
SUMO vehicle class (passenger by default).
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from collections import defaultdict


# ---------------------------------------------------------------------------
# Select network file and vehicle class
# ---------------------------------------------------------------------------

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_NETWORK_FILE = os.path.join(
    PROJECT_DIR,
    "network",
    "network.net.xml",
)

NETWORK_FILE = os.path.abspath(
    os.environ.get("SUMO_NETWORK_FILE") or DEFAULT_NETWORK_FILE
)

VEHICLE_CLASS = os.environ.get("SUMO_VEHICLE_CLASS", "passenger").strip()
if not VEHICLE_CLASS:
    raise ValueError("SUMO_VEHICLE_CLASS cannot be empty")

if not os.path.isfile(NETWORK_FILE):
    raise FileNotFoundError(f"SUMO network file not found: {NETWORK_FILE}")

print(f"[NETWORK] Loading: {NETWORK_FILE}")
print(f"[NETWORK] Vehicle class for legal connections: {VEHICLE_CLASS}")


# ---------------------------------------------------------------------------
# Permission helper
# ---------------------------------------------------------------------------

def _allows_vehicle_class(attributes, vehicle_class=VEHICLE_CLASS):
    """Return whether an edge/lane's allow/disallow attributes permit class.

    SUMO's default, when neither attribute is present, is to allow all
    vehicle classes. An explicit allow list takes precedence; otherwise an
    explicit disallow list blocks only the named classes.
    """
    allow_value = attributes.get("allow")
    disallow_value = attributes.get("disallow")

    if allow_value is not None:
        return vehicle_class in allow_value.split()

    if disallow_value is not None:
        return vehicle_class not in disallow_value.split()

    return True


# ---------------------------------------------------------------------------
# Parse SUMO network
# ---------------------------------------------------------------------------

root = ET.parse(NETWORK_FILE).getroot()

# Backward-compatible junction graph:
# graph[from_node][to_node] -> one road record.
graph = {}

# Complete junction graph:
# road_options[from_node][to_node] -> every normal edge for that pair.
road_options = {}

# Junction coordinates: positions[junction_id] = (x, y).
positions = {}

# Edge metadata keyed by SUMO edge ID. Contains only normal (non-internal)
# edges. All normal edges are retained, including edges that do not permit
# passenger vehicles; permission is recorded explicitly.
edge_data = {}

# Legal directed transitions between normal SUMO road edges.
# edge_successors[from_edge] is a sorted list of legal next-edge IDs.
edge_successors = {}

# Lane-level evidence for each legal edge transition.
# edge_transition_lanes[(from_edge, to_edge)] = set((fromLane, toLane), ...).
edge_transition_lanes = {}


# ---------------------------------------------------------------------------
# Read normal junction positions
# ---------------------------------------------------------------------------

for junction in root.findall("junction"):
    junction_id = junction.get("id", "")

    # SUMO internal junctions are not route-planning junctions.
    if junction_id.startswith(":"):
        continue

    x_value = junction.get("x")
    y_value = junction.get("y")
    if x_value is None or y_value is None:
        continue

    try:
        x = float(x_value)
        y = float(y_value)
    except (TypeError, ValueError):
        continue

    positions[junction_id] = (x, y)


# ---------------------------------------------------------------------------
# Read every normal road edge and its lane permissions
# ---------------------------------------------------------------------------

road_edge_count = 0
passenger_edge_count = 0

for edge in root.findall("edge"):
    edge_id = edge.get("id", "")

    # Ignore SUMO internal connector edges.
    if edge.get("function") == "internal" or edge_id.startswith(":"):
        continue

    from_node = edge.get("from")
    to_node = edge.get("to")
    if not from_node or not to_node:
        continue

    lanes = {}
    passenger_lane_indices = set()
    passenger_lane_lengths = []
    passenger_lane_speeds = []

    for lane in edge.findall("lane"):
        lane_index = lane.get("index")
        if lane_index is None:
            # SUMO-generated networks normally provide an index. Skip lanes
            # without one because connection lane references could not be
            # checked safely.
            continue

        lane_allowed = _allows_vehicle_class(edge.attrib) and _allows_vehicle_class(lane.attrib)
        lanes[lane_index] = {
            "allowed": lane_allowed,
            "lane_id": lane.get("id"),
        }

        if lane_allowed:
            passenger_lane_indices.add(lane_index)
            try:
                passenger_lane_lengths.append(float(lane.get("length")))
            except (TypeError, ValueError):
                pass
            try:
                passenger_lane_speeds.append(float(lane.get("speed")))
            except (TypeError, ValueError):
                pass

    # Use passenger-usable lanes for planning metadata when available;
    # otherwise retain the first lane's dimensions for legacy compatibility.
    first_lane = edge.find("lane")
    if first_lane is None:
        raise ValueError(f"No lane found for edge {edge_id}")

    try:
        fallback_length = float(first_lane.get("length"))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid lane length for edge {edge_id}") from exc

    length = min(passenger_lane_lengths) if passenger_lane_lengths else fallback_length
    speed = min(passenger_lane_speeds) if passenger_lane_speeds else None

    road = {
        "edge_id": edge_id,
        "from_node": from_node,
        "to_node": to_node,
        "length": length,
        "speed": speed,
        "passenger_allowed": bool(passenger_lane_indices),
        "passenger_lanes": sorted(passenger_lane_indices),
    }

    edge_data[edge_id] = {
        **road,
        "lanes": lanes,
    }

    # Preserve original graph interface for backward compatibility.
    graph.setdefault(from_node, {})[to_node] = road

    # Preserve every parallel road between the same ordered junction pair.
    road_options.setdefault(from_node, {}).setdefault(to_node, []).append(road)

    road_edge_count += 1
    if passenger_lane_indices:
        passenger_edge_count += 1


# ---------------------------------------------------------------------------
# Build legal edge-to-edge connectivity from SUMO <connection> entries
# ---------------------------------------------------------------------------

transition_lane_pairs = defaultdict(set)
connection_count = 0
ignored_connection_count = 0

for connection in root.findall("connection"):
    connection_count += 1

    from_edge = connection.get("from")
    to_edge = connection.get("to")
    from_lane = connection.get("fromLane")
    to_lane = connection.get("toLane")

    # A route-planning transition must connect two known normal road edges.
    # Connections touching internal edges are represented by SUMO's
    # connection/via geometry, but are not graph states in this planner.
    if (
        from_edge not in edge_data
        or to_edge not in edge_data
        or from_lane is None
        or to_lane is None
    ):
        ignored_connection_count += 1
        continue

    from_lane_data = edge_data[from_edge]["lanes"].get(from_lane)
    to_lane_data = edge_data[to_edge]["lanes"].get(to_lane)

    # Fail closed: only retain transitions for which both referenced lanes
    # exist and permit the selected vehicle class.
    if (
        from_lane_data is None
        or to_lane_data is None
        or not from_lane_data["allowed"]
        or not to_lane_data["allowed"]
    ):
        ignored_connection_count += 1
        continue

    transition_lane_pairs[(from_edge, to_edge)].add((from_lane, to_lane))

successor_sets = defaultdict(set)
for from_edge, to_edge in transition_lane_pairs:
    successor_sets[from_edge].add(to_edge)

edge_successors = {
    edge_id: sorted(successors)
    for edge_id, successors in successor_sets.items()
}

edge_transition_lanes = {
    pair: set(lane_pairs)
    for pair, lane_pairs in transition_lane_pairs.items()
}


# ---------------------------------------------------------------------------
# Network access helpers
# ---------------------------------------------------------------------------

def get_neighbors(node):
    """Return distinct junctions directly reachable from node (legacy API)."""
    return graph.get(node, {})


def get_road_options(from_node, to_node=None):
    """Return all road records for a junction pair, or all from a node."""
    options = road_options.get(from_node, {})
    if to_node is None:
        return options
    return options.get(to_node, [])


def get_edge_successors(edge_id):
    """Return legal passenger-vehicle successor edges for edge_id."""
    return edge_successors.get(edge_id, [])


def is_legal_edge_transition(from_edge, to_edge):
    """Return True only when SUMO defines a usable lane connection."""
    return to_edge in edge_successors.get(from_edge, [])


def get_edge_cost(from_node, to_node):
    """Return the legacy static road-length cost for a junction pair."""
    if from_node not in graph:
        raise ValueError(f"Unknown junction: {from_node}")
    if to_node not in graph[from_node]:
        raise ValueError(f"No road from {from_node} to {to_node}")
    return graph[from_node][to_node]["length"]


def find_path(start, goal):
    """Find a basic junction path; primarily a network-reader smoke test."""
    if start not in positions:
        raise ValueError(f"Start junction '{start}' does not exist.")
    if goal not in positions:
        raise ValueError(f"Goal junction '{goal}' does not exist.")

    queue = [start]
    previous = {start: None}

    while queue:
        current = queue.pop(0)
        if current == goal:
            break

        for neighbor in get_neighbors(current):
            if neighbor not in previous:
                previous[neighbor] = current
                queue.append(neighbor)

    if goal not in previous:
        return []

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = previous[current]

    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Diagnostics when run directly
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unique_edge_ids = {
        road["edge_id"]
        for destinations in road_options.values()
        for roads in destinations.values()
        for road in roads
    }
    ordered_pair_count = sum(len(destinations) for destinations in road_options.values())
    parallel_pair_count = sum(
        1
        for destinations in road_options.values()
        for roads in destinations.values()
        if len(roads) > 1
    )

    print("\n=== ROAD NETWORK SUMMARY ===")
    print("Network file:", NETWORK_FILE)
    print("Junction positions:", len(positions))
    print("Graph junctions with outgoing roads:", len(graph))
    print("Ordered junction pairs:", ordered_pair_count)
    print("Parallel-road junction pairs:", parallel_pair_count)
    print("Normal road edges loaded:", road_edge_count)
    print("Unique road edges preserved:", len(unique_edge_ids))
    print(f"Edges permitting {VEHICLE_CLASS}:", passenger_edge_count)
    print("SUMO connection records:", connection_count)
    print("Legal edge-to-edge transitions:", len(edge_transition_lanes))
    print("Ignored/unusable connection records:", ignored_connection_count)

    # Preserve the original tiny-network smoke test where applicable.
    if "J0" in positions and "J2" in positions:
        print("Synthetic junction path J0 -> J2:", find_path("J0", "J2"))
    else:
        print("J0/J2 smoke test skipped (this is not the synthetic network).")
