"""
Edge-based A* planner for SUMO road networks.

The planner searches over normal SUMO road edges rather than junctions.
A transition is allowed only when network_reader identifies a legal
passenger-vehicle lane connection in the SUMO connection records.

Backward compatibility:
- plan_path(start, goal) returns (junction_path, total_cost).
- plan_path(..., return_edges=True) returns
  (junction_path, total_cost, exact_edge_path).
- junctions_to_edges(path) remains available.

For rerouting from a vehicle already travelling on a road, pass that
road as start_edge. The returned edge_path contains the roads AFTER
start_edge. The current edge is not included in that returned path.
"""

import heapq
import math

from obstacle_manager import is_blocked

from network_reader import (
    graph,
    positions,
    road_options,
    edge_data,
    edge_successors,
    is_legal_edge_transition,
)


# ============================================================
# 1. HEURISTIC CONFIGURATION
# ============================================================

def calculate_heuristic_scale():
    """
    Calculate a conservative scale for the distance heuristic.

    Only passenger-usable edges are considered.
    """

    ratios = []

    for edge_id, road in edge_data.items():

        if not road.get("passenger_allowed", False):
            continue

        from_node = road["from_node"]
        to_node = road["to_node"]

        if from_node not in positions or to_node not in positions:
            continue

        x1, y1 = positions[from_node]
        x2, y2 = positions[to_node]

        straight_distance = math.hypot(
            x2 - x1,
            y2 - y1
        )

        if straight_distance <= 1e-9:
            continue

        road_length = float(road["length"])

        if road_length >= 0:
            ratios.append(
                road_length / straight_distance
            )

    if not ratios:
        return 0.0

    return max(
        0.0,
        min(1.0, min(ratios))
    )


STATIC_HEURISTIC_SCALE = calculate_heuristic_scale()


# ============================================================
# 2. HEURISTIC
# ============================================================

def heuristic(node, goal, edge_cost_function=None):
    """
    Estimate remaining cost from a junction to the goal.

    Static distance costs:
        Use scaled Euclidean distance.

    Custom costs:
        Use zero heuristic because their units and lower bounds
        are unknown. The search then behaves as Dijkstra.
    """

    if edge_cost_function is not None:
        return 0.0

    if node not in positions or goal not in positions:
        return 0.0

    x1, y1 = positions[node]
    x2, y2 = positions[goal]

    return (
        STATIC_HEURISTIC_SCALE
        * math.hypot(x2 - x1, y2 - y1)
    )


# ============================================================
# 3. EDGE-BASED A* PATH PLANNER
# ============================================================

def plan_path(
    start,
    goal,
    edge_cost_function=None,
    return_edges=False,
    start_edge=None,
):
    """
    Find a minimum-cost legal route between two junctions.

    Parameters
    ----------
    start : str
        Starting junction ID.

    goal : str
        Destination junction ID.

    edge_cost_function : callable, optional
        Receives a SUMO edge ID and returns its traversal cost.
        If omitted, static road length is used.

    return_edges : bool
        False:
            Return (junction_path, total_cost).

        True:
            Return (junction_path, total_cost, exact_edge_path).

    start_edge : str, optional
        Current road edge occupied by the vehicle.

        If supplied, the search can leave this edge only through
        its legal SUMO successors.

        This edge is not included in the returned edge_path.

    Returns
    -------
    junction_path : list[str]
        Selected junction sequence.

    total_cost : float
        Cost of the selected route.

    edge_path : list[str], optional
        Exact SUMO edge sequence.
    """

    def make_result(path, cost, edges):

        if return_edges:
            return path, cost, edges

        return path, cost

    # --------------------------------------------------------
    # A. VALIDATE START AND GOAL
    # --------------------------------------------------------

    if start not in positions:
        raise ValueError(
            f"Start junction '{start}' does not exist."
        )

    if goal not in positions:
        raise ValueError(
            f"Goal junction '{goal}' does not exist."
        )

    if start == goal:
        return make_result(
            [start],
            0.0,
            []
        )

    # --------------------------------------------------------
    # B. DETERMINE INITIAL EDGES
    # --------------------------------------------------------

    if start_edge is not None:

        if start_edge not in edge_data:
            raise ValueError(
                f"Unknown start edge: {start_edge}"
            )

        current_edge_data = edge_data[start_edge]

        if current_edge_data["to_node"] != start:
            raise ValueError(
                f"start_edge '{start_edge}' ends at "
                f"'{current_edge_data['to_node']}', "
                f"not start junction '{start}'."
            )

        if not current_edge_data.get(
            "passenger_allowed",
            False
        ):
            raise ValueError(
                f"start_edge '{start_edge}' does not permit "
                "passenger vehicles."
            )

        # A vehicle already on a road can only enter roads
        # connected legally to its current road.
        initial_edges = list(
            edge_successors.get(start_edge, [])
        )

    else:

        # At a trip origin, there is no incoming edge.
        # Any passenger-accessible road leaving the origin
        # may be considered.

        initial_edges = []

        for roads in road_options.get(start, {}).values():

            for road in roads:

                edge_id = road["edge_id"]

                if (
                    edge_id in edge_data
                    and edge_data[edge_id].get(
                        "passenger_allowed",
                        False
                    )
                ):
                    initial_edges.append(edge_id)

    initial_edges = sorted(set(initial_edges))

    # --------------------------------------------------------
    # C. EDGE COST CACHE
    # --------------------------------------------------------

    edge_cost_cache = {}

    def get_cost(edge_id):

        if edge_id not in edge_cost_cache:

            if edge_id not in edge_data:
                raise ValueError(
                    f"Unknown road edge: {edge_id}"
                )

            if edge_cost_function is None:

                cost = float(
                    edge_data[edge_id]["length"]
                )

            else:

                cost = float(
                    edge_cost_function(edge_id)
                )

            if (
                not math.isfinite(cost)
                or cost < 0
            ):
                raise ValueError(
                    f"Invalid cost for edge {edge_id}: {cost}"
                )

            edge_cost_cache[edge_id] = cost

        return edge_cost_cache[edge_id]

    # --------------------------------------------------------
    # D. INITIALIZE EDGE-BASED A*
    # --------------------------------------------------------

    g_cost = {}
    previous_edge = {}

    open_heap = []

    for edge_id in initial_edges:

        if is_blocked(edge_id):
            continue

        road = edge_data[edge_id]

        if road["from_node"] != start:
            continue

        candidate_g = get_cost(edge_id)

        old_g = g_cost.get(
            edge_id,
            float("inf")
        )

        if candidate_g < old_g:

            g_cost[edge_id] = candidate_g
            previous_edge[edge_id] = None

            h_cost = heuristic(
                road["to_node"],
                goal,
                edge_cost_function
            )

            heapq.heappush(
                open_heap,
                (
                    candidate_g + h_cost,
                    candidate_g,
                    edge_id
                )
            )

    # --------------------------------------------------------
    # E. MAIN SEARCH
    # --------------------------------------------------------

    goal_edge = None

    while open_heap:

        current_f, current_g, current_edge = (
            heapq.heappop(open_heap)
        )

        # Ignore outdated priority queue entries.
        best_g = g_cost.get(
            current_edge,
            float("inf")
        )

        if current_g > best_g + 1e-9:
            continue

        current_road = edge_data[current_edge]

        current_junction = current_road["to_node"]

        # Destination reached.
        if current_junction == goal:

            goal_edge = current_edge
            break

        # Only legally connected successor edges are explored.
        for next_edge in edge_successors.get(
            current_edge,
            []
        ):

            if is_blocked(next_edge):
                continue

            next_road = edge_data.get(next_edge)

            if next_road is None:
                continue

            if not next_road.get(
                "passenger_allowed",
                False
            ):
                continue

            # Defensive check for network consistency.
            if next_road["from_node"] != current_junction:
                continue

            candidate_g = (
                current_g + get_cost(next_edge)
            )

            old_g = g_cost.get(
                next_edge,
                float("inf")
            )

            if candidate_g + 1e-9 < old_g:

                g_cost[next_edge] = candidate_g

                previous_edge[next_edge] = current_edge

                h_cost = heuristic(
                    next_road["to_node"],
                    goal,
                    edge_cost_function
                )

                candidate_f = (
                    candidate_g + h_cost
                )

                heapq.heappush(
                    open_heap,
                    (
                        candidate_f,
                        candidate_g,
                        next_edge
                    )
                )

    # --------------------------------------------------------
    # F. CHECK WHETHER A ROUTE EXISTS
    # --------------------------------------------------------

    if goal_edge is None:

        return make_result(
            [],
            float("inf"),
            []
        )

    # --------------------------------------------------------
    # G. RECONSTRUCT EXACT EDGE PATH
    # --------------------------------------------------------

    edge_path = []

    current_edge = goal_edge

    while current_edge is not None:

        edge_path.append(current_edge)

        current_edge = previous_edge[current_edge]

    edge_path.reverse()

    # --------------------------------------------------------
    # H. BUILD JUNCTION PATH FROM EDGE PATH
    # --------------------------------------------------------

    junction_path = [start]

    current_junction = start

    for edge_id in edge_path:

        road = edge_data[edge_id]

        if road["from_node"] != current_junction:
            raise RuntimeError(
                "Planner error: selected edge sequence "
                "is not contiguous."
            )

        current_junction = road["to_node"]

        junction_path.append(current_junction)

    # --------------------------------------------------------
    # I. FINAL LEGAL TRANSITION VALIDATION
    # --------------------------------------------------------

    for first_edge, second_edge in zip(
        edge_path,
        edge_path[1:]
    ):

        if not is_legal_edge_transition(
            first_edge,
            second_edge
        ):
            raise RuntimeError(
                f"Illegal transition generated: "
                f"{first_edge} -> {second_edge}"
            )

    total_cost = g_cost[goal_edge]

    return make_result(
        junction_path,
        total_cost,
        edge_path
    )


# ============================================================
# 4. JUNCTION PATH -> LEGAL SUMO EDGE PATH
# ============================================================

def junctions_to_edges(
    path,
    selected_edges=None
):
    """
    Convert junction IDs into a legal SUMO edge sequence.

    Preferred:
        Supply selected_edges returned by plan_path().

    Compatibility fallback:
        If selected_edges is omitted, find the lowest-distance
        legal edge sequence for the supplied junction path.

    Note:
        The fallback uses static road length. For a route planned
        using traffic or hazard costs, always pass selected_edges.
    """

    path = list(path)

    if len(path) <= 1:

        if selected_edges not in (None, []):
            raise ValueError(
                "A one-junction path cannot contain road edges."
            )

        return []

    # --------------------------------------------------------
    # A. USE EXACT A* EDGE SEQUENCE
    # --------------------------------------------------------

    if selected_edges is not None:

        selected_edges = list(selected_edges)

        expected_edges = len(path) - 1

        if len(selected_edges) != expected_edges:
            raise ValueError(
                "Junction path and edge path have "
                "incompatible lengths."
            )

        for index, edge_id in enumerate(selected_edges):

            road = edge_data.get(edge_id)

            if road is None:
                raise ValueError(
                    f"Unknown selected edge: {edge_id}"
                )

            if (
                road["from_node"] != path[index]
                or road["to_node"] != path[index + 1]
            ):
                raise ValueError(
                    f"Selected edge '{edge_id}' does not match "
                    f"junction transition {path[index]} -> "
                    f"{path[index + 1]}"
                )

            if not road.get(
                "passenger_allowed",
                False
            ):
                raise ValueError(
                    f"Selected edge '{edge_id}' does not "
                    "permit passenger vehicles."
                )

            if is_blocked(edge_id):
                raise ValueError(
                    f"Selected edge '{edge_id}' is blocked."
                )

            if index > 0:

                previous_id = selected_edges[index - 1]

                if not is_legal_edge_transition(
                    previous_id,
                    edge_id
                ):
                    raise ValueError(
                        f"Illegal edge transition: "
                        f"{previous_id} -> {edge_id}"
                    )

        return selected_edges

    # --------------------------------------------------------
    # B. COMPATIBILITY FALLBACK
    # --------------------------------------------------------

    # Find passenger-usable candidate edges for every
    # consecutive junction pair.

    layers = []

    for from_node, to_node in zip(
        path,
        path[1:]
    ):

        candidates = []

        for road in road_options.get(
            from_node,
            {}
        ).get(
            to_node,
            []
        ):

            edge_id = road["edge_id"]

            edge_info = edge_data.get(edge_id)

            if edge_info is None:
                continue

            if not edge_info.get(
                "passenger_allowed",
                False
            ):
                continue

            if is_blocked(edge_id):
                continue

            candidates.append(edge_id)

        if not candidates:
            raise ValueError(
                f"No passenger-usable road from "
                f"{from_node} to {to_node}"
            )

        layers.append(sorted(set(candidates)))

    # Dynamic programming:
    # edge_id -> (minimum total length, previous edge)

    costs = {}
    predecessors = {}

    # First junction transition.
    for edge_id in layers[0]:

        costs[edge_id] = float(
            edge_data[edge_id]["length"]
        )

        predecessors[edge_id] = None

    # Remaining junction transitions.
    for layer_index in range(1, len(layers)):

        next_costs = {}
        next_predecessors = {}

        for edge_id in layers[layer_index]:

            best_cost = float("inf")
            best_previous = None

            for previous_id, previous_cost in costs.items():

                if not is_legal_edge_transition(
                    previous_id,
                    edge_id
                ):
                    continue

                candidate_cost = (
                    previous_cost
                    + float(edge_data[edge_id]["length"])
                )

                if candidate_cost < best_cost:

                    best_cost = candidate_cost
                    best_previous = previous_id

            if best_previous is not None:

                next_costs[edge_id] = best_cost

                next_predecessors[edge_id] = best_previous

        if not next_costs:
            raise ValueError(
                "The supplied junction path has no "
                "legal edge-to-edge route."
            )

        costs = next_costs

        predecessors.update(next_predecessors)

    # Select the cheapest final road.
    final_edge = min(
        costs,
        key=costs.get
    )

    route_edges = [final_edge]

    while predecessors[route_edges[-1]] is not None:

        route_edges.append(
            predecessors[route_edges[-1]]
        )

    route_edges.reverse()

    return route_edges


# ============================================================
# 5. TEST WITHOUT STARTING SUMO
# ============================================================

if __name__ == "__main__":

    # Use the original synthetic test when available.
    # Otherwise find a test route in the loaded network.

    if "J0" in positions and "J2" in positions:

        start = "J0"
        goal = "J2"

    else:

        test_pair = None

        for first_edge, successors in edge_successors.items():

            first_road = edge_data.get(first_edge)

            if (
                first_road is None
                or not first_road.get("passenger_allowed", False)
            ):
                continue

            for second_edge in successors:

                second_road = edge_data.get(second_edge)

                if second_road is None:
                    continue

                candidate_start = first_road["from_node"]
                candidate_goal = second_road["to_node"]

                if (
                    candidate_start != candidate_goal
                    and candidate_start in positions
                    and candidate_goal in positions
                ):
                    test_pair = (
                        candidate_start,
                        candidate_goal
                    )
                    break

            if test_pair is not None:
                break

        if test_pair is None:
            raise RuntimeError(
                "Could not find a passenger-usable test route."
            )

        start, goal = test_pair

    # Backward-compatible two-value call.
    path, total_cost = plan_path(
        start,
        goal
    )

    # Preferred call: retrieve exact selected edges.
    edge_path_junctions, edge_path_cost, selected_edges = plan_path(
        start,
        goal,
        return_edges=True
    )

    print("\n=== EDGE-BASED A* TEST ===")

    print("Start junction:", start)
    print("Goal junction:", goal)

    print("Junction route:", path)

    print("Exact selected edge route:", selected_edges)

    print("Total static cost:", total_cost)

    assert path, "No path found."

    assert edge_path_junctions, "Extended A* call found no path."

    assert selected_edges

    assert len(selected_edges) == len(edge_path_junctions) - 1

    assert junctions_to_edges(
        edge_path_junctions,
        selected_edges
    ) == selected_edges

    for first_edge, second_edge in zip(
        selected_edges,
        selected_edges[1:]
    ):

        assert is_legal_edge_transition(
            first_edge,
            second_edge
        )

    print("\nPASS: Edge-based A* found a route.")

    print(
        "PASS: Every consecutive road pair is a legal SUMO transition."
    )

    print("PASS: Exact selected edge sequence was preserved.")