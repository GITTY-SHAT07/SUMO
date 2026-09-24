import math

# Import obstacle checking.
# A* will ignore roads that are currently blocked.
from obstacle_manager import is_blocked

# Import the road-network information.
from network_reader import (
    graph,
    positions,
    get_neighbors,
    get_edge_cost
)


# ============================================================
# HEURISTIC
# ============================================================

def heuristic(node, goal):
    """
    Estimate the minimum remaining travel time from
    'node' to 'goal'.

    We use:

        straight-line distance / maximum speed

    The traffic penalty is NOT included in the heuristic,
    keeping it conservative.
    """

    # Check that the current node exists.
    if node not in positions:
        raise ValueError(
            f"Unknown node: {node}"
        )

    # Check that the goal exists.
    if goal not in positions:
        raise ValueError(
            f"Unknown goal: {goal}"
        )

    # Current junction coordinates.
    x1, y1 = positions[node]

    # Goal junction coordinates.
    x2, y2 = positions[goal]

    # Straight-line distance.
    distance = math.sqrt(
        (x2 - x1) ** 2
        + (y2 - y1) ** 2
    )

    # Maximum vehicle speed from SUMO.
    max_speed = 13.89

    # Estimated minimum travel time.
    return distance / max_speed


# ============================================================
# A* PATH PLANNER
# ============================================================

def plan_path(
    start,
    goal,
    edge_cost_function=None
):
    """
    Find the lowest-cost path from start to goal.

    Parameters
    ----------
    start : str
        Starting junction.

    goal : str
        Destination junction.

    edge_cost_function : function, optional
        Function receiving a SUMO edge ID and returning
        its cost.

        None:
            Use static road length.

        get_traffic_cost:
            Use live traffic-aware cost.

    Returns
    -------
    path : list
        Junction sequence from start to goal.

    total_cost : float
        Total cost of the selected path.
    """

    # ========================================================
    # VALIDATE START AND GOAL
    # ========================================================

    if start not in positions:
        raise ValueError(
            f"Start node '{start}' does not exist."
        )

    if goal not in positions:
        raise ValueError(
            f"Goal node '{goal}' does not exist."
        )


    # ========================================================
    # INITIALIZE A*
    # ========================================================

    # Nodes waiting to be explored.
    open_nodes = {start}

    # Stores the parent of each node.
    previous = {
        start: None
    }

    # Actual cost from start to each node.
    g_cost = {
        start: 0.0
    }

    # Estimated total cost:
    #
    # f(n) = g(n) + h(n)
    #
    f_cost = {
        start: heuristic(
            start,
            goal
        )
    }


    # ========================================================
    # MAIN A* SEARCH
    # ========================================================

    while open_nodes:

        # ----------------------------------------------------
        # Select node with smallest f-cost
        # ----------------------------------------------------

        current = min(
            open_nodes,
            key=lambda node: f_cost.get(
                node,
                float("inf")
            )
        )


        # ----------------------------------------------------
        # Goal reached
        # ----------------------------------------------------

        if current == goal:
            break


        # Remove current node from the open set.
        open_nodes.remove(
            current
        )


        # ----------------------------------------------------
        # Explore neighboring junctions
        # ----------------------------------------------------

        for neighbor in get_neighbors(
            current
        ):

            # Get information about the road.
            road = graph[current][neighbor]

            # Get corresponding SUMO edge ID.
            edge_id = road["edge_id"]


            # =================================================
            # OBSTACLE CHECK
            # =================================================

            # If the road is blocked, A* must not use it.
            if is_blocked(edge_id):

                print(
                    f"A*: skipping blocked edge {edge_id}"
                )

                continue


            # =================================================
            # CALCULATE EDGE COST
            # =================================================

            if edge_cost_function is not None:

                # Traffic-aware cost.
                edge_cost = edge_cost_function(
                    edge_id
                )

            else:

                # Static distance cost.
                edge_cost = get_edge_cost(
                    current,
                    neighbor
                )


            # ------------------------------------------------
            # Validate cost
            # ------------------------------------------------

            if edge_cost < 0:

                raise ValueError(
                    f"Negative edge cost for {edge_id}"
                )


            # =================================================
            # CALCULATE NEW PATH COST
            # =================================================

            new_g_cost = (
                g_cost[current]
                + edge_cost
            )


            # Current best known cost to neighbor.
            old_cost = g_cost.get(
                neighbor,
                float("inf")
            )


            # =================================================
            # UPDATE IF THIS ROUTE IS BETTER
            # =================================================

            if new_g_cost < old_cost:

                # Remember how we reached the neighbor.
                previous[neighbor] = current

                # Save actual cost.
                g_cost[neighbor] = new_g_cost

                # Calculate:
                #
                # f(n) = g(n) + h(n)
                #
                f_cost[neighbor] = (
                    new_g_cost
                    + heuristic(
                        neighbor,
                        goal
                    )
                )

                # Add node to the set of nodes to explore.
                open_nodes.add(
                    neighbor
                )


    # ========================================================
    # NO PATH FOUND
    # ========================================================

    if goal not in previous:

        return [], float("inf")


    # ========================================================
    # RECONSTRUCT PATH
    # ========================================================

    path = []

    current = goal

    # Work backwards from goal to start.
    while current is not None:

        path.append(
            current
        )

        current = previous[current]


    # Reverse to obtain:
    #
    # Start -> Goal
    #
    path.reverse()


    # Final cost of selected path.
    total_cost = g_cost[goal]


    return path, total_cost


# ============================================================
# JUNCTION PATH -> SUMO EDGE PATH
# ============================================================

def junctions_to_edges(path):
    """
    Convert a junction path into SUMO edge IDs.

    Example:

        ['J0', 'J1', 'J2']

    becomes:

        ['E0', 'E2']
    """

    route_edges = []


    # Process every consecutive pair of junctions.
    for i in range(
        len(path) - 1
    ):

        from_node = path[i]
        to_node = path[i + 1]


        # Make sure the starting junction exists.
        if from_node not in graph:

            raise ValueError(
                f"Unknown junction: {from_node}"
            )


        # Make sure the road exists.
        if to_node not in graph[from_node]:

            raise ValueError(
                f"No road from "
                f"{from_node} to {to_node}"
            )


        # Get road information.
        road = graph[from_node][to_node]


        # Add SUMO edge ID.
        route_edges.append(
            road["edge_id"]
        )


    return route_edges


# ============================================================
# TEST WITHOUT SUMO
# ============================================================

if __name__ == "__main__":

    # --------------------------------------------------------
    # Basic A* test
    # --------------------------------------------------------

    start = "J0"
    goal = "J2"

    path, total_cost = plan_path(
        start,
        goal
    )

    route_edges = junctions_to_edges(
        path
    )

    print(
        "Junction route:",
        path
    )

    print(
        "SUMO edge route:",
        route_edges
    )

    print(
        "Total cost:",
        total_cost
    )

