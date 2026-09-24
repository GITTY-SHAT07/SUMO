import os
import xml.etree.ElementTree as ET


# ============================================================
# 1. FIND THE PROJECT DIRECTORY
# ============================================================

# __file__ = location of this Python file
#
# network_reader.py is inside:
#
#     SUMO_AutonomousVehicle/controller/
#
# Going two levels through dirname() gives:
#
#     SUMO_AutonomousVehicle/
#
PROJECT_DIR = os.path.dirname(
    os.path.dirname(
        os.path.abspath(__file__)
    )
)


# ============================================================
# 2. LOCATE THE SUMO NETWORK FILE
# ============================================================

# Build the complete path to:
#
#     network/network.net.xml
#
NETWORK_FILE = os.path.join(
    PROJECT_DIR,
    "network",
    "network.net.xml"
)


# ============================================================
# 3. READ THE XML NETWORK
# ============================================================

# Parse the SUMO network XML file.
tree = ET.parse(
    NETWORK_FILE
)

# Get the top-level <net> element.
root = tree.getroot()


# ============================================================
# 4. CREATE GRAPH AND JUNCTION POSITION DICTIONARY
# ============================================================

# graph stores the road network in this form:
#
# graph[from_node][to_node] =
# {
#     "edge_id": "E0",
#     "length": 92.8
# }
#
# This allows the path planner to ask:
#
#     "What roads can I take from J1?"

graph = {}


# Store the x,y coordinate of every normal junction.
#
# Example:
#
# positions["J1"] = (100.0, 0.0)
#
positions = {}


# ============================================================
# READ JUNCTION POSITIONS
# ============================================================

for junction in root.findall("junction"):

    # Get the junction ID
    junction_id = junction.get("id")


    # Ignore SUMO's internal junction objects.
    #
    # Internal junction IDs start with ":".
    if junction_id.startswith(":"):
        continue


    # Read x and y coordinates
    x = float(
        junction.get("x")
    )

    y = float(
        junction.get("y")
    )


    # Store the coordinates
    positions[junction_id] = (
        x,
        y
    )


# ============================================================
# 5. READ EVERY NORMAL ROAD EDGE
# ============================================================

for edge in root.findall("edge"):

    # Get the SUMO edge ID.
    #
    # Examples:
    #     E0
    #     E1
    #     E2
    #
    edge_id = edge.get("id")


    # Ignore internal SUMO edges.
    if edge_id.startswith(":"):
        continue


    # Junction where this road starts
    from_node = edge.get("from")


    # Junction where this road ends
    to_node = edge.get("to")


    # --------------------------------------------------------
    # Find the first lane belonging to this edge
    # --------------------------------------------------------

    lane = edge.find("lane")


    # If no lane exists, something is wrong with the network.
    if lane is None:

        raise ValueError(
            f"No lane found for edge {edge_id}"
        )


    # Get the road length in meters.
    length = float(
        lane.get("length")
    )


    # --------------------------------------------------------
    # Create graph entry
    # --------------------------------------------------------

    if from_node not in graph:

        graph[from_node] = {}


    # --------------------------------------------------------
    # Store road information
    # --------------------------------------------------------

    graph[from_node][to_node] = {
        "edge_id": edge_id,
        "length": length
    }


# ============================================================
# 6. GET NEIGHBORS
# ============================================================

def get_neighbors(node):
    """
    Return all junctions directly reachable from 'node'.
    """

    return graph.get(
        node,
        {}
    )


# ============================================================
# 7. GET ROAD COST
# ============================================================

def get_edge_cost(
    from_node,
    to_node
):
    """
    Return the static cost of travelling between
    two connected junctions.

    For the basic planner, the cost is road length.
    """

    # Make sure the starting junction exists
    if from_node not in graph:

        raise ValueError(
            f"Unknown junction: {from_node}"
        )


    # Make sure the road exists
    if to_node not in graph[from_node]:

        raise ValueError(
            f"No road from "
            f"{from_node} to {to_node}"
        )


    # Get road information
    road = graph[from_node][to_node]


    # Static cost = road length
    return road["length"]


# ============================================================
# 8. SIMPLE PATH SEARCH
# ============================================================

def find_path(
    start,
    goal
):
    """
    Find a basic path from start to goal.

    This is a simple breadth-first style search.
    It is mainly useful for testing the network.

    The actual optimized routing is handled by
    path_planner.py using A*.
    """

    # --------------------------------------------------------
    # Validate start and goal
    # --------------------------------------------------------

    if start not in positions:

        raise ValueError(
            f"Start junction '{start}' "
            f"does not exist."
        )


    if goal not in positions:

        raise ValueError(
            f"Goal junction '{goal}' "
            f"does not exist."
        )


    # --------------------------------------------------------
    # Nodes waiting to be explored
    # --------------------------------------------------------

    open_nodes = [
        start
    ]


    # --------------------------------------------------------
    # Store how every node was reached
    # --------------------------------------------------------

    previous = {
        start: None
    }


    # ========================================================
    # SEARCH
    # ========================================================

    while open_nodes:

        # Remove the first node
        current = open_nodes.pop(0)


        # ----------------------------------------------------
        # Destination reached
        # ----------------------------------------------------

        if current == goal:

            break


        # ----------------------------------------------------
        # Explore connected junctions
        # ----------------------------------------------------

        for neighbor in get_neighbors(
            current
        ):

            # Only visit a junction the first time
            # we encounter it.
            if neighbor not in previous:

                previous[neighbor] = current

                open_nodes.append(
                    neighbor
                )


    # ========================================================
    # CHECK WHETHER GOAL WAS REACHED
    # ========================================================

    if goal not in previous:

        return []


    # ========================================================
    # RECONSTRUCT PATH
    # ========================================================

    path = []

    current = goal


    # Work backwards from goal
    while current is not None:

        path.append(
            current
        )

        current = previous[current]


    # Reverse to get:
    #
    # Start -> Goal
    #
    path.reverse()


    return path


# ============================================================
# 9. TEST THE NETWORK READER
# ============================================================

# IMPORTANT:
#
# This section runs ONLY when you execute:
#
#     python3 controller/network_reader.py
#
# It does NOT run when path_planner.py imports this file.

if __name__ == "__main__":

    print()
    print(
        "ROAD NETWORK"
    )
    print()


    # --------------------------------------------------------
    # Display every road
    # --------------------------------------------------------

    for from_node in graph:

        for to_node in graph[from_node]:

            road = graph[from_node][to_node]


            print(
                from_node,
                "->",
                to_node,
                "|",
                road["edge_id"],
                "|",
                road["length"],
                "m"
            )


    # --------------------------------------------------------
    # Test basic path search
    # --------------------------------------------------------

    print()
    print(
        "TEST PATH"
    )
    print()


    start = "J0"
    goal = "J2"


    path = find_path(
        start,
        goal
    )


    print(
        "Start:",
        start
    )

    print(
        "Goal :",
        goal
    )

    print(
        "Path :",
        path
    )