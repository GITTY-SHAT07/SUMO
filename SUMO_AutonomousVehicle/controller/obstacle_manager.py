# ============================================================
# OBSTACLE MANAGER
# ============================================================

# Store the SUMO edge IDs that are currently blocked.
#
# Example:
#
# blocked_edges = {"E2"}
#
# means E2 cannot be used by the planner.
blocked_edges = set()


# ============================================================
# BLOCK AN EDGE
# ============================================================

def block_edge(edge_id):
    """
    Mark a road as blocked.
    """

    blocked_edges.add(edge_id)

    print(
        f"Obstacle detected: {edge_id} is BLOCKED"
    )


# ============================================================
# UNBLOCK AN EDGE
# ============================================================

def unblock_edge(edge_id):
    """
    Remove an edge from the blocked-road list.
    """

    blocked_edges.discard(edge_id)

    print(
        f"Obstacle removed: {edge_id} is AVAILABLE"
    )


# ============================================================
# CHECK WHETHER AN EDGE IS BLOCKED
# ============================================================

def is_blocked(edge_id):
    """
    Return True if the road is currently blocked.
    """

    return edge_id in blocked_edges


# ============================================================
# GET ALL BLOCKED EDGES
# ============================================================

def get_blocked_edges():
    """
    Return a copy of all currently blocked roads.
    """

    return blocked_edges.copy()