import traci


# ============================================================
# TRAFFIC COST SETTINGS
# ============================================================

# Additional cost added for every vehicle on the road.
#
# Example:
#   0 vehicles -> 0 penalty
#   3 vehicles -> 6 penalty
#   5 vehicles -> 10 penalty
#
# We keep this separate so it can easily be tuned later.
TRAFFIC_PENALTY_PER_VEHICLE = 2.0


# ============================================================
# TRAFFIC-AWARE EDGE COST
# ============================================================

def get_traffic_cost(edge_id):
    """
    Calculate the traffic-aware cost of travelling through
    one SUMO road.

    Final cost:

        travel time + congestion penalty

    Lower cost = better road
    Higher cost = more congested/expensive road
    """

    # ========================================================
    # GET CURRENT TRAFFIC
    # ========================================================

    # Number of vehicles currently on this edge
    vehicle_count = (
        traci.edge.getLastStepVehicleNumber(
            edge_id
        )
    )

    # Average speed of vehicles currently on this edge
    average_speed = (
        traci.edge.getLastStepMeanSpeed(
            edge_id
        )
    )


    # ========================================================
    # GET ROAD LENGTH
    # ========================================================

    # Your network currently has one lane per edge.
    # Therefore lane 0 represents the road.
    lane_id = f"{edge_id}_0"

    road_length = (
        traci.lane.getLength(
            lane_id
        )
    )


    # ========================================================
    # PROTECT AGAINST ZERO SPEED
    # ========================================================

    # If vehicles are stopped, SUMO can report a speed of 0.
    #
    # We use a small value to avoid division by zero.
    if average_speed <= 0:

        average_speed = 0.1


    # ========================================================
    # ESTIMATED TRAVEL TIME
    # ========================================================

    # Basic equation:
    #
    #       travel time = distance / speed
    #
    travel_time = (
        road_length / average_speed
    )


    # ========================================================
    # CONGESTION PENALTY
    # ========================================================

    # More vehicles -> higher cost
    traffic_penalty = (
        vehicle_count
        * TRAFFIC_PENALTY_PER_VEHICLE
    )


    # ========================================================
    # FINAL COST
    # ========================================================

    total_cost = (
        travel_time
        + traffic_penalty
    )


    return total_cost


# ============================================================
# TRAFFIC INFORMATION
# ============================================================

def get_traffic_info(edge_id):
    """
    Return detailed traffic information for one road.

    Useful for debugging, logging, and later visualization.
    """

    # Number of vehicles
    vehicle_count = (
        traci.edge.getLastStepVehicleNumber(
            edge_id
        )
    )


    # Average speed
    average_speed = (
        traci.edge.getLastStepMeanSpeed(
            edge_id
        )
    )


    # Road length
    road_length = (
        traci.lane.getLength(
            f"{edge_id}_0"
        )
    )


    # Calculate final traffic-aware cost
    cost = get_traffic_cost(
        edge_id
    )


    return {
        "vehicles": vehicle_count,
        "average_speed": average_speed,
        "length": road_length,
        "cost": cost
    }