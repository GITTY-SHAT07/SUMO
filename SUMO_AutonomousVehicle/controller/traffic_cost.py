"""Traffic-aware edge travel-time cost for SUMO."""

import math
import traci

DEFAULT_SPEED_LIMIT = 13.89
MIN_SPEED_MPS = 0.5
MIN_SPEED_FRACTION = 0.10


def _positive_float(value, fallback):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(fallback)

    if not math.isfinite(value) or value <= 0:
        return float(fallback)

    return value


def get_traffic_info(edge_id, road_length=None, speed_limit=None):
    """Return traffic and travel-time information for a SUMO edge."""

    lane_id = f"{edge_id}_0"

    if road_length is None:
        try:
            road_length = traci.lane.getLength(lane_id)
        except Exception:
            road_length = 1.0

    road_length = _positive_float(road_length, 1.0)

    if speed_limit is None:
        try:
            speed_limit = traci.lane.getMaxSpeed(lane_id)
        except Exception:
            speed_limit = DEFAULT_SPEED_LIMIT

    speed_limit = _positive_float(speed_limit, DEFAULT_SPEED_LIMIT)

    vehicle_count = int(traci.edge.getLastStepVehicleNumber(edge_id))
    mean_speed = float(traci.edge.getLastStepMeanSpeed(edge_id))

    if vehicle_count <= 0:
        effective_speed = speed_limit
    elif not math.isfinite(mean_speed) or mean_speed <= 0:
        effective_speed = max(
            MIN_SPEED_MPS,
            speed_limit * MIN_SPEED_FRACTION
        )
    else:
        effective_speed = min(
            speed_limit,
            max(MIN_SPEED_MPS, mean_speed)
        )

    free_flow_time = road_length / speed_limit
    estimated_travel_time = road_length / effective_speed

    return {
        "edge_id": edge_id,
        "vehicle_count": vehicle_count,
        "road_length_m": road_length,
        "speed_limit_mps": speed_limit,
        "mean_speed_mps": mean_speed,
        "effective_speed_mps": effective_speed,
        "free_flow_time_s": free_flow_time,
        "estimated_travel_time_s": estimated_travel_time,
        "congestion_delay_s": max(
            0.0,
            estimated_travel_time - free_flow_time
        )
    }


def get_traffic_cost(edge_id, road_length=None, speed_limit=None):
    """Return estimated edge travel time in seconds."""

    info = get_traffic_info(
        edge_id,
        road_length=road_length,
        speed_limit=speed_limit
    )

    return float(info["estimated_travel_time_s"])
