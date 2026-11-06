"""SUMO simulation controller for synthetic and OSM road-network scenarios.

Default behavior remains the original synthetic network.

For the OSM road-closure experiment, run with:
SUMO_SCENARIO=large SUMO_TEST_CLOSURE=1

For the OSM graded-hazard experiment, run with:
SUMO_SCENARIO=large SUMO_TEST_HAZARD=1

The initial route-cost diagnostic is enabled by default in large mode.
Disable it with SUMO_DIAGNOSE_ROUTE_COSTS=0.
"""

import os
import math
import time
import xml.etree.ElementTree as ET

import traci

# ============================================================
# 1. PROJECT AND SCENARIO CONFIGURATION
# ============================================================

PROJECT_DIR = os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))
)

SCENARIO = os.environ.get(
    "SUMO_SCENARIO", "synthetic"
).strip().lower()

if SCENARIO not in {"synthetic", "large"}:
    raise ValueError(f"Unknown SUMO_SCENARIO: {SCENARIO!r}")

# ============================================================
# PLANNER MODE AND EXPERIMENT SEED
# ============================================================

PLANNER_MODE = os.environ.get(
    "SUMO_PLANNER", "risk"
).strip().lower()

VALID_PLANNER_MODES = {
    "shortest",
    "traffic",
    "risk"
}

MODEL_DT = 0.1

TRAFFIC_WARMUP_SECONDS = max(
    0.0,
    float(os.environ.get("SUMO_TRAFFIC_WARMUP_S", "0"))
)

if PLANNER_MODE not in VALID_PLANNER_MODES:
    raise ValueError(
        f"Invalid planner mode: {PLANNER_MODE}. "
        "Choose shortest, traffic, or risk."
    )

SUMO_SEED = int(
    os.environ.get("SUMO_SEED", "42")
)

if SUMO_SEED < 0:
    raise ValueError("SUMO_SEED must be zero or greater.")

if SCENARIO == "large":

    SUMO_CFG = os.path.join(
        PROJECT_DIR,
        "simulation",
        "simulation_large.sumocfg"
    )

    NETWORK_FILE = os.path.join(
        PROJECT_DIR,
        "map.net.xml"
    )

    EGO_ID = "largeMapEgo"
    START_JUNCTION = None
    GOAL_JUNCTION = "2227588809"

    MAX_SIMULATION_TIME = 300.0

    OBSTACLE_EDGE = None
    OBSTACLE_TIME = float("inf")

else:

    SUMO_CFG = os.path.join(
        PROJECT_DIR,
        "simulation",
        "simulation.sumocfg"
    )

    NETWORK_FILE = os.path.join(
        PROJECT_DIR,
        "network",
        "network.net.xml"
    )

    EGO_ID = "egoVehicle"
    START_JUNCTION = "J0"
    GOAL_JUNCTION = "J2"

    MAX_SIMULATION_TIME = 100.0

    OBSTACLE_EDGE = "E2"
    OBSTACLE_TIME = 45.0

# ============================================================
# RUNTIME CONFIGURATION (ENVIRONMENT OVERRIDES)
# ============================================================

def resolve_project_path(value, fallback):
    """Resolve a configured path relative to the project root."""
    selected = value or fallback
    if os.path.isabs(selected):
        return os.path.abspath(selected)
    return os.path.abspath(os.path.join(PROJECT_DIR, selected))


SUMO_CFG = resolve_project_path(
    os.environ.get("SUMO_CONFIG_FILE"), SUMO_CFG
)
NETWORK_FILE = resolve_project_path(
    os.environ.get("SUMO_NETWORK_FILE"), NETWORK_FILE
)
EGO_ID = os.environ.get("SUMO_EGO_ID", EGO_ID).strip()
GOAL_JUNCTION = os.environ.get("SUMO_GOAL_JUNCTION", GOAL_JUNCTION).strip()
START_EDGE_OVERRIDE = os.environ.get("SUMO_START_EDGE", "").strip() or None
SUMO_VEHICLE_CLASS = os.environ.get("SUMO_VEHICLE_CLASS", "passenger").strip()
SUMO_VEHICLE_TYPE = os.environ.get("SUMO_VEHICLE_TYPE", "DEFAULT_VEHTYPE").strip()
SUMO_MAX_TIME = float(os.environ.get("SUMO_MAX_SIM_TIME_S", str(MAX_SIMULATION_TIME)))
MODEL_DT = float(os.environ.get("SUMO_MODEL_DT_S", str(MODEL_DT)))
TARGET_SPEED = float(os.environ.get("SUMO_TARGET_SPEED_MPS", "6.0"))
GOAL_DISTANCE_THRESHOLD = float(os.environ.get("SUMO_GOAL_TOLERANCE_M", "3.0"))
REPLAN_INTERVAL = float(os.environ.get("SUMO_REPLAN_INTERVAL_S", "10.0"))
DEBUG_DIAGNOSTICS = os.environ.get("SUMO_DEBUG_DIAGNOSTICS", "0").strip().lower() in {"1", "true", "yes"}
STATE_PUBLISH_INTERVAL = max(0.1, float(os.environ.get("SUMO_STATE_PUBLISH_INTERVAL_S", "0.5")))

if not EGO_ID or not GOAL_JUNCTION or not SUMO_VEHICLE_CLASS:
    raise ValueError("SUMO_EGO_ID, SUMO_GOAL_JUNCTION and SUMO_VEHICLE_CLASS must be non-empty.")
if SUMO_MAX_TIME <= 0 or MODEL_DT <= 0 or TARGET_SPEED <= 0 or GOAL_DISTANCE_THRESHOLD <= 0 or REPLAN_INTERVAL <= 0:
    raise ValueError("Simulation time, timestep, target speed, goal tolerance and replanning interval must be positive.")
if not os.path.isfile(NETWORK_FILE):
    raise FileNotFoundError(f"SUMO network file not found: {NETWORK_FILE}")
if not os.path.isfile(SUMO_CFG):
    raise FileNotFoundError(f"SUMO configuration not found: {SUMO_CFG}")

# Fail early if the controller graph and SUMO simulation use different networks.
_sumo_config_root = ET.parse(SUMO_CFG).getroot()
_net_file_node = _sumo_config_root.find("./input/net-file")
if _net_file_node is None or not _net_file_node.get("value"):
    raise ValueError(f"SUMO config has no input/net-file value: {SUMO_CFG}")
_config_network_value = _net_file_node.get("value")
_config_network_path = (
    _config_network_value
    if os.path.isabs(_config_network_value)
    else os.path.join(os.path.dirname(SUMO_CFG), _config_network_value)
)
_config_network_path = os.path.abspath(_config_network_path)
if _config_network_path != os.path.abspath(NETWORK_FILE):
    raise ValueError(
        "Controller/SUMO network mismatch. "
        f"SUMO_NETWORK_FILE resolves to {NETWORK_FILE}, but "
        f"{SUMO_CFG} loads {_config_network_path}."
    )

_route_files_node = _sumo_config_root.find("./input/route-files")
if _route_files_node is not None and _route_files_node.get("value"):
    for _route_file_value in _route_files_node.get("value").split(","):
        _route_file_value = _route_file_value.strip()
        if not _route_file_value:
            continue
        _route_file_path = (
            _route_file_value
            if os.path.isabs(_route_file_value)
            else os.path.join(os.path.dirname(SUMO_CFG), _route_file_value)
        )
        if not os.path.isfile(_route_file_path):
            raise FileNotFoundError(
                f"Route file referenced by {SUMO_CFG} was not found: {_route_file_path}"
            )

if START_EDGE_OVERRIDE and START_EDGE_OVERRIDE.startswith(":"):
    raise ValueError("SUMO_START_EDGE must be a normal road edge, not an internal connector.")

# Preserve the network's georeferencing metadata for the visualizer/exporter.
_network_xml_root = ET.parse(NETWORK_FILE).getroot()
_location_element = _network_xml_root.find("location")
NETWORK_LOCATION_METADATA = (
    dict(_location_element.attrib) if _location_element is not None else {}
)
MAX_SIMULATION_TIME = SUMO_MAX_TIME

# ============================================================
# OSM CLOSURE EXPERIMENT SETTINGS
# ============================================================

TEST_OSM_CLOSURE = (
    SCENARIO == "large"
    and os.environ.get(
        "SUMO_TEST_CLOSURE", "0"
    ).strip().lower() in {"1", "true", "yes"}
)

OSM_CLOSURE_TIME = float(os.environ.get("SUMO_CLOSURE_TIME_S", "45.0"))
OSM_CLOSURE_RETRY_INTERVAL = float(os.environ.get("SUMO_CLOSURE_RETRY_INTERVAL_S", "5.0"))
OSM_CLOSURE_LOOKAHEAD = int(os.environ.get("SUMO_CLOSURE_LOOKAHEAD_EDGES", "10"))

# ============================================================
# RISK-AWARE EXPERIMENT SETTINGS
# ============================================================

TEST_OSM_HAZARD = (
    SCENARIO == "large"
    and os.environ.get(
        "SUMO_TEST_HAZARD", "0"
    ).strip().lower() in {"1", "true", "yes"}
)

# Fixed-edge comparison mode: the same map hazard is presented to every planner.
TEST_OSM_HAZARD_COMPARISON = (
    SCENARIO == "large"
    and os.environ.get(
        "SUMO_TEST_HAZARD_COMPARISON", "0"
    ).strip().lower() in {"1", "true", "yes"}
)

HAZARD_COMPARISON_EDGE = os.environ.get(
    "SUMO_HAZARD_COMPARISON_EDGE", "1513939046#2"
).strip()

if sum((TEST_OSM_CLOSURE, TEST_OSM_HAZARD, TEST_OSM_HAZARD_COMPARISON)) > 1:
    raise ValueError(
        "Run closure, risk-only hazard, and comparative hazard experiments separately."
    )

if TEST_OSM_HAZARD and PLANNER_MODE != "risk":
    raise ValueError(
        "SUMO_TEST_HAZARD=1 requires SUMO_PLANNER=risk."
    )

HAZARD_TIME = float(os.environ.get("SUMO_HAZARD_TIME_S", "45.0"))
HAZARD_SEVERITY = int(os.environ.get("SUMO_HAZARD_SEVERITY", "3"))
HAZARD_LOOKAHEAD_EDGES = int(os.environ.get("SUMO_HAZARD_LOOKAHEAD_EDGES", "10"))
HAZARD_RETRY_INTERVAL = float(os.environ.get("SUMO_HAZARD_RETRY_INTERVAL_S", "5.0"))

if not 0 <= HAZARD_SEVERITY <= 3:
    raise ValueError("SUMO_HAZARD_SEVERITY must be between 0 and 3")
if HAZARD_TIME < 0 or HAZARD_RETRY_INTERVAL <= 0 or HAZARD_LOOKAHEAD_EDGES < 1:
    raise ValueError("Hazard time must be non-negative; retry interval and lookahead must be positive")
if OSM_CLOSURE_TIME < 0 or OSM_CLOSURE_RETRY_INTERVAL <= 0 or OSM_CLOSURE_LOOKAHEAD < 1:
    raise ValueError("Closure time must be non-negative; retry interval and lookahead must be positive")

# Initial route-cost diagnostic. Enabled by default for large-map runs.
# Disable with: SUMO_DIAGNOSE_ROUTE_COSTS=0
DIAGNOSE_INITIAL_ROUTE_COSTS = (
    SCENARIO == "large"
    and os.environ.get(
        "SUMO_DIAGNOSE_ROUTE_COSTS", "1"
    ).strip().lower() in {"1", "true", "yes"}
)

if not os.path.isfile(NETWORK_FILE):
    raise FileNotFoundError(
        f"Network file not found: {NETWORK_FILE}"
    )

if not os.path.isfile(SUMO_CFG):
    raise FileNotFoundError(
        f"SUMO configuration not found: {SUMO_CFG}"
    )

# Select the network before importing network_reader.
os.environ["SUMO_NETWORK_FILE"] = NETWORK_FILE
os.environ["SUMO_VEHICLE_CLASS"] = SUMO_VEHICLE_CLASS

# ============================================================
# 2. PROJECT MODULES
# ============================================================

from network_reader import (
    graph,
    road_options,
    edge_data,
    positions,
    get_edge_successors,
    is_legal_edge_transition,
)

from traffic_cost import get_traffic_cost

from path_planner import (
    plan_path,
    junctions_to_edges
)

from obstacle_manager import (
    block_edge,
    unblock_edge,
    get_blocked_edges,
)

from path_tracker import (
    calculate_tracking_error,
    reset_tracker
)

from hazard_manager import (
    get_hazard_cost,
    set_hazard,
    clear_hazard,
    get_active_hazards,
)

from Steering_controller import calculate_steering_angle

from vehicle_model import BicycleModel
from telemetry_logger import TelemetryLogger
from engine_interface import EngineInterface

# ============================================================
# 3. CONTROLLER PARAMETERS AND STATE
# ============================================================

WHEELBASE = float(os.environ.get("SUMO_WHEELBASE_M", "2.5"))

reference_path = []
route_edges = []

vehicle_model = None

route_assigned = False
obstacle_added = False

last_replan_time = 0.0

commanded_speed = TARGET_SPEED

telemetry_logger = None
engine_interface = None
pending_replan_reason = "periodic_traffic_refresh"
last_state_publish_time = -float("inf")
last_simulation_time = 0.0
last_console_print_time = -float("inf")
CONSOLE_LOG_INTERVAL = max(0.1, float(os.environ.get("SUMO_CONSOLE_LOG_INTERVAL_S", "1.0")))
route_cost = None

# OSM closure experiment state.
next_osm_closure_attempt = OSM_CLOSURE_TIME

osm_closure_edge = None
osm_closure_detected_time = None
osm_closure_reroute_time = None
osm_closure_wall_started = None

# OSM hazard experiment state.
risk_hazard_added = False
risk_hazard_edge = None
risk_hazard_rerouted = False

risk_hazard_detected_time = None
risk_hazard_reroute_time = None

risk_hazard_baseline_cost = None
risk_hazard_alternative_cost = None

risk_hazard_wall_started = None

# Fixed-edge, three-planner hazard comparison state.
hazard_comparison_active = False
hazard_comparison_replan_checked = False
hazard_comparison_initial_exposure = None
hazard_comparison_replanned_exposure = None
hazard_comparison_result = "NOT_RUN"
hazard_comparison_activation_time = None

last_hazard_attempt_time = -HAZARD_RETRY_INTERVAL

goal_reached_flag = False

if SCENARIO == "large":
    if GOAL_JUNCTION not in positions:
        raise ValueError(
            f"SUMO_GOAL_JUNCTION {GOAL_JUNCTION!r} is not present in {NETWORK_FILE}."
        )
    if START_EDGE_OVERRIDE and START_EDGE_OVERRIDE not in edge_data:
        raise ValueError(
            f"SUMO_START_EDGE {START_EDGE_OVERRIDE!r} is not present in {NETWORK_FILE}."
        )
    if START_EDGE_OVERRIDE and not edge_data[START_EDGE_OVERRIDE].get("passenger_allowed", False):
        raise ValueError(
            f"SUMO_START_EDGE {START_EDGE_OVERRIDE!r} does not allow vehicle class {SUMO_VEHICLE_CLASS!r}."
        )

EDGE_RECORDS = {
    road["edge_id"]: road
    for destinations in road_options.values()
    for roads in destinations.values()
    for road in roads
}

# ============================================================
# 4. COMPLETE ROUTING COST
# ============================================================

def get_traffic_planning_cost(edge_id):
    """Estimate edge traversal time using observed traffic speed."""

    road = EDGE_RECORDS.get(edge_id)

    if SCENARIO == "large" and road is not None:
        return get_traffic_cost(
            edge_id,
            road_length=float(road["length"]),
            speed_limit=road.get("speed")
        )

    return get_traffic_cost(edge_id)


def get_risk_planning_cost(edge_id):
    """Traffic cost + graded hazard penalty."""

    return (
        get_traffic_planning_cost(edge_id)
        + get_hazard_cost(edge_id)
    )


def get_active_cost_function():
    """Return the edge-cost function selected by SUMO_PLANNER."""

    if PLANNER_MODE == "shortest":
        return None

    elif PLANNER_MODE == "traffic":
        return get_traffic_planning_cost

    else:
        return get_risk_planning_cost


# These are the two complete candidate routes found by the K-route
# diagnostic from the project start junction to GOAL_JUNCTION.
# The currently occupied edge is intentionally omitted because
# plan_path(..., start_edge=current_edge) plans only the remaining roads.

DIAGNOSTIC_MAIN_ROUTE = [
    "-1513939046#1",
    "-1513939046#0",
    "1084319629#2",
    "1084319629#3",
    "1084319629#4",
    "1084319629#5",
    "1084319629#6",
]

DIAGNOSTIC_ALTERNATIVE_ROUTE = [
    "1513939046#2",
    "-784746944#3",
    "-784746944#2",
    "-784746944#1",
    "-784746944#0",
    "-1153978413",
    "-1513939040",
    "1416241810#4",
    "1416241810#5",
    "1416241810#6",
    "1416241810#7",
    "1512946149",
    "974129255",
    "-212983375",
]


def check_route_connectivity(start_edge, candidate_edges):
    """Check every candidate edge is a legal successor of the previous edge."""

    previous_edge = start_edge

    for candidate_edge in candidate_edges:
        legal_successors = get_edge_successors(previous_edge)

        if candidate_edge not in legal_successors:
            return False, previous_edge, candidate_edge

        previous_edge = candidate_edge

    return True, None, None


def inspect_route_cost(label, candidate_edges, start_edge):
    """Print a route's point-in-time traffic cost, edge by edge."""

    print(f"\n--- {label} ---")

    connected, from_edge, to_edge = check_route_connectivity(
        start_edge,
        candidate_edges
    )

    if connected:
        print("Legal successor sequence from current edge: YES")
    else:
        print(
            "Legal successor sequence from current edge: NO | "
            f"invalid transition: {from_edge} -> {to_edge}"
        )

    total_cost = 0.0
    total_distance = 0.0
    total_free_flow_time = 0.0
    missing_edges = []

    for edge_id in candidate_edges:
        road = EDGE_RECORDS.get(edge_id)

        if road is None:
            print(f"{edge_id}: NOT FOUND in EDGE_RECORDS")
            missing_edges.append(edge_id)
            continue

        try:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
            mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
            edge_cost = get_traffic_planning_cost(edge_id)

            length = float(road["length"])
            speed_limit = float(road.get("speed", 13.89))

            if speed_limit <= 0:
                speed_limit = 13.89

            free_flow_time = length / speed_limit

            total_cost += float(edge_cost)
            total_distance += length
            total_free_flow_time += free_flow_time

            print(
                f"{edge_id}: vehicles={vehicle_count}, "
                f"mean_speed={mean_speed:.2f} m/s, "
                f"free_flow={free_flow_time:.2f} s, "
                f"traffic_cost={edge_cost:.2f} s"
            )

        except (
            traci.TraCIException,
            KeyError,
            TypeError,
            ValueError
        ) as error:
            print(f"{edge_id}: cost could not be evaluated: {error}")
            missing_edges.append(edge_id)

    if missing_edges:
        print(
            "ROUTE COST: INCOMPLETE - "
            "one or more edges could not be scored"
        )
        return None

    print(f"{label} distance: {total_distance:.2f} m")
    print(f"{label} free-flow time: {total_free_flow_time:.2f} s")
    print(f"{label} traffic cost: {total_cost:.2f} s")

    if not connected:
        print(
            "WARNING: The edge costs were calculated, but this candidate "
            "does not connect legally from the ego vehicle's current edge."
        )

    return total_cost


def inspect_initial_route_costs(current_edge, current_junction):
    """Compare the known main and detour routes at the initial A* decision."""

    print("\n========== INITIAL ROUTE-COST DIAGNOSTIC ==========")
    print(f"Simulation time: {traci.simulation.getTime():.1f} s")
    print(f"Ego current edge: {current_edge}")
    print(f"Ego next junction: {current_junction}")
    print("Cost model: current traffic-cost function, point-in-time snapshot")

    main_cost = inspect_route_cost(
        "MAIN ROUTE (K-ROUTE 1)",
        DIAGNOSTIC_MAIN_ROUTE,
        current_edge
    )

    alternative_cost = inspect_route_cost(
        "ALTERNATIVE ROUTE (K-ROUTE 4)",
        DIAGNOSTIC_ALTERNATIVE_ROUTE,
        current_edge
    )

    print("\n---------- INITIAL COMPARISON ----------")

    if main_cost is None or alternative_cost is None:
        print(
            "Comparison unavailable because at least one route "
            "cost is incomplete."
        )

    elif alternative_cost < main_cost:
        print(
            "ALTERNATIVE ROUTE IS CHEAPER by "
            f"{main_cost - alternative_cost:.2f} s."
        )

    elif main_cost < alternative_cost:
        print(
            "MAIN ROUTE IS CHEAPER by "
            f"{alternative_cost - main_cost:.2f} s."
        )

    else:
        print("Both routes have equal traffic cost.")

    print("===================================================\n")


print(f"[SCENARIO] {SCENARIO}")
print(f"[PLANNER] {PLANNER_MODE}")
print(f"[SUMO SEED] {SUMO_SEED}")
print(f"[SCENARIO] SUMO configuration: {SUMO_CFG}")
print(f"[SCENARIO] Network: {NETWORK_FILE}")

if TEST_OSM_CLOSURE:

    print("[OSM CLOSURE TEST] ENABLED")

    print(
        f"[OSM CLOSURE TEST] First attempt at "
        f"{OSM_CLOSURE_TIME:.1f}s"
    )

if TEST_OSM_HAZARD:

    print("[OSM RISK TEST] ENABLED")

    print(
        f"[OSM RISK TEST] First attempt at {HAZARD_TIME:.1f}s; "
        f"severity {HAZARD_SEVERITY}/3"
    )

# ============================================================
# 5. ANGLE HELPERS
# ============================================================

def normalize_angle(angle):
    """Normalize the angle to the range -180 to +180 degrees."""

    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


def convert_sumo_angle(sumo_angle):
    # Convert SUMO heading to Cartesian heading.
    # SUMO: 0 = north, 90 = east.
    # Cartesian: 0 = east, 90 = north.

    return normalize_angle(90.0 - sumo_angle)


def convert_cartesian_angle_to_sumo(cartesian_angle):
    """Convert Cartesian heading back to SUMO heading."""

    return normalize_angle(90.0 - cartesian_angle)

# ============================================================
# 6. REFERENCE PATH GEOMETRY
# ============================================================

def get_route_path_points(route_edges):
    """Build the reference path using road and internal connector geometry."""

    path_points = []

    root = ET.parse(NETWORK_FILE).getroot()

    lane_shapes = {}

    for lane in root.iter("lane"):

        lane_id = lane.get("id")
        shape_text = lane.get("shape")

        if not lane_id or not shape_text:
            continue

        points = []

        for coordinate in shape_text.split():

            x_text, y_text = coordinate.split(",")

            points.append((
                float(x_text),
                float(y_text)
            ))

        lane_shapes[lane_id] = points

    # Map each ordered road-edge pair to its internal connector lane.
    connection_vias = {}

    for connection in root.findall("connection"):

        from_edge = connection.get("from")
        to_edge = connection.get("to")
        via_lane = connection.get("via")

        if from_edge and to_edge and via_lane:
            connection_vias[(from_edge, to_edge)] = via_lane

    def append_shape(shape):
        """Append points in order while avoiding duplicate adjacent points."""

        for point in shape:

            point = (
                float(point[0]),
                float(point[1])
            )

            if not path_points:
                path_points.append(point)
                continue

            previous = path_points[-1]

            gap = math.hypot(
                point[0] - previous[0],
                point[1] - previous[1]
            )

            if gap > 0.001:
                path_points.append(point)

    for index, edge_id in enumerate(route_edges):

        if edge_id.startswith(":"):
            continue

        lane_id = f"{edge_id}_0"

        lane_shape = lane_shapes.get(lane_id)

        if lane_shape:

            append_shape(lane_shape)

        else:

            try:

                append_shape(
                    traci.lane.getShape(lane_id)
                )

            except traci.TraCIException:

                print(
                    f"Warning: Could not read lane shape for {lane_id}"
                )

                continue

        if index < len(route_edges) - 1:

            next_edge_id = route_edges[index + 1]

            via_lane = connection_vias.get(
                (edge_id, next_edge_id)
            )

            if via_lane:

                connector_shape = lane_shapes.get(via_lane)

                if connector_shape:

                    append_shape(connector_shape)

                else:

                    print(
                        f"Warning: Connector lane {via_lane} "
                        f"has no geometry for "
                        f"{edge_id} -> {next_edge_id}"
                    )

            else:

                print(
                    f"Warning: No internal connector found for "
                    f"{edge_id} -> {next_edge_id}"
                )

    return path_points

# ============================================================
# 7. ROAD / JUNCTION LOOKUP
# ============================================================

def get_current_junction(edge_id):
    """Return the destination junction of a SUMO road edge."""

    road = edge_data.get(edge_id)

    if road is None:
        return None

    return road["to_node"]

# ============================================================
# 8. OSM CLOSURE CANDIDATE SELECTION
# ============================================================

def find_osm_closure_candidate(current_edge):
    """Find a future route edge whose temporary closure has a legal detour.

    Candidate edges come from the active SUMO route, at least two road
    positions ahead. Each candidate is temporarily blocked and edge-based
    A* checks whether an alternative route exists from the current edge.
    """

    if current_edge not in edge_data:
        return None

    current_junction = get_current_junction(current_edge)

    if (
        current_junction is None
        or current_junction == GOAL_JUNCTION
    ):
        return None

    current_route = list(
        traci.vehicle.getRoute(EGO_ID)
    )

    route_index = traci.vehicle.getRouteIndex(EGO_ID)

    if route_index < 0 or route_index >= len(current_route):
        return None

    first_index = route_index + 2

    last_index = min(
        len(current_route),
        route_index + OSM_CLOSURE_LOOKAHEAD + 1
    )

    candidates = current_route[first_index:last_index]

    for candidate_edge in candidates:

        if (
            candidate_edge.startswith(":")
            or candidate_edge not in edge_data
            or candidate_edge == current_edge
        ):
            continue

        # Temporarily block this future road to test detour feasibility.
        block_edge(candidate_edge)

        try:

            trial_path, _, trial_edges = plan_path(
                current_junction,
                GOAL_JUNCTION,
                edge_cost_function=get_active_cost_function(),
                return_edges=True,
                start_edge=current_edge
            )

        finally:

            # Always restore the candidate after the feasibility test.
            unblock_edge(candidate_edge)

        if (
            trial_path
            and candidate_edge not in trial_edges
        ):

            print(
                "[OSM CLOSURE TEST] Detour pre-check passed for:",
                candidate_edge
            )

            return candidate_edge

    return None

# ============================================================
# 9. RISK-AWARE HAZARD CANDIDATE SELECTION
# ============================================================

def find_risk_avoidance_candidate(current_edge):
    """Find a future route edge that A* avoids when it becomes hazardous.

    The edge remains open. A temporary severity penalty is applied during
    this pre-check, and removed immediately after the trial plan completes.
    The current edge is supplied to edge-based A* so only legal next roads
    can be selected from the vehicle's actual position.
    """

    if current_edge not in edge_data:
        return None

    current_junction = get_current_junction(current_edge)

    if (
        current_junction is None
        or current_junction == GOAL_JUNCTION
    ):
        return None

    current_route = list(
        traci.vehicle.getRoute(EGO_ID)
    )

    route_index = traci.vehicle.getRouteIndex(EGO_ID)

    if route_index < 0 or route_index >= len(current_route):
        return None

    candidates = current_route[
        route_index + 1:
        route_index + 1 + HAZARD_LOOKAHEAD_EDGES
    ]

    baseline_path, baseline_cost, baseline_edges = plan_path(
        current_junction,
        GOAL_JUNCTION,
        edge_cost_function=get_active_cost_function(),
        return_edges=True,
        start_edge=current_edge
    )

    if not baseline_path:
        return None

    for candidate_edge in candidates:

        if (
            candidate_edge.startswith(":")
            or candidate_edge not in edge_data
            or candidate_edge not in baseline_edges
        ):
            continue

        # Temporarily apply a high-risk penalty. The road is NOT blocked.
        set_hazard(
            candidate_edge,
            HAZARD_SEVERITY,
            "Temporary risk-candidate feasibility check"
        )

        try:

            risk_path, risk_cost, risk_edges = plan_path(
                current_junction,
                GOAL_JUNCTION,
                edge_cost_function=get_active_cost_function(),
                return_edges=True,
                start_edge=current_edge
            )

        finally:

            clear_hazard(candidate_edge)

        # We want a candidate where the risk penalty changes the route.
        if risk_path and candidate_edge not in risk_edges:

            print(
                "[RISK TEST] Alternative-route pre-check passed for:",
                candidate_edge
            )

            return (
                candidate_edge,
                baseline_cost,
                risk_cost
            )

    return None

# ============================================================
# 10. SPEED CONTROL
# ============================================================

def calculate_model_speed(steering_angle):
    """
    Reduce speed during sharper turns.

        |steering| <= 10°  -> 6.0 m/s
        |steering| <= 18°  -> 4.5 m/s
        otherwise          -> 3.0 m/s
    """

    steering_magnitude = abs(steering_angle)

    if steering_magnitude > 18:
        return 3.0

    if steering_magnitude > 10:
        return 4.5

    return TARGET_SPEED

# ============================================================
# 11. GOAL CHECK
# ============================================================

def reached_goal(model_x, model_y):
    """Check whether the vehicle has reached the end of its reference path."""

    if len(reference_path) < 1:
        return False

    goal_point = reference_path[-1]

    dx = model_x - goal_point[0]
    dy = model_y - goal_point[1]

    distance_to_goal = (dx * dx + dy * dy) ** 0.5

    return distance_to_goal <= GOAL_DISTANCE_THRESHOLD

def process_visualizer_command(command, simulation_time, current_edge=None):
    """Validate and apply one command appended by the visualization layer."""
    global GOAL_JUNCTION, last_replan_time, pending_replan_reason

    command_id = command.get("command_id")
    command_type = str(command.get("type", "")).strip().lower()
    accepted = False
    message = ""
    event_name = "visualizer_command_rejected"
    payload = {"command_id": command_id, "command_type": command_type}

    try:
        if command.get("_parse_error"):
            raise ValueError(command["_parse_error"])

        if command_type == "set_hazard":
            edge_id = str(command.get("edge_id", "")).strip()
            severity = command.get("severity")
            if edge_id not in edge_data:
                raise ValueError(f"Unknown SUMO edge: {edge_id!r}")
            if isinstance(severity, bool) or not isinstance(severity, int):
                raise ValueError("severity must be an integer from 0 to 3")
            if not 0 <= severity <= 3:
                raise ValueError("severity must be an integer from 0 to 3")
            if severity == 0:
                clear_hazard(edge_id)
                event_name = "hazard_cleared"
            else:
                set_hazard(edge_id, severity, str(command.get("reason", "Visualizer-injected hazard")))
                event_name = "hazard_activated"
            payload.update({"edge_id": edge_id, "severity": severity})
            accepted = True
            message = "Hazard command applied."

        elif command_type == "clear_hazard":
            edge_id = str(command.get("edge_id", "")).strip()
            if edge_id not in edge_data:
                raise ValueError(f"Unknown SUMO edge: {edge_id!r}")
            clear_hazard(edge_id)
            event_name = "hazard_cleared"
            payload.update({"edge_id": edge_id})
            accepted = True
            message = "Hazard cleared."

        elif command_type == "close_edge":
            edge_id = str(command.get("edge_id", "")).strip()
            if edge_id not in edge_data:
                raise ValueError(f"Unknown SUMO edge: {edge_id!r}")
            block_edge(edge_id)
            event_name = "road_closed"
            payload.update({"edge_id": edge_id, "vehicle_current_edge": current_edge})
            accepted = True
            message = "Road added to the planner's blocked-edge set."
            if edge_id == current_edge:
                message += " The ego vehicle is already on this edge; the closure affects subsequent route planning."

        elif command_type == "open_edge":
            edge_id = str(command.get("edge_id", "")).strip()
            if edge_id not in edge_data:
                raise ValueError(f"Unknown SUMO edge: {edge_id!r}")
            unblock_edge(edge_id)
            event_name = "road_reopened"
            payload.update({"edge_id": edge_id})
            accepted = True
            message = "Road removed from the planner's blocked-edge set."

        elif command_type == "set_goal":
            junction_id = str(command.get("junction_id", "")).strip()
            if junction_id not in positions:
                raise ValueError(f"Unknown SUMO junction: {junction_id!r}")
            GOAL_JUNCTION = junction_id
            event_name = "destination_changed"
            payload.update({"goal_junction": junction_id})
            accepted = True
            message = "Destination updated."

        elif command_type == "request_replan":
            accepted = True
            event_name = "replan_requested"
            message = "Immediate route evaluation requested."

        else:
            raise ValueError(f"Unsupported command type: {command_type!r}")

    except (TypeError, ValueError, KeyError) as error:
        message = str(error)
        accepted = False

    if accepted:
        last_replan_time = simulation_time - REPLAN_INTERVAL
        pending_replan_reason = f"visualizer:{command_type}"

    payload.update({"accepted": accepted, "message": message})
    if engine_interface is not None:
        engine_interface.log_event(
            event_name if accepted else "visualizer_command_rejected",
            simulation_time,
            **payload,
        )
    print(
        "[ENGINE COMMAND]",
        "ACCEPTED" if accepted else "REJECTED",
        command_type,
        message,
    )
    return accepted


# ============================================================
# 12. START SUMO
# ============================================================

from datetime import datetime, timezone

RUN_ID = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + f"_{PLANNER_MODE}_s{SUMO_SEED}"
RUNS_ROOT = resolve_project_path(
    os.environ.get("SUMO_OUTPUT_DIR"), os.path.join("results", "runs")
)
RUN_DIR = os.path.join(RUNS_ROOT, RUN_ID)
os.makedirs(RUN_DIR, exist_ok=False)

SUMO_BINARY = os.environ.get("SUMO_BINARY", "sumo")
sumo_cmd = [
    SUMO_BINARY,
    "-c",
    SUMO_CFG,
    "--seed",
    str(SUMO_SEED),
    "--tripinfo-output",
    os.path.join(RUN_DIR, "tripinfo.xml"),
    "--summary-output",
    os.path.join(RUN_DIR, "sumo_summary.xml"),
]

if SCENARIO == "large":

    sumo_cmd.extend([
        "--end",
        str(MAX_SIMULATION_TIME)
    ])

print("Starting SUMO...")

traci.start(sumo_cmd)

# A generic OSM config may provide background traffic only. In that case the
# engine can create its own ego vehicle from SUMO_START_EDGE.
if START_EDGE_OVERRIDE and EGO_ID not in traci.vehicle.getIDList():
    bootstrap_route_id = f"__{EGO_ID}_bootstrap_route"
    bootstrap_edges = [START_EDGE_OVERRIDE]
    bootstrap_successors = get_edge_successors(START_EDGE_OVERRIDE)
    if bootstrap_successors:
        bootstrap_edges.append(bootstrap_successors[0])
    traci.route.add(bootstrap_route_id, bootstrap_edges)
    traci.vehicle.add(
        EGO_ID,
        bootstrap_route_id,
        typeID=SUMO_VEHICLE_TYPE,
        depart="now",
        departLane="best",
        departSpeed="max",
    )
    # Hold the manually spawned ego at the origin until A* replaces this
    # bootstrap route with the complete destination route in the first tick.
    traci.vehicle.setSpeedMode(EGO_ID, 0)
    traci.vehicle.setSpeed(EGO_ID, 0.0)
    print(f"[SPAWN] Created {EGO_ID} on {START_EDGE_OVERRIDE}; holding for initial route planning")

print("Connected to SUMO!")
print()

# ============================================================
# 13. SIMULATION, PLANNING, CONTROL AND LOGGING
# ============================================================

try:

    os.makedirs(RUN_DIR, exist_ok=True)

    run_metadata = {
        "run_id": RUN_ID,
        "scenario": SCENARIO,
        "planner": PLANNER_MODE,
        "seed": SUMO_SEED,
        "network_file": NETWORK_FILE,
        "sumo_config": SUMO_CFG,
        "network_location": NETWORK_LOCATION_METADATA,
        "network_coordinate_system": "SUMO projected Cartesian x/y; use network_location for map georeferencing",
        "vehicle_id": EGO_ID,
        "start_edge_override": START_EDGE_OVERRIDE,
        "goal_junction": GOAL_JUNCTION,
        "vehicle_class": SUMO_VEHICLE_CLASS,
        "target_speed_mps": TARGET_SPEED,
        "model_dt_s": MODEL_DT,
        "replan_interval_s": REPLAN_INTERVAL,
        "traffic_warmup_s": TRAFFIC_WARMUP_SECONDS,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    engine_interface = EngineInterface(RUN_DIR, run_metadata)
    telemetry_logger = TelemetryLogger(os.path.join(RUN_DIR, "telemetry.csv"))
    engine_interface.log_event("simulation_connected", simulation_time=traci.simulation.getTime(), vehicle_id=EGO_ID)

    print("Building background traffic...")

    if SCENARIO == "synthetic":

        warmup_steps = int(10.0 / MODEL_DT)

        for _ in range(warmup_steps):
            traci.simulationStep()

    else:

        warmup_steps = int(
            round(TRAFFIC_WARMUP_SECONDS / MODEL_DT)
        )

        if warmup_steps > 0:

            print(
                f"Large-map traffic warm-up: "
                f"{TRAFFIC_WARMUP_SECONDS:.1f}s"
            )

            for _ in range(warmup_steps):

                # Hold the ego at its departure location while background
                # traffic enters and builds up on the network.
                if EGO_ID in traci.vehicle.getIDList():
                    try:
                        traci.vehicle.setSpeedMode(EGO_ID, 0)
                        traci.vehicle.setSpeed(EGO_ID, 0.0)
                    except traci.TraCIException:
                        pass

                traci.simulationStep()

                if EGO_ID in traci.vehicle.getIDList():
                    try:
                        traci.vehicle.setSpeedMode(EGO_ID, 0)
                        traci.vehicle.setSpeed(EGO_ID, 0.0)
                    except traci.TraCIException:
                        pass

            print("Traffic warm-up complete.")

        else:
            print("Traffic warm-up disabled.")

    print()

    # --------------------------------------------------------
    # INITIAL ROUTE PLANNING FOR SYNTHETIC SCENARIO
    # --------------------------------------------------------

    if SCENARIO == "synthetic":

        print(f"\nRunning {PLANNER_MODE}-mode A*...\n")

        path, total_cost, selected_edges = plan_path(
            START_JUNCTION,
            GOAL_JUNCTION,
            edge_cost_function=get_active_cost_function(),
            return_edges=True
        )
        route_cost = total_cost

        if not path:
            raise RuntimeError("No traffic-aware path found.")

        route_edges = junctions_to_edges(
            path,
            selected_edges
        )

        print(f"{PLANNER_MODE.capitalize()} junction path:", path)
        print("Selected SUMO route:", route_edges)

        print(
            "Selected planner total cost:",
            f"{total_cost:.2f}"
        )

    else:

        route_edges = []

        print(
            "\nLarge-map mode: planning route from "
            "the active vehicle edge.\n"
        )

    # ========================================================
    # MAIN SIMULATION LOOP
    # ========================================================

    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIMULATION_TIME
    ):

        vehicle_ids = traci.vehicle.getIDList()

        if EGO_ID in vehicle_ids and vehicle_model is not None:

            try:

                traci.vehicle.setSpeed(
                    EGO_ID,
                    commanded_speed
                )

            except traci.TraCIException:
                pass

        traci.simulationStep()

        current_time = traci.simulation.getTime()
        last_simulation_time = current_time

        vehicle_ids = traci.vehicle.getIDList()
        current_edge_for_commands = (
            traci.vehicle.getRoadID(EGO_ID)
            if EGO_ID in vehicle_ids
            else None
        )

        # Commands are appended by the visualization process to commands.jsonl.
        if engine_interface is not None:
            for engine_command in engine_interface.read_commands():
                process_visualizer_command(
                    engine_command,
                    current_time,
                    current_edge=current_edge_for_commands,
                )

        # ====================================================
        # SYNTHETIC SCENARIO OBSTACLE
        # ====================================================

        if (
            SCENARIO == "synthetic"
            and not obstacle_added
            and current_time >= OBSTACLE_TIME
            and EGO_ID in vehicle_ids
        ):

            current_edge = traci.vehicle.getRoadID(EGO_ID)

            if current_edge != OBSTACLE_EDGE:

                print()
                print("========================================")
                print("OBSTACLE DETECTED")
                print(f"Obstacle on road: {OBSTACLE_EDGE}")
                print("========================================")

                block_edge(OBSTACLE_EDGE)

                last_replan_time = current_time - REPLAN_INTERVAL
                pending_replan_reason = "scripted_synthetic_obstacle"
                if engine_interface is not None:
                    engine_interface.log_event(
                        "road_closed", current_time, edge_id=OBSTACLE_EDGE,
                        source="synthetic_test_scenario"
                    )

                obstacle_added = True

        # ====================================================
        # OSM ROAD-CLOSURE EXPERIMENT (OPT-IN)
        # ====================================================

        if (
            TEST_OSM_CLOSURE
            and not obstacle_added
            and route_assigned
            and current_time >= next_osm_closure_attempt
            and EGO_ID in vehicle_ids
        ):

            current_edge_for_event = (
                traci.vehicle.getRoadID(EGO_ID)
            )

            if (
                current_edge_for_event
                and not current_edge_for_event.startswith(":")
            ):

                candidate = find_osm_closure_candidate(
                    current_edge_for_event
                )

                if candidate is not None:

                    osm_closure_edge = candidate

                    # Apply the actual simulated closure.
                    block_edge(osm_closure_edge)

                    obstacle_added = True

                    osm_closure_detected_time = current_time
                    osm_closure_wall_started = time.perf_counter()
                    pending_replan_reason = "scripted_osm_road_closure"
                    if engine_interface is not None:
                        engine_interface.log_event(
                            "road_closed", current_time,
                            edge_id=osm_closure_edge,
                            source="scripted_osm_closure_experiment",
                        )

                    # Force route evaluation during this simulation cycle.
                    last_replan_time = current_time - REPLAN_INTERVAL

                    print()
                    print("========================================")
                    print("[OSM CLOSURE TEST] ROAD CLOSURE TRIGGERED")

                    print(
                        "[OSM CLOSURE TEST] Blocked edge:",
                        osm_closure_edge
                    )

                    print(
                        "[OSM CLOSURE TEST] Vehicle current edge:",
                        current_edge_for_event
                    )

                    print("========================================")

                else:

                    next_osm_closure_attempt = (
                        current_time + OSM_CLOSURE_RETRY_INTERVAL
                    )

                    print(
                        "[OSM CLOSURE TEST] No suitable detour "
                        "candidate yet; "
                        f"retrying at {next_osm_closure_attempt:.1f}s."
                    )

        # ====================================================
        # FIXED-EDGE HAZARD COMPARISON (ALL THREE PLANNERS)
        # ====================================================

        if (
            TEST_OSM_HAZARD_COMPARISON
            and not hazard_comparison_active
            and route_assigned
            and current_time >= HAZARD_TIME
            and EGO_ID in vehicle_ids
        ):

            if not HAZARD_COMPARISON_EDGE or HAZARD_COMPARISON_EDGE not in edge_data:
                raise RuntimeError(
                    "Comparative hazard edge is not present in the loaded network: "
                    f"{HAZARD_COMPARISON_EDGE!r}"
                )

            comparison_route = list(traci.vehicle.getRoute(EGO_ID))
            comparison_route_index = traci.vehicle.getRouteIndex(EGO_ID)
            if comparison_route_index < 0:
                comparison_route_index = 0

            comparison_remaining_route = comparison_route[comparison_route_index:]
            hazard_comparison_initial_exposure = (
                HAZARD_COMPARISON_EDGE in comparison_remaining_route
            )

            comparison_current_edge = traci.vehicle.getRoadID(EGO_ID)
            hazard_comparison_already_entered = (
                comparison_current_edge == HAZARD_COMPARISON_EDGE
            )

            set_hazard(
                HAZARD_COMPARISON_EDGE,
                HAZARD_SEVERITY,
                "Fixed hazard for three-planner comparison"
            )
            pending_replan_reason = "fixed_edge_hazard_comparison"
            if engine_interface is not None:
                engine_interface.log_event(
                    "hazard_activated",
                    current_time,
                    edge_id=HAZARD_COMPARISON_EDGE,
                    severity=HAZARD_SEVERITY,
                    source="fixed_edge_comparison_experiment",
                )

            hazard_comparison_active = True
            hazard_comparison_activation_time = current_time
            last_replan_time = current_time - REPLAN_INTERVAL

            print()
            print("========== FIXED-EDGE HAZARD COMPARISON ==========")
            print("Planner:", PLANNER_MODE)
            print("Hazard activated at:", f"{current_time:.1f}s")
            print("Hazard edge:", HAZARD_COMPARISON_EDGE)
            print("Hazard severity:", f"{HAZARD_SEVERITY}/3")
            print(
                "Hazard in remaining route before activation:",
                hazard_comparison_initial_exposure
            )
            print("Vehicle already on hazard edge:", hazard_comparison_already_entered)
            print("Road remains open; immediate replanning requested.")
            print("===================================================")

        # ====================================================
        # OSM GRADED-HAZARD EXPERIMENT (OPT-IN)
        # ====================================================

        if (
            TEST_OSM_HAZARD
            and not risk_hazard_added
            and route_assigned
            and current_time >= HAZARD_TIME
            and EGO_ID in vehicle_ids
            and current_time - last_hazard_attempt_time
                >= HAZARD_RETRY_INTERVAL
        ):

            last_hazard_attempt_time = current_time

            current_edge_for_hazard = (
                traci.vehicle.getRoadID(EGO_ID)
            )

            if (
                current_edge_for_hazard
                and not current_edge_for_hazard.startswith(":")
            ):

                risk_hazard_wall_started = time.perf_counter()

                candidate_result = find_risk_avoidance_candidate(
                    current_edge_for_hazard
                )

                if candidate_result is not None:

                    (
                        risk_hazard_edge,
                        risk_hazard_baseline_cost,
                        risk_hazard_alternative_cost
                    ) = candidate_result

                    # Apply the simulated hazard permanently for this run.
                    # The road remains available; only its route cost rises.
                    set_hazard(
                        risk_hazard_edge,
                        HAZARD_SEVERITY,
                        "Simulated high-risk road hazard"
                    )

                    risk_hazard_added = True
                    risk_hazard_detected_time = current_time
                    pending_replan_reason = "scripted_osm_risk_hazard"
                    if engine_interface is not None:
                        engine_interface.log_event(
                            "hazard_activated", current_time,
                            edge_id=risk_hazard_edge,
                            severity=HAZARD_SEVERITY,
                            source="scripted_osm_hazard_experiment",
                        )

                    # Trigger replanning in this same simulation cycle.
                    last_replan_time = current_time - REPLAN_INTERVAL

                    print()
                    print("========================================")
                    print("[OSM RISK TEST] HAZARD ACTIVATED")

                    print(
                        "[OSM RISK TEST] Hazard edge:",
                        risk_hazard_edge
                    )

                    print(
                        "[OSM RISK TEST] Severity:",
                        HAZARD_SEVERITY
                    )

                    print(
                        "[OSM RISK TEST] Baseline route cost:",
                        f"{risk_hazard_baseline_cost:.2f}"
                    )

                    print(
                        "[OSM RISK TEST] Risk-aware route cost:",
                        f"{risk_hazard_alternative_cost:.2f}"
                    )

                    print(
                        "[OSM RISK TEST] Road remains open; "
                        "rerouting requested."
                    )

                    print("========================================")

                else:

                    print(
                        "[OSM RISK TEST] No suitable risk-avoidance "
                        "candidate found; retrying in "
                        f"{HAZARD_RETRY_INTERVAL:.1f}s."
                    )

                    risk_hazard_wall_started = None

        if EGO_ID not in vehicle_ids:
            continue

        # ====================================================
        # INITIAL ROUTE ASSIGNMENT
        # ====================================================

        if not route_assigned:

            if SCENARIO == "large":

                current_edge = traci.vehicle.getRoadID(EGO_ID)

                if START_EDGE_OVERRIDE and current_edge != START_EDGE_OVERRIDE:
                    raise RuntimeError(
                        f"Ego vehicle started on {current_edge!r}, but SUMO_START_EDGE="
                        f"{START_EDGE_OVERRIDE!r}. Check the SUMO route/config."
                    )

                if (
                    not current_edge
                    or current_edge.startswith(":")
                ):
                    continue

                current_junction = get_current_junction(
                    current_edge
                )

                if current_junction is None:

                    print(
                        f"Waiting for a recognizable road edge: "
                        f"{current_edge}"
                    )

                    continue

                # Inspect the two known candidate corridors using the
                # traffic state that exists at this exact initial decision.
                # This diagnostic does not change the planner or SUMO route.
                if DIAGNOSE_INITIAL_ROUTE_COSTS:
                    inspect_initial_route_costs(
                        current_edge,
                        current_junction
                    )

                # Edge-based A*: use the current edge to constrain
                # the legal next roads.

                path, total_cost, selected_edges = plan_path(
                    current_junction,
                    GOAL_JUNCTION,
                    edge_cost_function=get_active_cost_function(),
                    return_edges=True,
                    start_edge=current_edge
                )
                route_cost = total_cost

                if not path:

                    raise RuntimeError(
                        f"A* could not find an OSM route from "
                        f"{current_junction} to {GOAL_JUNCTION}."
                    )

                if DIAGNOSE_INITIAL_ROUTE_COSTS:
                    # Report the traffic cost of the actual selected path.
                    selected_traffic_cost = sum(
                        get_traffic_planning_cost(edge_id)
                        for edge_id in selected_edges
                    )

                    print(
                        "[ROUTE DIAGNOSTIC] A* selected path traffic cost:",
                        f"{selected_traffic_cost:.2f} s"
                    )

                    print(
                        "[ROUTE DIAGNOSTIC] A* selected remaining edges:",
                        selected_edges
                    )

                # The current edge is already occupied.
                route_edges = [current_edge] + selected_edges

                print("\nLarge-map initial A* route")
                print("Current edge:", current_edge)
                print("Start junction:", current_junction)
                print("Goal junction:", GOAL_JUNCTION)
                print("Planned junction path:", path)
                print("Selected SUMO edges:", route_edges)

                print(
                    "Total cost:",
                    f"{total_cost:.2f}"
                )

            try:

                traci.vehicle.setRoute(
                    EGO_ID,
                    route_edges
                )

            except traci.TraCIException as error:

                raise RuntimeError(
                    f"SUMO rejected the planned route: {error}"
                ) from error

            traci.vehicle.setSpeedMode(EGO_ID, 0)
            traci.vehicle.setLaneChangeMode(EGO_ID, 257)

            route_assigned = True
            last_replan_time = current_time

            reference_path = get_route_path_points(
                route_edges
            )

            if engine_interface is not None:
                engine_interface.log_event(
                    "route_planned",
                    current_time,
                    reason="initial_plan",
                    planner=PLANNER_MODE,
                    start_edge=route_edges[0] if route_edges else None,
                    goal_junction=GOAL_JUNCTION,
                    route_edges=list(route_edges),
                    route_cost=route_cost,
                )
                engine_interface.publish_route({
                    "run_id": RUN_ID,
                    "planner": PLANNER_MODE,
                    "goal_junction": GOAL_JUNCTION,
                    "route_edges": list(route_edges),
                    "path_points": [[float(x), float(y)] for x, y in reference_path],
                    "reason": "initial_plan",
                    "simulation_time": current_time,
                })

            if len(reference_path) < 2:

                raise RuntimeError(
                    "The selected route did not produce enough "
                    "reference-path points."
                )

            reset_tracker()

            initial_position = traci.vehicle.getPosition(EGO_ID)

            initial_sumo_angle = traci.vehicle.getAngle(EGO_ID)

            initial_heading = convert_sumo_angle(
                initial_sumo_angle
            )

            vehicle_model = BicycleModel(
                x=initial_position[0],
                y=initial_position[1],
                heading=initial_heading,
                speed=TARGET_SPEED,
                wheelbase=WHEELBASE
            )

            commanded_speed = TARGET_SPEED

            print(
                f"Route assigned to {EGO_ID}:",
                route_edges
            )

            print(
                "Reference path points:",
                len(reference_path)
            )

            print("Bicycle model initialized.")

            print(
                f"Target speed: {TARGET_SPEED:.2f} m/s"
            )

        # ====================================================
        # DYNAMIC REPLANNING
        # ====================================================

        if current_time - last_replan_time >= REPLAN_INTERVAL:

            current_edge = traci.vehicle.getRoadID(EGO_ID)

            if (
                current_edge
                and not current_edge.startswith(":")
            ):

                current_junction = get_current_junction(
                    current_edge
                )

                if (
                    current_junction is not None
                    and current_junction != GOAL_JUNCTION
                ):

                    print()
                    print("Checking for a better route...")
                    print("Current edge:", current_edge)
                    print("Current junction:", current_junction)

                    # Edge-based A*: constrain the next road using
                    # the road the vehicle is currently occupying.

                    new_path, new_cost, remaining_edges = plan_path(
                        current_junction,
                        GOAL_JUNCTION,
                        edge_cost_function=get_active_cost_function(),
                        return_edges=True,
                        start_edge=current_edge
                    )

                    if new_path:

                        route_cost = new_cost
                        print("New junction path:", new_path)

                        new_route_edges = (
                            [current_edge] + remaining_edges
                        )

                        # Evaluate the first A* route computed after the fixed hazard.
                        # This check is outside the setRoute-only branch so planners
                        # that retain their existing route are measured too.
                        if (
                            TEST_OSM_HAZARD_COMPARISON
                            and hazard_comparison_active
                            and not hazard_comparison_replan_checked
                        ):

                            hazard_comparison_replanned_exposure = (
                                HAZARD_COMPARISON_EDGE in new_route_edges
                            )
                            hazard_comparison_replan_checked = True

                            if not hazard_comparison_initial_exposure:
                                hazard_comparison_result = "NOT_EXPOSED_BEFORE_ACTIVATION"
                            elif current_edge == HAZARD_COMPARISON_EDGE:
                                hazard_comparison_result = "ALREADY_ON_HAZARD_EDGE_AT_ACTIVATION"
                            elif hazard_comparison_replanned_exposure:
                                hazard_comparison_result = "PLANNER_RETAINED_HAZARD_EDGE"
                            else:
                                hazard_comparison_result = "PLANNER_AVOIDED_HAZARD_EDGE"

                            print("[HAZARD COMPARISON] Pre-activation route contained hazard:",
                                  hazard_comparison_initial_exposure)
                            print("[HAZARD COMPARISON] First post-hazard A* route contains hazard:",
                                  hazard_comparison_replanned_exposure)
                            print("[HAZARD COMPARISON] Planner response:",
                                  hazard_comparison_result)

                        # Ensure the updated route avoids the
                        # road that was closed in this experiment.
                        if (
                            TEST_OSM_CLOSURE
                            and osm_closure_edge is not None
                            and osm_closure_edge in new_route_edges
                        ):

                            raise RuntimeError(
                                "Closure test failed: A* returned "
                                "a route containing the blocked edge "
                                f"{osm_closure_edge}."
                            )

                        current_route = list(
                            traci.vehicle.getRoute(EGO_ID)
                        )

                        route_index = traci.vehicle.getRouteIndex(
                            EGO_ID
                        )

                        if route_index >= 0:

                            current_remaining_route = (
                                current_route[route_index:]
                            )

                        else:

                            current_remaining_route = []

                        if new_route_edges != current_remaining_route:

                            print("Different route found.")

                            try:

                                traci.vehicle.setRoute(
                                    EGO_ID,
                                    new_route_edges
                                )

                                # Confirm that the accepted route avoids
                                # the hazard while keeping the road open.
                                if (
                                    TEST_OSM_HAZARD
                                    and risk_hazard_added
                                    and risk_hazard_edge is not None
                                    and risk_hazard_reroute_time is None
                                ):

                                    risk_hazard_rerouted = (
                                        risk_hazard_edge not in new_route_edges
                                    )

                                    if risk_hazard_rerouted:

                                        risk_hazard_reroute_time = current_time

                                        elapsed = (
                                            time.perf_counter()
                                            - risk_hazard_wall_started
                                            if risk_hazard_wall_started is not None
                                            else float("nan")
                                        )

                                        print(
                                            "[OSM RISK TEST] PASSED - "
                                            "hazardous road excluded from "
                                            "accepted route."
                                        )

                                        print(
                                            "[OSM RISK TEST] Reroute wall time:",
                                            f"{elapsed:.4f}s"
                                        )

                                        print(
                                            "[OSM RISK TEST] Reroute simulation time:",
                                            f"{risk_hazard_reroute_time:.1f}s"
                                        )

                                    else:

                                        print(
                                            "[OSM RISK TEST] Hazard remains "
                                            "on the accepted route; "
                                            "avoidance not confirmed."
                                        )

                                previous_route_edges = list(route_edges)
                                route_edges = new_route_edges

                                reference_path = get_route_path_points(
                                    route_edges
                                )

                                if engine_interface is not None:
                                    engine_interface.log_event(
                                        "route_updated",
                                        current_time,
                                        reason=pending_replan_reason,
                                        planner=PLANNER_MODE,
                                        previous_route_edges=previous_route_edges,
                                        route_edges=list(route_edges),
                                        goal_junction=GOAL_JUNCTION,
                                        route_changed=True,
                                    )
                                    engine_interface.publish_route({
                                        "run_id": RUN_ID,
                                        "planner": PLANNER_MODE,
                                        "goal_junction": GOAL_JUNCTION,
                                        "route_edges": list(route_edges),
                                        "path_points": [[float(x), float(y)] for x, y in reference_path],
                                        "reason": pending_replan_reason,
                                        "simulation_time": current_time,
                                    })
                                pending_replan_reason = "periodic_traffic_refresh"

                                reset_tracker()

                                print(
                                    "Dynamic reroute applied:",
                                    new_route_edges
                                )

                                print("Reference path updated.")

                                # Record successful closure rerouting.
                                if (
                                    TEST_OSM_CLOSURE
                                    and osm_closure_detected_time is not None
                                    and osm_closure_reroute_time is None
                                ):

                                    osm_closure_reroute_time = current_time

                                    planning_elapsed = (
                                        time.perf_counter()
                                        - osm_closure_wall_started
                                        if osm_closure_wall_started is not None
                                        else float("nan")
                                    )

                                    print(
                                        "[OSM CLOSURE TEST] "
                                        "Wall-clock reroute time:",
                                        f"{planning_elapsed:.4f}s"
                                    )

                                    print(
                                        "[OSM CLOSURE TEST] "
                                        "Reroute applied at simulation time:",
                                        f"{osm_closure_reroute_time:.1f}s"
                                    )

                                    print(
                                        "[OSM CLOSURE TEST] "
                                        "Blocked edge excluded:",
                                        osm_closure_edge not in new_route_edges
                                    )

                            except traci.TraCIException as error:

                                print(
                                    "Could not apply reroute:",
                                    error
                                )

                        else:

                            print("Current route is still suitable.")
                            if engine_interface is not None:
                                engine_interface.log_event(
                                    "replan_completed",
                                    current_time,
                                    reason=pending_replan_reason,
                                    planner=PLANNER_MODE,
                                    route_changed=False,
                                    route_edges=list(route_edges),
                                    goal_junction=GOAL_JUNCTION,
                                    route_cost=route_cost,
                                )

                    else:

                        print("No route available.")
                        if engine_interface is not None:
                            engine_interface.log_event(
                                "route_unavailable",
                                current_time,
                                reason=pending_replan_reason,
                                planner=PLANNER_MODE,
                                current_edge=current_edge,
                                current_junction=current_junction,
                                goal_junction=GOAL_JUNCTION,
                            )

            last_replan_time = current_time
            # Consume the trigger reason after every completed route evaluation,
            # including unchanged/no-route outcomes. This prevents a later
            # periodic update from being misattributed to an old UI command.
            pending_replan_reason = "periodic_traffic_refresh"

        # ====================================================
        # VEHICLE MODEL AND GOAL CHECK
        # ====================================================

        model_x, model_y, model_heading, model_speed = (
            vehicle_model.get_state()
        )

        if reached_goal(model_x, model_y):

            goal_reached_flag = True
            if engine_interface is not None:
                engine_interface.log_event(
                    "goal_reached",
                    current_time,
                    position={"x": model_x, "y": model_y},
                    goal_junction=GOAL_JUNCTION,
                    route_edges=list(route_edges),
                )
                engine_interface.publish_state({
                    "run_id": RUN_ID,
                    "status": "goal_reached",
                    "simulation_time": current_time,
                    "vehicle_id": EGO_ID,
                    "planner": PLANNER_MODE,
                    "goal_junction": GOAL_JUNCTION,
                    "position": {"x": model_x, "y": model_y},
                    "heading_deg": model_heading,
                    "speed_mps": model_speed,
                    "route_edges": list(route_edges),
                    "goal_reached": True,
                })

            print()
            print("========================================")
            print("GOAL REACHED")

            print(
                f"Final position: ({model_x:.2f}, {model_y:.2f})"
            )

            print("========================================")

            
     
            try:
                traci.vehicle.setSpeed(EGO_ID, 0.0)

                # ============================================================
                # MOVE TO BICYCLE MODEL POSITION
                # ============================================================

                traci.vehicle.moveToXY(
                    EGO_ID,
                    "",
                    -1,
                    model_x,
                    model_y,
                    convert_cartesian_angle_to_sumo(model_heading),
                    1
                )

                # ============================================================
                # MODEL vs SUMO SYNCHRONIZATION DIAGNOSTIC
                # ============================================================

                sumo_x, sumo_y = traci.vehicle.getPosition(EGO_ID)
                sumo_angle = traci.vehicle.getAngle(EGO_ID)

                sumo_heading = normalize_angle(90.0 - sumo_angle)

                position_error = math.hypot(
                    sumo_x - model_x,
                    sumo_y - model_y
                )

                heading_sync_error = normalize_angle(
                    sumo_heading - model_heading
                )

                if position_error > 0.5 or abs(heading_sync_error) > 5.0:
                    print(
                        f"[SYNC WARNING] "
                        f"Time={traci.simulation.getTime():.1f}s | "
                        f"Position error={position_error:.3f}m | "
                        f"Heading error={heading_sync_error:.2f}deg"
                    )

            except traci.TraCIException:
                pass

            break
        
        if len(reference_path) < 2:
            continue

        # ====================================================
        # PATH TRACKING
        # ====================================================

        (
            cross_track_error,
            heading_error,
            closest_index
        ) = calculate_tracking_error(
            (model_x, model_y),
            model_heading,
            reference_path
        )

        steering_angle = calculate_steering_angle(
            cross_track_error,
            heading_error,
            model_speed
        )

        desired_speed = calculate_model_speed(
            steering_angle
        )

        vehicle_model.update(
            steering_angle=steering_angle,
            speed=desired_speed,
            dt=MODEL_DT
        )

        new_x, new_y, new_heading, new_speed = (
            vehicle_model.get_state()
        )

        new_sumo_angle = convert_cartesian_angle_to_sumo(
            new_heading
        )

        # ====================================================
        # HEADING DIAGNOSTIC: MODEL vs REFERENCE PATH
        # ====================================================
        # Compare the bicycle-model heading with the local reference-path
        # direction before sending the new position to SUMO.
        reference_heading = float("nan")
        reference_index = max(
            0,
            min(closest_index, len(reference_path) - 2)
        )

        if len(reference_path) >= 2:
            ref_x1, ref_y1 = reference_path[reference_index]
            ref_x2, ref_y2 = reference_path[reference_index + 1]
            reference_heading = math.degrees(
                math.atan2(ref_y2 - ref_y1, ref_x2 - ref_x1)
            )

        heading_debug_time = traci.simulation.getTime()
        print_heading_diagnostic = (
            DEBUG_DIAGNOSTICS
            and int(round(heading_debug_time * 10)) % 10 == 0
        )

        if print_heading_diagnostic:
            print("\\n========== HEADING DIAGNOSTIC (BEFORE moveToXY) ==========")
            print(f"Time: {heading_debug_time:.1f}s")
            print(f"Reference index: {reference_index}")
            print(f"Reference path heading: {reference_heading:.2f} deg")
            print(f"Bicycle-model heading: {new_heading:.2f} deg")
            print(f"Commanded SUMO heading: {new_sumo_angle:.2f} deg")
            print(
                "Model vs reference error: "
                f"{normalize_angle(reference_heading - new_heading):.2f} deg"
            )

        # ====================================================
        # UPDATE SUMO AND LOG TELEMETRY
        # ====================================================

        try:

            commanded_speed = new_speed

            traci.vehicle.setSpeed(
                EGO_ID,
                new_speed
            )

            # TEMPORARY: inspect SUMO preferred lanes once per simulated second.
            try:
                debug_time = traci.simulation.getTime()

                if DEBUG_DIAGNOSTICS and int(round(debug_time * 10)) % 10 == 0:
                    current_road = traci.vehicle.getRoadID(EGO_ID)
                    current_lane = traci.vehicle.getLaneID(EGO_ID)
                    current_lane_index = traci.vehicle.getLaneIndex(EGO_ID)

                    route = list(traci.vehicle.getRoute(EGO_ID))
                    route_index = traci.vehicle.getRouteIndex(EGO_ID)

                    next_edge = (
                        route[route_index + 1]
                        if 0 <= route_index < len(route) - 1
                        else None
                    )

                    best_lanes = traci.vehicle.getBestLanes(EGO_ID)

                    print("\n========== PREFERRED LANE DIAGNOSTIC ==========")
                    print(f"Time: {debug_time:.1f}s")
                    print(f"Current road: {current_road}")
                    print(f"Current lane: {current_lane}")
                    print(f"Current lane index: {current_lane_index}")
                    print(f"Route index: {route_index}")
                    print(f"Next route edge: {next_edge}")
                    print("\nSUMO preferred lanes:")

                    for lane_info in best_lanes:
                        (
                            lane_id,
                            length,
                            occupation,
                            offset,
                            allows_continuation,
                            next_lanes
                        ) = lane_info

                        print(
                            f"Lane: {lane_id} | "
                            f"Continuation: {allows_continuation} | "
                            f"Offset: {offset} | "
                            f"Next lanes: {next_lanes}"
                        )

                    print("===============================================\n")

            except traci.TraCIException as error:
                print("[LANE DIAGNOSTIC ERROR]", error)

            traci.vehicle.moveToXY(
                EGO_ID,
                "",
                -1,
                new_x,
                new_y,
                new_sumo_angle,
                1
            )

            # ====================================================
            # MODEL vs SUMO POSITION / HEADING DIAGNOSTIC
            # Print once per simulated second to avoid flooding the terminal.
            # ====================================================
            if print_heading_diagnostic:
                sumo_x, sumo_y = traci.vehicle.getPosition(EGO_ID)
                sumo_angle = traci.vehicle.getAngle(EGO_ID)
                sumo_heading = convert_sumo_angle(sumo_angle)
                sumo_edge = traci.vehicle.getRoadID(EGO_ID)
                sumo_lane = traci.vehicle.getLaneID(EGO_ID)
                sumo_speed = traci.vehicle.getSpeed(EGO_ID)

                position_error = math.hypot(
                    sumo_x - new_x,
                    sumo_y - new_y
                )
                heading_sync_error = normalize_angle(
                    sumo_heading - new_heading
                )

                print(
                    f"[POSITION CHECK] "
                    f"Model=({new_x:.2f}, {new_y:.2f}) | "
                    f"SUMO=({sumo_x:.2f}, {sumo_y:.2f}) | "
                    f"Edge={sumo_edge} | "
                    f"Lane={sumo_lane} | "
                    f"SUMO speed={sumo_speed:.2f}m/s | "
                    f"Position error={position_error:.3f}m | "
                    f"Heading sync error={heading_sync_error:.2f}deg"
                )

                if position_error > 0.5 or abs(heading_sync_error) > 5.0:
                    print(
                        f"[SYNC WARNING] "
                        f"Time={current_time:.1f}s | "
                        f"Position error={position_error:.3f}m | "
                        f"Heading error={heading_sync_error:.2f}deg"
                    )

            # Read SUMO's actual state after moveToXY and compare it with
            # the bicycle model's commanded state.
            if print_heading_diagnostic:
                actual_sumo_angle = traci.vehicle.getAngle(EGO_ID)
                actual_cartesian_heading = convert_sumo_angle(
                    actual_sumo_angle
                )
                actual_sumo_position = traci.vehicle.getPosition(EGO_ID)

                print("---------- HEADING DIAGNOSTIC (AFTER moveToXY) ----------")
                print(
                    f"SUMO actual position: "
                    f"({actual_sumo_position[0]:.2f}, "
                    f"{actual_sumo_position[1]:.2f})"
                )
                print(
                    f"SUMO actual angle: {actual_sumo_angle:.2f} deg"
                )
                print(
                    f"SUMO actual Cartesian heading: "
                    f"{actual_cartesian_heading:.2f} deg"
                )
                print(
                    "SUMO vs model heading difference: "
                    f"{normalize_angle(actual_cartesian_heading - new_heading):.2f} deg"
                )
                print(
                    "SUMO vs reference heading difference: "
                    f"{normalize_angle(reference_heading - actual_cartesian_heading):.2f} deg"
                )
                print("========================================================")

            if telemetry_logger is not None:

                try:

                    sumo_x, sumo_y = traci.vehicle.getPosition(
                        EGO_ID
                    )

                    sumo_speed = traci.vehicle.getSpeed(
                        EGO_ID
                    )

                    current_edge_for_log = (
                        traci.vehicle.getRoadID(EGO_ID)
                    )

                    telemetry_logger.log(
                        simulation_time=current_time,
                        model_x=new_x,
                        model_y=new_y,
                        model_heading=new_heading,
                        model_speed=new_speed,
                        sumo_x=sumo_x,
                        sumo_y=sumo_y,
                        sumo_speed=sumo_speed,
                        cross_track_error=cross_track_error,
                        heading_error=heading_error,
                        steering_angle=steering_angle,
                        target_index=closest_index,
                        current_edge=current_edge_for_log
                    )

                    if (
                        engine_interface is not None
                        and current_time - last_state_publish_time >= STATE_PUBLISH_INTERVAL
                    ):
                        lane_id = traci.vehicle.getLaneID(EGO_ID)
                        engine_interface.publish_state({
                            "run_id": RUN_ID,
                            "status": "running",
                            "simulation_time": current_time,
                            "vehicle_id": EGO_ID,
                            "scenario": SCENARIO,
                            "planner": PLANNER_MODE,
                            "goal_junction": GOAL_JUNCTION,
                            "position": {"x": new_x, "y": new_y},
                            "sumo_position": {"x": sumo_x, "y": sumo_y},
                            "heading_deg": new_heading,
                            "speed_mps": new_speed,
                            "sumo_speed_mps": sumo_speed,
                            "current_edge": current_edge_for_log,
                            "current_lane": lane_id,
                            "cross_track_error_m": cross_track_error,
                            "heading_error_deg": heading_error,
                            "steering_angle_deg": steering_angle,
                            "target_index": closest_index,
                            "goal_reached": False,
                            "route_edges": list(route_edges),
                            "active_hazards": get_active_hazards(),
                            "blocked_edges": sorted(get_blocked_edges()),
                        })
                        last_state_publish_time = current_time

                except (
                    traci.TraCIException,
                    OSError,
                    ValueError
                ) as error:

                    print(
                        "Telemetry logging warning:",
                        error
                    )

        except traci.TraCIException as error:

            print(
                "Bicycle model TraCI error:",
                error
            )

            break

        if current_time - last_console_print_time >= CONSOLE_LOG_INTERVAL:
            print(
                f"Time: {current_time:6.1f}s | "
                f"Position: ({new_x:7.2f}, {new_y:7.2f}) | "
                f"Speed: {new_speed:5.2f} m/s | "
                f"CTE: {cross_track_error:7.2f} m | "
                f"Heading Error: {heading_error:7.2f}° | "
                f"Steering: {steering_angle:7.2f}° | "
                f"Target: {closest_index}"
            )
            last_console_print_time = current_time


finally:

    try:
        traci.close()

    except traci.TraCIException:
        pass

    if telemetry_logger is not None:
        telemetry_logger.close()

    if engine_interface is not None:
        final_summary = {
            "run_id": RUN_ID,
            "scenario": SCENARIO,
            "planner": PLANNER_MODE,
            "seed": SUMO_SEED,
            "vehicle_id": EGO_ID,
            "goal_junction": GOAL_JUNCTION,
            "goal_reached": goal_reached_flag,
            "final_route_edges": list(route_edges),
            "active_hazards": get_active_hazards(),
            "blocked_edges": sorted(get_blocked_edges()),
        }
        engine_interface.log_event(
            "simulation_finished",
            simulation_time=last_simulation_time,
            goal_reached=goal_reached_flag,
            final_route_edges=list(route_edges),
        )
        engine_interface.publish_summary(final_summary)

    # ========================================================
    # FINAL FIXED-EDGE THREE-PLANNER HAZARD COMPARISON REPORT
    # ========================================================

    if TEST_OSM_HAZARD_COMPARISON:

        print()
        print("========== FIXED-EDGE HAZARD COMPARISON SUMMARY ==========")
        print("Planner:", PLANNER_MODE)
        print("Hazard edge:", HAZARD_COMPARISON_EDGE)
        print("Hazard severity:", f"{HAZARD_SEVERITY}/3")
        print("Hazard activated:", hazard_comparison_active)
        print("Vehicle reached goal:", goal_reached_flag)
        print("Pre-activation route exposed:", hazard_comparison_initial_exposure)
        print("Post-hazard A* route contains hazard:", hazard_comparison_replanned_exposure)
        print("Planner response:", hazard_comparison_result)

        if not hazard_comparison_active:
            print("COMPARISON RESULT: INCOMPLETE - hazard was not activated.")
        elif not hazard_comparison_replan_checked:
            print("COMPARISON RESULT: INCOMPLETE - no post-hazard A* result was recorded.")
        elif hazard_comparison_result == "NOT_EXPOSED_BEFORE_ACTIVATION":
            print("COMPARISON RESULT: NOT EXPOSED - the initial remaining route did not use this edge.")
        elif hazard_comparison_result == "ALREADY_ON_HAZARD_EDGE_AT_ACTIVATION":
            print("COMPARISON RESULT: INCONCLUSIVE - vehicle had already entered the hazard edge.")
        elif hazard_comparison_result == "PLANNER_AVOIDED_HAZARD_EDGE":
            print("COMPARISON RESULT: AVOIDED - post-hazard A* route excludes the hazard edge.")
        else:
            print("COMPARISON RESULT: NOT AVOIDED - post-hazard A* route still includes the hazard edge.")

        print("END-TO-END RESULT:", "GOAL REACHED" if goal_reached_flag else "GOAL NOT REACHED")

    # ========================================================
    # FINAL OSM RISK-AWARE EXPERIMENT REPORT
    # ========================================================

    if TEST_OSM_HAZARD:

        print()
        print("========== OSM RISK-AWARE EXPERIMENT SUMMARY ==========")

        print("Hazard activated:", risk_hazard_added)
        print("Hazard edge:", risk_hazard_edge)
        print("Hazard severity:", f"{HAZARD_SEVERITY}/3")
        print("Vehicle reached goal:", goal_reached_flag)
        print("Alternative route accepted:", risk_hazard_rerouted)

        if risk_hazard_edge is None:

            print(
                "RESULT: NOT COMPLETED - no suitable road with an "
                "available safer alternative was found."
            )

        elif not risk_hazard_rerouted:

            print(
                "RESULT: NOT CONFIRMED - hazard was activated, but "
                "an alternative route was not confirmed as accepted."
            )

        elif risk_hazard_edge in route_edges:

            print(
                "RESULT: FAILED - final route still contains the "
                "hazardous edge."
            )

        else:

            print(
                "RESULT: RISK-AWARE REROUTING PASSED - hazardous "
                "edge is excluded from the final route."
            )

            if goal_reached_flag:

                print(
                    "END-TO-END RESULT: PASSED - vehicle reached "
                    "the destination after risk-aware rerouting."
                )

            else:

                print(
                    "END-TO-END RESULT: INCOMPLETE - rerouting "
                    "succeeded, but destination arrival was not confirmed."
                )

    # ========================================================
    # FINAL OSM CLOSURE EXPERIMENT REPORT
    # ========================================================

    if TEST_OSM_CLOSURE:

        print()
        print("========== OSM CLOSURE EXPERIMENT SUMMARY ==========")

        print("Closure edge:", osm_closure_edge)
        print("Vehicle reached goal:", goal_reached_flag)

        if osm_closure_edge is None:

            print(
                "RESULT: NOT COMPLETED - no suitable closure "
                "candidate was found."
            )

        elif osm_closure_reroute_time is None:

            print(
                "RESULT: FAILED - closure was triggered but "
                "reroute was not applied."
            )

        elif osm_closure_edge in route_edges:

            print(
                "RESULT: FAILED - final route still contains "
                "the blocked edge."
            )

        else:

            print(
                "RESULT: REROUTING PASSED - blocked edge is "
                "excluded from the updated route."
            )

            if goal_reached_flag:

                print(
                    "END-TO-END RESULT: PASSED - vehicle reached "
                    "the destination after rerouting."
                )

            else:

                print(
                    "END-TO-END RESULT: INCOMPLETE - reroute "
                    "succeeded, but destination arrival was "
                    "not confirmed."
                )

    print()
    print("Simulation finished.")