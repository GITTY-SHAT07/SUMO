import os
import traci


from network_reader import graph
from traffic_cost import get_traffic_cost
from path_planner import plan_path, junctions_to_edges
from obstacle_manager import block_edge
from path_tracker import calculate_tracking_error, reset_tracker
from Steering_controller import calculate_steering_angle
from vehicle_model import BicycleModel




PROJECT_DIR = os.path.dirname(
    os.path.dirname(
        os.path.abspath(__file__)
    )
)

SUMO_CFG = os.path.join(
    PROJECT_DIR,
    "simulation",
    "simulation.sumocfg"
)




EGO_ID = "egoVehicle"

START_JUNCTION = "J0"
GOAL_JUNCTION = "J2"




MAX_SIMULATION_TIME = 100.0
MODEL_DT = 0.1




WHEELBASE = 2.5
TARGET_SPEED = 6.0


GOAL_DISTANCE_THRESHOLD = 3.0




REPLAN_INTERVAL = 10.0




OBSTACLE_EDGE = "E2"
OBSTACLE_TIME = 45.0




reference_path = []
vehicle_model = None
route_assigned = False
obstacle_added = False
last_replan_time = 0.0

# Speed command that SUMO will use during the next simulation step.
commanded_speed = TARGET_SPEED


# ============================================================
# ANGLE HELPERS
# ============================================================

def normalize_angle(angle):
    """Normalize the angle to the range -180 to +180 degrees."""

    while angle > 180:
        angle -= 360

    while angle < -180:
        angle += 360

    return angle


def convert_sumo_angle(sumo_angle):
    """
    Convert SUMO heading into the Cartesian convention used by
    the tracker and bicycle model.

    SUMO:
        0°  = north
        90° = east

    Cartesian:
        0°  = east
        90° = north
    """

    return normalize_angle(
        90.0 - sumo_angle
    )


def convert_cartesian_angle_to_sumo(cartesian_angle):
    """Convert Cartesian heading back to SUMO heading."""

    return normalize_angle(
        90.0 - cartesian_angle
    )


# ============================================================
# REFERENCE PATH
# ============================================================

def get_route_path_points(route_edges):
    """
    Build the reference path from the actual SUMO lane shapes.
    """

    path_points = []

    for edge_id in route_edges:

        if edge_id.startswith(":"):
            continue

        lane_id = f"{edge_id}_0"

        try:
            shape = traci.lane.getShape(lane_id)

        except traci.TraCIException:
            continue

        for point in shape:

            if not path_points or point != path_points[-1]:
                path_points.append(point)

    return path_points


# ============================================================
# CURRENT JUNCTION
# ============================================================

def get_current_junction(edge_id):
    """Return the destination junction for a normal road edge."""

    for from_node in graph:

        for to_node in graph[from_node]:

            road = graph[from_node][to_node]

            if road["edge_id"] == edge_id:
                return to_node

    return None


# ============================================================
# SPEED CONTROL
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
# GOAL CHECK
# ============================================================

def reached_goal(model_x, model_y):
    """
    Check whether the bicycle-model vehicle has reached the end
    of the current reference path.

    We use the final reference-path point rather than a hard-coded
    junction coordinate. This keeps the check correct even when
    a reroute changes the reference path.
    """

    if len(reference_path) < 1:
        return False

    goal_point = reference_path[-1]

    dx = model_x - goal_point[0]
    dy = model_y - goal_point[1]

    distance_to_goal = (dx * dx + dy * dy) ** 0.5

    return distance_to_goal <= GOAL_DISTANCE_THRESHOLD


# ============================================================
# START SUMO
# ============================================================

sumo_cmd = [
    "sumo",
    "-c",
    SUMO_CFG
]

print("Starting SUMO...")

traci.start(sumo_cmd)

print("Connected to SUMO!")
print()


# ============================================================
# SIMULATION
# ============================================================

try:

    # ========================================================
    # WARM-UP
    # ========================================================

    print("Building background traffic...")

    warmup_steps = int(10.0 / MODEL_DT)

    for _ in range(warmup_steps):
        traci.simulationStep()


    # ========================================================
    # TRAFFIC CONDITIONS
    # ========================================================

    print()
    print("Current traffic conditions:")
    print()

    for edge_id in traci.edge.getIDList():

        if edge_id.startswith(":"):
            continue

        vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
        average_speed = traci.edge.getLastStepMeanSpeed(edge_id)
        traffic_cost = get_traffic_cost(edge_id)

        print(
            f"Road {edge_id} | "
            f"Vehicles: {vehicle_count} | "
            f"Avg speed: {average_speed:.2f} m/s | "
            f"Cost: {traffic_cost:.2f}"
        )


    # ========================================================
    # INITIAL A*
    # ========================================================

    print()
    print("Running traffic-aware A*...")
    print()

    path, total_cost = plan_path(
        START_JUNCTION,
        GOAL_JUNCTION,
        edge_cost_function=get_traffic_cost
    )

    if not path:
        raise RuntimeError("No traffic-aware path found.")

    route_edges = junctions_to_edges(path)

    print("Traffic-aware junction path:", path)
    print("Traffic-aware SUMO route:", route_edges)
    print("Traffic-aware total cost:", f"{total_cost:.2f}")
    print()


    # ========================================================
    # MAIN CONTROL LOOP
    # ========================================================

    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIMULATION_TIME
    ):

        # ----------------------------------------------------
        # Synchronization improvement
        #
        # Before SUMO advances, give it the speed command that
        # our bicycle model requested on the previous cycle.
        # ----------------------------------------------------

        vehicle_ids = traci.vehicle.getIDList()

        if EGO_ID in vehicle_ids and vehicle_model is not None:

            try:

                traci.vehicle.setSpeed(
                    EGO_ID,
                    commanded_speed
                )

            except traci.TraCIException:
                pass


        # ====================================================
        # ADVANCE SUMO
        # ====================================================

        traci.simulationStep()

        current_time = traci.simulation.getTime()
        vehicle_ids = traci.vehicle.getIDList()


        # ====================================================
        # OBSTACLE DETECTION
        # ====================================================

        if (
            not obstacle_added
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

                # Force immediate route check.
                last_replan_time = (
                    current_time - REPLAN_INTERVAL
                )

                obstacle_added = True


        # ====================================================
        # WAIT FOR EGO VEHICLE
        # ====================================================

        if EGO_ID not in vehicle_ids:
            continue


        # ====================================================
        # INITIAL ROUTE ASSIGNMENT
        # ====================================================

        if not route_assigned:

            traci.vehicle.setRoute(
                EGO_ID,
                route_edges
            )

            # Disable SUMO's automatic longitudinal controller.
            traci.vehicle.setSpeedMode(
                EGO_ID,
                0
            )

            # Disable automatic lane changes.
            traci.vehicle.setLaneChangeMode(
                EGO_ID,
                0
            )

            route_assigned = True
            last_replan_time = current_time

            # Build reference path.
            reference_path = get_route_path_points(route_edges)

            # IMPORTANT:
            # Start the path tracker from segment 0 for the
            # newly created reference path.
            reset_tracker()

            # Get initial state from SUMO once.
            initial_position = traci.vehicle.getPosition(EGO_ID)
            initial_sumo_angle = traci.vehicle.getAngle(EGO_ID)

            initial_heading = convert_sumo_angle(
                initial_sumo_angle
            )

            # Initialize bicycle model with our controlled speed.
            vehicle_model = BicycleModel(
                x=initial_position[0],
                y=initial_position[1],
                heading=initial_heading,
                speed=TARGET_SPEED,
                wheelbase=WHEELBASE
            )

            commanded_speed = TARGET_SPEED

            print()
            print(
                "Route assigned to egoVehicle:",
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
        # DYNAMIC REROUTING
        # ====================================================

        if (
            current_time - last_replan_time
            >= REPLAN_INTERVAL
        ):

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

                    new_path, new_cost = plan_path(
                        current_junction,
                        GOAL_JUNCTION,
                        edge_cost_function=get_traffic_cost
                    )

                    if new_path:

                        print(
                            "New junction path:",
                            new_path
                        )

                        remaining_edges = junctions_to_edges(
                            new_path
                        )

                        # Avoid duplicate current edge.
                        if (
                            remaining_edges
                            and remaining_edges[0] == current_edge
                        ):

                            new_route_edges = remaining_edges

                        else:

                            new_route_edges = (
                                [current_edge]
                                + remaining_edges
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

                                route_edges = new_route_edges

                                # Rebuild reference path for the
                                # new route.
                                reference_path = (
                                    get_route_path_points(
                                        route_edges
                                    )
                                )

                                # IMPORTANT:
                                # The path has changed, so reset
                                # the progress-aware tracker.
                                reset_tracker()

                                print(
                                    "Dynamic reroute applied:",
                                    new_route_edges
                                )

                                print(
                                    "Reference path updated."
                                )

                            except traci.TraCIException as error:

                                print(
                                    "Could not apply reroute:",
                                    error
                                )

                        else:

                            print(
                                "Current route is still suitable."
                            )

                    else:

                        print(
                            "No route available."
                        )

            last_replan_time = current_time


        # ====================================================
        # GET CURRENT BICYCLE MODEL STATE
        # ====================================================

        (
            model_x,
            model_y,
            model_heading,
            model_speed
        ) = vehicle_model.get_state()


        # ====================================================
        # REACH-GOAL CHECK
        # ====================================================

        if reached_goal(model_x, model_y):

            print()
            print("========================================")
            print("GOAL REACHED")

            print(
                f"Final position: ({model_x:.2f}, {model_y:.2f})"
            )

            print("========================================")

            # Stop the SUMO vehicle.
            try:

                traci.vehicle.setSpeed(
                    EGO_ID,
                    0.0
                )

                # Keep the displayed SUMO vehicle at the
                # final model state.
                traci.vehicle.moveToXY(
                    EGO_ID,
                    "",
                    -1,
                    model_x,
                    model_y,
                    convert_cartesian_angle_to_sumo(
                        model_heading
                    ),
                    1
                )

            except traci.TraCIException:
                pass

            break


        # ====================================================
        # REQUIRE A VALID REFERENCE PATH
        # ====================================================

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


        # ====================================================
        # STEERING CONTROLLER
        # ====================================================

        steering_angle = calculate_steering_angle(
            cross_track_error,
            heading_error,
            model_speed
        )


        # ====================================================
        # SPEED CONTROL
        # ====================================================

        desired_speed = calculate_model_speed(
            steering_angle
        )


        # ====================================================
        # UPDATE BICYCLE MODEL
        # ====================================================

        vehicle_model.update(
            steering_angle=steering_angle,
            speed=desired_speed,
            dt=MODEL_DT
        )


        # ====================================================
        # GET NEW MODEL STATE
        # ====================================================

        (
            new_x,
            new_y,
            new_heading,
            new_speed
        ) = vehicle_model.get_state()


        # ====================================================
        # CONVERT HEADING TO SUMO FORMAT
        # ====================================================

        new_sumo_angle = convert_cartesian_angle_to_sumo(
            new_heading
        )


        # ====================================================
        # SEND BICYCLE STATE TO SUMO
        # ====================================================

        try:

            # Use the same speed for the next SUMO step.
            commanded_speed = new_speed

            traci.vehicle.setSpeed(
                EGO_ID,
                new_speed
            )

            # Display the state produced by the bicycle model.
            traci.vehicle.moveToXY(
                EGO_ID,
                "",
                -1,
                new_x,
                new_y,
                new_sumo_angle,
                1
            )

        except traci.TraCIException as error:

            print(
                "Bicycle model TraCI error:",
                error
            )

            break


        # ====================================================
        # DISPLAY
        # ====================================================

        print(
            f"Time: {current_time:6.1f}s | "
            f"Position: ({new_x:7.2f}, {new_y:7.2f}) | "
            f"Speed: {new_speed:5.2f} m/s | "
            f"SUMO Angle: {new_sumo_angle:7.2f}° | "
            f"CTE: {cross_track_error:7.2f} m | "
            f"Heading Error: {heading_error:7.2f}° | "
            f"Steering: {steering_angle:7.2f}° | "
            f"Target: {closest_index}"
        )


# ============================================================
# CLEANUP
# ============================================================

finally:

    traci.close()

    print()
    print("Simulation finished.")