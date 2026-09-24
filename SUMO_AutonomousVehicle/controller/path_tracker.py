"""
path_tracker.py

Progress-aware path tracking.

Calculates:
    1. Signed Cross-Track Error
    2. Heading Error
    3. Current path segment

The tracker keeps track of its progress along the route so
it does not jump between unrelated path segments near
junctions or at the end of the route.
"""

import math


# ============================================================
# PARAMETERS
# ============================================================

LOOKAHEAD_DISTANCE = 8.0

# Number of segments we allow the tracker to look ahead.
# It can move forward, but will not jump far backward.
SEGMENT_LOOKAHEAD_COUNT = 3


# ============================================================
# TRACKER STATE
# ============================================================

current_segment_index = 0


def reset_tracker():
    """
    Reset route progress.

    Call this whenever a new reference path is created,
    especially after dynamic rerouting.
    """

    global current_segment_index

    current_segment_index = 0


# ============================================================
# DISTANCE
# ============================================================

def distance(point1, point2):

    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]

    return math.sqrt(
        dx ** 2 + dy ** 2
    )


# ============================================================
# CLOSEST POINT ON SEGMENT
# ============================================================

def closest_point_on_segment(
    vehicle_position,
    point1,
    point2
):

    x = vehicle_position[0]
    y = vehicle_position[1]

    x1 = point1[0]
    y1 = point1[1]

    x2 = point2[0]
    y2 = point2[1]

    dx = x2 - x1
    dy = y2 - y1

    segment_length_squared = (
        dx * dx +
        dy * dy
    )

    # Zero-length segment
    if segment_length_squared == 0:

        error = distance(
            vehicle_position,
            point1
        )

        return (
            point1,
            0.0,
            error,
            0.0
        )

    # --------------------------------------------------------
    # Projection
    # --------------------------------------------------------

    t = (
        (x - x1) * dx +
        (y - y1) * dy
    ) / segment_length_squared

    t = max(
        0.0,
        min(1.0, t)
    )

    closest_x = (
        x1 +
        t * dx
    )

    closest_y = (
        y1 +
        t * dy
    )

    closest_point = (
        closest_x,
        closest_y
    )

    # --------------------------------------------------------
    # Distance
    # --------------------------------------------------------

    unsigned_distance = distance(
        vehicle_position,
        closest_point
    )

    # --------------------------------------------------------
    # Left / right sign
    # --------------------------------------------------------

    cross_product = (
        dx * (y - y1)
        -
        dy * (x - x1)
    )

    if cross_product > 0:

        signed_distance = unsigned_distance

    elif cross_product < 0:

        signed_distance = -unsigned_distance

    else:

        signed_distance = 0.0

    return (
        closest_point,
        signed_distance,
        unsigned_distance,
        t
    )


# ============================================================
# FIND CLOSEST SEGMENT WITH PROGRESS
# ============================================================

def find_closest_segment(
    vehicle_position,
    path_points
):
    """
    Find the closest segment while respecting route progress.

    The tracker searches:

        current segment
        +
        a few segments ahead

    It does not search arbitrarily far backward.
    """

    global current_segment_index

    if len(path_points) < 2:

        return (
            None,
            0.0,
            -1,
            0.0
        )

    # --------------------------------------------------------
    # Keep index valid
    # --------------------------------------------------------

    max_segment_index = (
        len(path_points) - 2
    )

    current_segment_index = max(
        0,
        min(
            current_segment_index,
            max_segment_index
        )
    )

    # --------------------------------------------------------
    # Search window
    # --------------------------------------------------------

    start_index = current_segment_index

    end_index = min(
        max_segment_index,
        current_segment_index
        +
        SEGMENT_LOOKAHEAD_COUNT
    )

    minimum_distance = float("inf")

    best_point = None
    best_signed_error = 0.0
    best_segment_index = start_index
    best_projection_ratio = 0.0

    # --------------------------------------------------------
    # Search only current and forward segments
    # --------------------------------------------------------

    for i in range(
        start_index,
        end_index + 1
    ):

        point1 = path_points[i]

        point2 = path_points[i + 1]

        (
            closest_point,
            signed_distance,
            unsigned_distance,
            projection_ratio
        ) = closest_point_on_segment(
            vehicle_position,
            point1,
            point2
        )

        if unsigned_distance < minimum_distance:

            minimum_distance = (
                unsigned_distance
            )

            best_point = closest_point

            best_signed_error = signed_distance

            best_segment_index = i

            best_projection_ratio = (
                projection_ratio
            )

    # --------------------------------------------------------
    # Update progress
    # --------------------------------------------------------

    if best_segment_index >= current_segment_index:

        current_segment_index = (
            best_segment_index
        )

    return (
        best_point,
        best_signed_error,
        current_segment_index,
        best_projection_ratio
    )


# ============================================================
# FIND LOOK-AHEAD POINT
# ============================================================

def find_lookahead_point(
    path_points,
    closest_segment_index,
    closest_point,
    projection_ratio,
    lookahead_distance=LOOKAHEAD_DISTANCE
):

    if len(path_points) < 2:

        return None, -1

    if closest_segment_index < 0:

        closest_segment_index = 0

    max_segment_index = (
        len(path_points) - 2
    )

    if closest_segment_index > max_segment_index:

        closest_segment_index = max_segment_index

    current_point = path_points[
        closest_segment_index
    ]

    next_point = path_points[
        closest_segment_index + 1
    ]

    current_segment_length = distance(
        current_point,
        next_point
    )

    remaining_current_segment = (
        current_segment_length
        *
        (
            1.0 -
            projection_ratio
        )
    )

    remaining_distance = (
        lookahead_distance
    )

    # --------------------------------------------------------
    # Look-ahead remains on current segment
    # --------------------------------------------------------

    if (
        remaining_distance
        <=
        remaining_current_segment
    ):

        if current_segment_length > 0:

            ratio = (
                remaining_distance
                /
                current_segment_length
            )

        else:

            ratio = 0.0

        lookahead_x = (
            closest_point[0]
            +
            ratio *
            (
                next_point[0]
                -
                closest_point[0]
            )
        )

        lookahead_y = (
            closest_point[1]
            +
            ratio *
            (
                next_point[1]
                -
                closest_point[1]
            )
        )

        return (
            (lookahead_x, lookahead_y),
            closest_segment_index
        )

    # --------------------------------------------------------
    # Move into later segments
    # --------------------------------------------------------

    remaining_distance -= (
        remaining_current_segment
    )

    segment_index = (
        closest_segment_index + 1
    )

    while (
        segment_index
        <
        len(path_points) - 1
    ):

        point1 = path_points[
            segment_index
        ]

        point2 = path_points[
            segment_index + 1
        ]

        segment_length = distance(
            point1,
            point2
        )

        if segment_length >= remaining_distance:

            if segment_length > 0:

                ratio = (
                    remaining_distance
                    /
                    segment_length
                )

            else:

                ratio = 0.0

            x = (
                point1[0]
                +
                ratio *
                (
                    point2[0]
                    -
                    point1[0]
                )
            )

            y = (
                point1[1]
                +
                ratio *
                (
                    point2[1]
                    -
                    point1[1]
                )
            )

            return (
                (x, y),
                segment_index
            )

        remaining_distance -= (
            segment_length
        )

        segment_index += 1

    # --------------------------------------------------------
    # Beyond the end of path
    # --------------------------------------------------------

    return (
        path_points[-1],
        len(path_points) - 2
    )


# ============================================================
# HEADING BETWEEN TWO POINTS
# ============================================================

def calculate_heading_between_points(
    point1,
    point2
):

    dx = (
        point2[0]
        -
        point1[0]
    )

    dy = (
        point2[1]
        -
        point1[1]
    )

    return math.degrees(
        math.atan2(
            dy,
            dx
        )
    )


# ============================================================
# DESIRED HEADING
# ============================================================

def calculate_desired_heading(
    path_points,
    segment_index
):

    if len(path_points) < 2:

        return 0.0

    segment_index = max(
        0,
        min(
            segment_index,
            len(path_points) - 2
        )
    )

    return calculate_heading_between_points(
        path_points[segment_index],
        path_points[segment_index + 1]
    )


# ============================================================
# HEADING ERROR
# ============================================================

def calculate_heading_error(
    vehicle_angle,
    desired_heading
):

    error = (
        desired_heading
        -
        vehicle_angle
    )

    while error > 180:

        error -= 360

    while error < -180:

        error += 360

    return error


# ============================================================
# COMPLETE TRACKING ERROR
# ============================================================

def calculate_tracking_error(
    vehicle_position,
    vehicle_angle,
    path_points
):
    """
    Main tracking function.

    Returns:

        signed CTE
        heading error
        current segment index
    """

    if len(path_points) < 2:

        return (
            0.0,
            0.0,
            -1
        )

    # --------------------------------------------------------
    # Find closest segment with progress tracking
    # --------------------------------------------------------

    (
        cross_track_error,
        closest_segment_index,
        closest_point,
        projection_ratio
    ) = calculate_cross_track_error(
        vehicle_position,
        path_points
    )

    # --------------------------------------------------------
    # Find look-ahead point
    # --------------------------------------------------------

    (
        lookahead_point,
        lookahead_segment_index
    ) = find_lookahead_point(

        path_points,

        closest_segment_index,

        closest_point,

        projection_ratio,

        LOOKAHEAD_DISTANCE
    )

    # --------------------------------------------------------
    # Calculate desired heading
    # --------------------------------------------------------

    if lookahead_point is not None:

        if (
            lookahead_segment_index
            <
            len(path_points) - 2
        ):

            next_point = path_points[
                lookahead_segment_index + 1
            ]

            desired_heading = (
                calculate_heading_between_points(
                    lookahead_point,
                    next_point
                )
            )

        else:

            desired_heading = (
                calculate_desired_heading(
                    path_points,
                    lookahead_segment_index
                )
            )

    else:

        desired_heading = (
            calculate_desired_heading(
                path_points,
                closest_segment_index
            )
        )

    # --------------------------------------------------------
    # Heading error
    # --------------------------------------------------------

    heading_error = (
        calculate_heading_error(
            vehicle_angle,
            desired_heading
        )
    )

    return (
        cross_track_error,
        heading_error,
        closest_segment_index
    )


# ============================================================
# CROSS-TRACK ERROR WRAPPER
# ============================================================

def calculate_cross_track_error(
    vehicle_position,
    path_points
):

    (
        closest_point,
        signed_error,
        segment_index,
        projection_ratio
    ) = find_closest_segment(
        vehicle_position,
        path_points
    )

    return (
        signed_error,
        segment_index,
        closest_point,
        projection_ratio
    )


# ============================================================
# SIMPLE CLOSEST POINT FUNCTION
# ============================================================

def find_closest_point(
    vehicle_position,
    path_points
):

    if not path_points:

        return None, -1

    closest_point = None
    closest_index = -1

    minimum_distance = float("inf")

    for i, point in enumerate(path_points):

        current_distance = distance(
            vehicle_position,
            point
        )

        if current_distance < minimum_distance:

            minimum_distance = (
                current_distance
            )

            closest_point = point

            closest_index = i

    return (
        closest_point,
        closest_index
    )


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    # Straight + turning path
    path_points = [
        (0, 0),
        (50, 0),
        (100, 0),
        (100, 50),
        (100, 100)
    ]

    reset_tracker()

    print("Progress-Aware Path Tracker Test")
    print("================================")
    print()

    # Test 1
    vehicle_position = (
        50,
        10
    )

    (
        cte,
        heading_error,
        segment
    ) = calculate_tracking_error(
        vehicle_position,
        0.0,
        path_points
    )

    print("Vehicle above path:")
    print(f"CTE: {cte:.2f} m")
    print(f"Heading error: {heading_error:.2f}°")
    print(f"Segment: {segment}")

    # Test 2
    vehicle_position = (
        50,
        -10
    )

    (
        cte,
        heading_error,
        segment
    ) = calculate_tracking_error(
        vehicle_position,
        0.0,
        path_points
    )

    print()
    print("Vehicle below path:")
    print(f"CTE: {cte:.2f} m")
    print(f"Heading error: {heading_error:.2f}°")
    print(f"Segment: {segment}")