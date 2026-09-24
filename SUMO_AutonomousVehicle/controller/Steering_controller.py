"""
Steering_controller.py

Stanley-style steering controller.

Inputs:
    - Signed Cross-Track Error
    - Heading Error
    - Vehicle Speed

Output:
    - Steering Angle
"""

import math


# ============================================================
# CONTROLLER PARAMETERS
# ============================================================

K_CTE = 0.2

MIN_SPEED = 0.1

MAX_STEERING_ANGLE = 25.0


# ============================================================
# ANGLE NORMALIZATION
# ============================================================

def normalize_angle(angle):
    """
    Normalize angle to:

        -180° to +180°
    """

    while angle > 180:

        angle -= 360

    while angle < -180:

        angle += 360

    return angle


# ============================================================
# STEERING CONTROLLER
# ============================================================

def calculate_steering_angle(
    cross_track_error,
    heading_error,
    speed,
    k_cte=K_CTE
):
    """
    Stanley-style path tracking controller.

    CTE convention:

        + CTE -> vehicle left of path
        - CTE -> vehicle right of path

    Steering convention:

        + steering -> left
        - steering -> right
    """

    # --------------------------------------------------------
    # 1. Protect against zero speed
    # --------------------------------------------------------

    speed = max(
        speed,
        MIN_SPEED
    )


    # --------------------------------------------------------
    # 2. Normalize heading error
    # --------------------------------------------------------

    heading_error = normalize_angle(
        heading_error
    )


    # --------------------------------------------------------
    # 3. Convert heading error to radians
    # --------------------------------------------------------

    heading_error_rad = math.radians(
        heading_error
    )


    # --------------------------------------------------------
    # 4. Cross-track correction
    # --------------------------------------------------------

    cte_correction_rad = math.atan2(
        k_cte * cross_track_error,
        speed
    )


    # --------------------------------------------------------
    # 5. Combine both corrections
    #
    # Positive CTE means vehicle is left of path,
    # therefore steering should act toward the right.
    # --------------------------------------------------------

    steering_rad = (
        heading_error_rad
        -
        cte_correction_rad
    )


    # --------------------------------------------------------
    # 6. Convert to degrees
    # --------------------------------------------------------

    steering_angle = math.degrees(
        steering_rad
    )


    # --------------------------------------------------------
    # 7. Limit steering
    # --------------------------------------------------------

    steering_angle = max(
        -MAX_STEERING_ANGLE,
        min(
            MAX_STEERING_ANGLE,
            steering_angle
        )
    )


    return steering_angle


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    print("Steering Controller Test")
    print("------------------------")


    # ========================================================
    # TEST 1
    # Vehicle is LEFT of path.
    # Expected: steer RIGHT.
    # ========================================================

    cte = 2.0

    heading_error = 0.0

    speed = 10.0


    steering = calculate_steering_angle(
        cte,
        heading_error,
        speed
    )


    print()
    print("Test 1: Vehicle LEFT of path")

    print(
        f"CTE            : {cte:.2f} m"
    )

    print(
        f"Heading error  : {heading_error:.2f}°"
    )

    print(
        f"Speed          : {speed:.2f} m/s"
    )

    print(
        f"Steering       : {steering:.2f}°"
    )


    # ========================================================
    # TEST 2
    # Vehicle is RIGHT of path.
    # Expected: steer LEFT.
    # ========================================================

    cte = -2.0


    steering = calculate_steering_angle(
        cte,
        heading_error,
        speed
    )


    print()
    print("Test 2: Vehicle RIGHT of path")

    print(
        f"CTE            : {cte:.2f} m"
    )

    print(
        f"Heading error  : {heading_error:.2f}°"
    )

    print(
        f"Speed          : {speed:.2f} m/s"
    )

    print(
        f"Steering       : {steering:.2f}°"
    )