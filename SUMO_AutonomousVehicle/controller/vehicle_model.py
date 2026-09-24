"""
vehicle_model.py

Kinematic bicycle model.

The model uses:

    speed
    steering angle
    wheelbase

to calculate:

    x position
    y position
    heading
"""


import math


# ============================================================
# BICYCLE MODEL
# ============================================================

class BicycleModel:

    def __init__(
        self,
        x,
        y,
        heading,
        speed=0.0,
        wheelbase=2.5
    ):
        """
        Initialize bicycle model.

        Parameters:
            x          -> vehicle X position
            y          -> vehicle Y position
            heading    -> vehicle heading in degrees
            speed      -> vehicle speed in m/s
            wheelbase  -> distance between axles
        """

        self.x = x

        self.y = y

        self.heading = heading

        self.speed = speed

        self.wheelbase = wheelbase


    # ========================================================
    # UPDATE MODEL
    # ========================================================

    def update(
        self,
        steering_angle,
        speed,
        dt
    ):
        """
        Update vehicle state using the kinematic bicycle model.

        Equations:

            x_dot = v cos(theta)

            y_dot = v sin(theta)

            theta_dot = v/L tan(delta)
        """

        # ----------------------------------------------------
        # Store current speed
        # ----------------------------------------------------

        self.speed = speed


        # ----------------------------------------------------
        # Convert degrees to radians
        # ----------------------------------------------------

        theta = math.radians(
            self.heading
        )

        delta = math.radians(
            steering_angle
        )


        # ----------------------------------------------------
        # Vehicle velocity
        # ----------------------------------------------------

        x_dot = (
            self.speed
            *
            math.cos(theta)
        )

        y_dot = (
            self.speed
            *
            math.sin(theta)
        )


        # ----------------------------------------------------
        # Vehicle heading rate
        # ----------------------------------------------------

        theta_dot = (
            self.speed
            /
            self.wheelbase
        ) * math.tan(delta)


        # ----------------------------------------------------
        # Integrate over timestep
        # ----------------------------------------------------

        self.x += (
            x_dot * dt
        )

        self.y += (
            y_dot * dt
        )

        theta += (
            theta_dot * dt
        )


        # ----------------------------------------------------
        # Convert heading back to degrees
        # ----------------------------------------------------

        self.heading = math.degrees(
            theta
        )


        # ----------------------------------------------------
        # Normalize heading
        # ----------------------------------------------------

        while self.heading > 180:

            self.heading -= 360

        while self.heading < -180:

            self.heading += 360


    # ========================================================
    # RETURN CURRENT STATE
    # ========================================================

    def get_state(self):

        return (
            self.x,
            self.y,
            self.heading,
            self.speed
        )


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    vehicle = BicycleModel(
        x=0.0,
        y=0.0,
        heading=0.0,
        speed=10.0,
        wheelbase=2.5
    )


    print("Bicycle Model Test")
    print("-------------------")


    # Apply left steering for 1 second.
    vehicle.update(
        steering_angle=10.0,
        speed=10.0,
        dt=1.0
    )


    x, y, heading, speed = (
        vehicle.get_state()
    )


    print(
        f"X       : {x:.2f} m"
    )

    print(
        f"Y       : {y:.2f} m"
    )

    print(
        f"Heading : {heading:.2f}°"
    )

    print(
        f"Speed   : {speed:.2f} m/s"
    )