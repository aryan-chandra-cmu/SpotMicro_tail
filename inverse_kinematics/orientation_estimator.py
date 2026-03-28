# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 28th March 2026

"""
Orientation estimator for the Spot Micro quadruped.

Uses a BNO055 IMU to provide roll, pitch, and yaw angles
of the chassis. Shares the I2C bus initialized in main.py.
"""

import adafruit_bno055


class OrientationEstimator:
    """
    Reads chassis orientation from a BNO055 IMU over I2C.

    The BNO055 provides fused Euler angles (heading, roll, pitch)
    using its on-board sensor fusion processor.
    """

    def __init__(self, i2c, address=0x28):
        """
        Initialize the BNO055 IMU.

        Parameters
        ----------
        i2c : busio.I2C
            The shared I2C bus object (created in main.py).
        address : int, optional
            I2C address of the BNO055 sensor. Default is 0x28.
        """
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)

    def get_rpy(self):
        """
        Read the current roll, pitch, and yaw angles from the IMU.

        The BNO055 returns Euler angles as (heading, roll, pitch).
        This method reorders them to (roll, pitch, yaw) for
        consistency with common robotics conventions.

        Returns
        -------
        tuple of (float, float, float)
            (roll, pitch, yaw) in degrees.
            Returns (0.0, 0.0, 0.0) if the sensor reading is unavailable.
        """
        euler = self.sensor.euler  # returns (heading, roll, pitch)

        if euler is None:
            return (0.0, 0.0, 0.0)

        yaw, roll, pitch = euler
        return (roll, pitch, yaw)
