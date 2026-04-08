# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 28th March 2026

"""
Spot Micro tail controller.

Inherits from the abstract Tail base class and implements the
control law and servo command pipeline for the Spot Micro's tail.
Uses two servos (left and right), each on a separate PCA9685 board.
"""

from tail import Tail
from servo import Servo
from spot_config import TAIL


class SpotMicroTail(Tail):

    def __init__(self, pca_left, pca_right):
        """
        Initialize the SpotMicroTail.

        Loads configuration from spot_config.py and passes it,
        along with both PCA9685 drivers, to the parent Tail class.
        Creates Servo objects for the left and right tail actuators.

        Parameters
        ----------
        pca_left : adafruit_pca9685.PCA9685
            The PCA9685 driver object for the left tail servo.
        pca_right : adafruit_pca9685.PCA9685
            The PCA9685 driver object for the right tail servo.
        """
        super().__init__(pca_left, pca_right, TAIL)

        servo_cfg = self.config["servos"]

        self.left_servo = Servo(
            pca=self.pca_left,
            channel=servo_cfg["left"]["channel"],
            min_pulse=servo_cfg["left"]["min_pulse"],
            max_pulse=servo_cfg["left"]["max_pulse"],
            min_angle=servo_cfg["left"]["min_angle"],
            max_angle=servo_cfg["left"]["max_angle"],
        )
        self.right_servo = Servo(
            pca=self.pca_right,
            channel=servo_cfg["right"]["channel"],
            min_pulse=servo_cfg["right"]["min_pulse"],
            max_pulse=servo_cfg["right"]["max_pulse"],
            min_angle=servo_cfg["right"]["min_angle"],
            max_angle=servo_cfg["right"]["max_angle"],
        )

        # --- Safety limits ---
        limits = self.config["safety_limits"]
        self.servo_limits = {
            "left":  (limits["left"]["min"],  limits["left"]["max"]),
            "right": (limits["right"]["min"], limits["right"]["max"]),
        }

        # --- PID gains ---
        self.kp = self.config["pid_gains"]["kp"]
        self.ki = self.config["pid_gains"]["ki"]
        self.kd = self.config["pid_gains"]["kd"]

    def initialize(self):
        """
        Command tail servos to their neutral positions from config.

        WARNING: Hold the quadruped in the air before calling this
        method to prevent damage or unexpected motion.
        """
        init_cfg = self.config["initialization"]

        self.left_servo.command(init_cfg["servo_left_angle"])
        self.right_servo.command(init_cfg["servo_right_angle"])

    def compute_control(self, roll, pitch, yaw):
        """
        Compute the commanded tail servo angles based on chassis orientation.

        Implements the control law (e.g. PID) that maps the current
        roll, pitch, and yaw of the chassis to desired servo angles
        for the left and right tail actuators.

        TODO: Implement control law using self.kp, self.ki, self.kd.

        Parameters
        ----------
        roll : float
            Chassis roll angle in degrees.
        pitch : float
            Chassis pitch angle in degrees.
        yaw : float
            Chassis yaw angle in degrees.

        Returns
        -------
        tuple of (float, float)
            (left_angle, right_angle) — commanded servo angles in degrees.
        """
        # Placeholder — returns neutral angles
        init_cfg = self.config["initialization"]
        left_angle = init_cfg["servo_left_angle"]
        right_angle = init_cfg["servo_right_angle"]

        return left_angle, right_angle

    def assert_safety_limits(self, left_angle, right_angle):
        """
        Check that servo commands are within safety thresholds.

        Raises a RuntimeError if any servo command falls outside
        the allowable range defined in the tail config.

        Parameters
        ----------
        left_angle : float
            Commanded left tail servo angle (degrees).
        right_angle : float
            Commanded right tail servo angle (degrees).
        """
        left_min, left_max = self.servo_limits["left"]
        right_min, right_max = self.servo_limits["right"]

        if not (left_min <= left_angle <= left_max):
            raise RuntimeError(
                f"SAFETY LIMIT VIOLATED: Left tail servo angle {left_angle:.2f}° "
                f"is outside [{left_min:.2f}°, {left_max:.2f}°]"
            )
        if not (right_min <= right_angle <= right_max):
            raise RuntimeError(
                f"SAFETY LIMIT VIOLATED: Right tail servo angle {right_angle:.2f}° "
                f"is outside [{right_min:.2f}°, {right_max:.2f}°]"
            )

    def move_tail(self, roll, pitch, yaw):
        """
        Compute control and command the tail servos.

        Pipeline: compute_control → safety check → servo commands.

        Parameters
        ----------
        roll : float
            Chassis roll angle in degrees.
        pitch : float
            Chassis pitch angle in degrees.
        yaw : float
            Chassis yaw angle in degrees.

        Returns
        -------
        tuple of (tuple, tuple)
            ((roll, pitch, yaw),
             (left_angle, right_angle))
            Input RPY and commanded servo angles, for logging.
        """
        left_angle, right_angle = self.compute_control(roll, pitch, yaw)

        self.assert_safety_limits(left_angle, right_angle)

        self.left_servo.command(left_angle)
        self.right_servo.command(right_angle)

        return (left_angle, right_angle)
