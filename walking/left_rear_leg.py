# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 28th March 2026

"""
Left rear leg for the Spot Micro quadruped.

Inherits from the abstract Leg base class and implements the
inverse kinematics and joint-to-motor angle conversion specific
to the left rear leg's geometry and servo orientation.
"""

import math

from leg import Leg
from servo import Servo
from spot_config import LEFT_REAR_LEG


class LeftRearLeg(Leg):

    def __init__(self, pca):
        """
        Initialize the LeftRearLeg.

        Loads configuration from config/leg_config.py and passes it,
        along with the PCA9685 driver, to the parent Leg class.
        Creates Servo objects for hip, shoulder, and knee.

        Parameters
        ----------
        pca : adafruit_pca9685.PCA9685
            The PCA9685 driver object controlling this leg's servos.
        """
        super().__init__(pca, LEFT_REAR_LEG)

        servo_cfg = self.config["servos"]
        init_cfg = self.config["initialization"]

        # Servo zeroing offsets (deg) — compensate for mechanical misalignment
        self.offset_hip = init_cfg["servo_offset_hip"]
        self.offset_shoulder = init_cfg["servo_offset_shoulder"]
        self.offset_knee = init_cfg["servo_offset_knee"]

        self.hip_servo = Servo(
            pca=self.pca,
            channel=servo_cfg["hip"]["channel"],
            min_pulse=servo_cfg["hip"]["min_pulse"],
            max_pulse=servo_cfg["hip"]["max_pulse"],
            min_angle=servo_cfg["hip"]["min_angle"],
            max_angle=servo_cfg["hip"]["max_angle"],
        )
        self.shoulder_servo = Servo(
            pca=self.pca,
            channel=servo_cfg["shoulder"]["channel"],
            min_pulse=servo_cfg["shoulder"]["min_pulse"],
            max_pulse=servo_cfg["shoulder"]["max_pulse"],
            min_angle=servo_cfg["shoulder"]["min_angle"],
            max_angle=servo_cfg["shoulder"]["max_angle"],
        )
        self.knee_servo = Servo(
            pca=self.pca,
            channel=servo_cfg["knee"]["channel"],
            min_pulse=servo_cfg["knee"]["min_pulse"],
            max_pulse=servo_cfg["knee"]["max_pulse"],
            min_angle=servo_cfg["knee"]["min_angle"],
            max_angle=servo_cfg["knee"]["max_angle"],
        )

        # --- Safety limits ---
        # Load joint angle limits from config and convert to servo angle limits
        limits = self.config["safety_limits"]

        self.joint_limits = {
            "knee":     (limits["knee"]["min"],     limits["knee"]["max"]),
            "hip":      (limits["hip"]["min"],      limits["hip"]["max"]),
            "shoulder": (limits["shoulder"]["min"], limits["shoulder"]["max"]),
        }

        # Convert joint angle extremes to servo angle extremes
        # For each joint, map both min and max through joint_angles_to_motor_angles
        # and take the resulting min/max (order may flip due to sign conventions)
        motor_hip_a, motor_shoulder_a, motor_knee_a = self.joint_angles_to_motor_angles(
            limits["hip"]["min"], limits["shoulder"]["min"], limits["knee"]["min"]
        )
        motor_hip_b, motor_shoulder_b, motor_knee_b = self.joint_angles_to_motor_angles(
            limits["hip"]["max"], limits["shoulder"]["max"], limits["knee"]["max"]
        )

        self.servo_limits = {
            "hip":      (min(motor_hip_a, motor_hip_b),           max(motor_hip_a, motor_hip_b)),
            "shoulder": (min(motor_shoulder_a, motor_shoulder_b), max(motor_shoulder_a, motor_shoulder_b)),
            "knee":     (min(motor_knee_a, motor_knee_b),         max(motor_knee_a, motor_knee_b)),
        }

    def initialize(self):
        """
        Command servos to their initial angles from config.

        Moves all three servos to the initialization angles defined
        in config/leg_config.py, corresponding to a straight leg
        configuration.  The servos jump to these positions at maximum
        speed because the starting position is unknown.

        WARNING: Hold the quadruped in the air before calling this
        method to prevent damage or unexpected motion.
        """
        init_cfg = self.config["initialization"]

        self.hip_servo.command(init_cfg["servo_hip_angle"])
        self.shoulder_servo.command(init_cfg["servo_shoulder_angle"])
        self.knee_servo.command(init_cfg["servo_knee_angle"])

    def compute_ik(self, x, y, z):
        """
        Compute inverse kinematics for the left rear leg.

        Given a target foot position in the left rear leg's local
        coordinate frame, compute the required joint angles using
        3-DOF geometric IK (hip abduction + shoulder/knee in the
        sagittal plane).

        Coordinate frame convention:
          - x: forward (positive = forward)
          - y: lateral (positive = inward to body)
          - z: downward (positive = downward)

        Uses link lengths from config:
          - l1: hip offset (lateral shoulder-to-hip distance)
          - l2: upper leg length
          - l3: lower leg length

        Parameters
        ----------
        x : float
            Target foot position along the forward axis (mm).
        y : float
            Target foot position along the lateral axis (mm).
        z : float
            Target foot position along the downward axis (mm).

        Returns
        -------
        tuple of (float, float, float)
            (theta_hip, theta_shoulder, theta_knee) in degrees.
        """
        
        # Link lengths from config
        upper_leg = self.config["link_lengths"]["l2"]
        lower_leg = self.config["link_lengths"]["l3"]
        shoulder_offset = self.config["link_lengths"]["l1"]

        # Quantity used to reduce 3D geometry to the hip-knee plane
        inside = y**2 + z**2 - shoulder_offset**2

        y1 = math.sqrt(inside)

        # Distance from hip joint to foot in the hip-knee plane
        distance = math.sqrt(x**2 + y1**2)

        # Reachability check
        if distance > upper_leg + lower_leg or distance < abs(upper_leg - lower_leg):
            raise ValueError("Target is outside the reachable workspace of the hip-knee chain.")

        # Clamp acos argument for numerical safety
        cos_knee = (distance**2 - upper_leg**2 - lower_leg**2) / (-2.0 * upper_leg * lower_leg)
        cos_knee = max(-1.0, min(1.0, cos_knee))

        knee = math.acos(cos_knee)

        # Obtain hip angle
        hip = math.asin((lower_leg * math.sin(knee)) / distance) + math.atan(-x / y1)

        # Obtain shoulder angle
        shoulder = math.atan2(y1, shoulder_offset) + math.atan2(-y, z)

        # Convert to degrees
        knee_deg = math.degrees(knee)
        hip_deg = math.degrees(hip)
        shoulder_deg = math.degrees(shoulder)

        return hip_deg, shoulder_deg, knee_deg

    def joint_angles_to_motor_angles(self, theta_hip, theta_shoulder, theta_knee):
        """
        Convert IK joint angles to servo motor angles for the left rear leg.

        Applies left-rear-specific offsets, sign flips, and mechanical
        zero adjustments so that the computed IK angles map correctly
        to the physical servo positions.

        Parameters
        ----------
        theta_hip : float
            Hip joint angle from IK (degrees).
        theta_shoulder : float
            Shoulder joint angle from IK (degrees).
        theta_knee : float
            Knee joint angle from IK (degrees).

        Returns
        -------
        tuple of (float, float, float)
            (motor_hip, motor_shoulder, motor_knee) in degrees,
            ready to be sent to the servos.
        """
        motor_knee = theta_knee + self.offset_knee
        motor_hip = theta_hip + 90.0 + self.offset_hip
        motor_shoulder = 180.0 - theta_shoulder + self.offset_shoulder

        return motor_hip, motor_shoulder, motor_knee

    def motor_angles_to_joint_angles(self, motor_hip, motor_shoulder, motor_knee):
        """
        Convert servo motor angles back to IK joint angles for the left rear leg.

        This is the inverse of joint_angles_to_motor_angles.

        Parameters
        ----------
        motor_hip : float
            Hip servo angle (degrees).
        motor_shoulder : float
            Shoulder servo angle (degrees).
        motor_knee : float
            Knee servo angle (degrees).

        Returns
        -------
        tuple of (float, float, float)
            (theta_hip, theta_shoulder, theta_knee) in degrees.
        """
        theta_knee = motor_knee - self.offset_knee
        theta_hip = motor_hip - 90.0 - self.offset_hip
        theta_shoulder = 180.0 - motor_shoulder + self.offset_shoulder

        return theta_hip, theta_shoulder, theta_knee

    def assert_safety_limits(self, motor_hip, motor_shoulder, motor_knee):
        """
        Check that servo commands are within safety thresholds.

        Raises a RuntimeError if any servo command falls outside
        the allowable range derived from the joint angle limits
        in config.

        Parameters
        ----------
        motor_hip : float
            Commanded hip servo angle (degrees).
        motor_shoulder : float
            Commanded shoulder servo angle (degrees).
        motor_knee : float
            Commanded knee servo angle (degrees).
        """
        hip_min, hip_max = self.servo_limits["hip"]
        shoulder_min, shoulder_max = self.servo_limits["shoulder"]
        knee_min, knee_max = self.servo_limits["knee"]

        if not (hip_min <= motor_hip <= hip_max):
            raise RuntimeError(
                f"SAFETY LIMIT VIOLATED: Hip servo angle {motor_hip:.2f}° "
                f"is outside [{hip_min:.2f}°, {hip_max:.2f}°]"
            )
        if not (shoulder_min <= motor_shoulder <= shoulder_max):
            raise RuntimeError(
                f"SAFETY LIMIT VIOLATED: Shoulder servo angle {motor_shoulder:.2f}° "
                f"is outside [{shoulder_min:.2f}°, {shoulder_max:.2f}°]"
            )
        if not (knee_min <= motor_knee <= knee_max):
            raise RuntimeError(
                f"SAFETY LIMIT VIOLATED: Knee servo angle {motor_knee:.2f}° "
                f"is outside [{knee_min:.2f}°, {knee_max:.2f}°]"
            )

    def command_servos(self, x, y, z):
        """
        Command all three leg servos to reach a target foot position.

        Pipeline: compute_ik → joint_angles_to_motor_angles → safety check → servo commands.

        Parameters
        ----------
        x : float
            Target foot position along the forward axis (mm).
        y : float
            Target foot position along the vertical axis (mm).
        z : float
            Target foot position along the lateral axis (mm).

        Returns
        -------
        tuple of (tuple, tuple)
            ((theta_hip, theta_shoulder, theta_knee),
             (motor_hip, motor_shoulder, motor_knee))
            Joint angles and servo motor angles in degrees.
        """
        theta_hip, theta_shoulder, theta_knee = self.compute_ik(x, y, z)
        motor_hip, motor_shoulder, motor_knee = self.joint_angles_to_motor_angles(theta_hip, theta_shoulder, theta_knee)

        self.assert_safety_limits(motor_hip, motor_shoulder, motor_knee)

        self.hip_servo.command(motor_hip)
        self.shoulder_servo.command(motor_shoulder)
        self.knee_servo.command(motor_knee)

        return (theta_hip, theta_shoulder, theta_knee), (motor_hip, motor_shoulder, motor_knee)
