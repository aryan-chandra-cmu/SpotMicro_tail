# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 19th March 2026

"""
Abstract base class for a quadruped leg.

Defines the interface that every leg (RightFrontLeg, LeftFrontLeg, etc.)
must implement. Subclasses handle servo instantiation and provide
the method to command all three joint servos from a target foot position.
"""

from abc import ABC, abstractmethod


class Leg(ABC):
    """
    Abstract leg template for the Spot Micro quadruped.

    Subclasses must implement:
      - compute_ik(x, y, z) -> (theta_hip, theta_shoulder, theta_knee)
      - joint_angles_to_motor_angles(theta_hip, theta_shoulder, theta_knee)
            -> (motor_hip, motor_shoulder, motor_knee)
      - command_servos(x, y, z)
    """

    def __init__(self, pca, leg_config):
        """
        Initialize a Leg instance.

        Stores the PCA9685 driver and leg configuration. Subclasses
        are responsible for creating their own Servo objects.

        Parameters
        ----------
        pca : adafruit_pca9685.PCA9685
            The PCA9685 driver object that controls this leg's servos.
        leg_config : dict
            Configuration dictionary for this leg (see config/leg_config.py).
            Must contain 'link_lengths' and 'servos' keys.
        """
        self.pca = pca
        self.config = leg_config
        self.link_lengths = leg_config["link_lengths"]

    @abstractmethod
    def compute_ik(self, x, y, z):
        """
        Compute inverse kinematics for a desired foot position.

        Given a target end-effector position in the leg's local coordinate
        frame, compute the required joint angles.

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
        tuple of (float, float, float)
            (theta_hip, theta_shoulder, theta_knee) in degrees.
        """
        pass

    @abstractmethod
    def joint_angles_to_motor_angles(self, theta_hip, theta_shoulder, theta_knee):
        """
        Convert IK joint angles to actual motor/servo angles.

        Applies leg-specific offsets, sign conventions, and mechanical
        zero adjustments so the servo angles match the physical assembly.

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
        pass

    @abstractmethod
    def command_servos(self, x, y, z):
        """
        Command all three leg servos to reach a target foot position.

        Pipeline: compute_ik → joint_angles_to_motor_angles → servo commands.

        Parameters
        ----------
        x : float
            Target foot position along the forward axis (mm).
        y : float
            Target foot position along the vertical axis (mm).
        z : float
            Target foot position along the lateral axis (mm).
        """
        pass
