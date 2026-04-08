# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 28th March 2026

"""
Abstract base class for a quadruped tail controller.

Defines the interface that every tail implementation must follow.
Subclasses provide the control logic (compute_control) and
the servo command pipeline (move_tail).
"""

from abc import ABC, abstractmethod


class Tail(ABC):
    """
    Abstract tail template for the Spot Micro quadruped.

    The tail uses two PCA9685 driver boards (left and right) to
    control servos that actuate the tail mechanism.  The tail
    reacts to chassis orientation (roll, pitch, yaw) via a
    PID-based control law implemented by subclasses.

    Subclasses must implement:
      - compute_control(roll, pitch, yaw)
            -> dict of servo name -> commanded angle
      - move_tail(roll, pitch, yaw)
            -> dict of servo name -> commanded angle
    """

    def __init__(self, pca_left, pca_right, tail_config):
        """
        Initialize a Tail instance.

        Stores both PCA9685 drivers and the tail configuration.
        Subclasses are responsible for creating their own Servo
        objects from the config.

        Parameters
        ----------
        pca_left : adafruit_pca9685.PCA9685
            The PCA9685 driver object for the left side of the tail.
        pca_right : adafruit_pca9685.PCA9685
            The PCA9685 driver object for the right side of the tail.
        tail_config : dict
            Configuration dictionary for the tail (see config/tail_config.py).
            Must contain 'servos' and any PID gain entries needed
            by the subclass.
        """
        self.pca_left = pca_left
        self.pca_right = pca_right
        self.config = tail_config

    @abstractmethod
    def compute_control(self, roll, pitch, yaw):
        """
        Compute the commanded servo positions based on chassis orientation.

        Implements the PID (or other) control law that maps the
        current roll, pitch, and yaw of the chassis to desired
        servo angles for the tail mechanism.

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
        dict
            Dictionary of servo name (str) : commanded angle (float)
            in degrees.  Keys should match the servo names defined
            in the tail config.
        """
        pass

    @abstractmethod
    def move_tail(self, roll, pitch, yaw):
        """
        Compute control and command the tail servos.

        Calls compute_control to obtain the desired servo angles,
        then sends those commands to the physical servos.

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
        tuple
            (servo_angle_1, servo_angle_2, ...)
            Control outputs from compute_control and the servo
            angles actually sent to the servos, for logging.
        """
        pass
