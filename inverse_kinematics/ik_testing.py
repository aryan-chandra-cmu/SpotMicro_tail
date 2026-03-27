# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 25th March 2026

"""
IK testing for the Spot Micro quadruped.

Generates simple, known foot trajectories (e.g. straight-line motions)
for hardware validation of inverse kinematics calculations.
"""

import numpy as np

from config.leg_config import LEFT_FRONT_LEG


class IKTesting:
    """
    Hardware IK tester.

    Computes initial foot positions via forward kinematics, generates
    simple test trajectories (straight lines with zero start/end
    velocity), and provides the target foot position for each leg
    at any point in time.
    """

    def __init__(self):
        """
        Initialize the IKTesting controller.

        Sets up internal state for trajectory generation and stores
        initial foot positions (to be populated by get_initial_positions).
        """
        # Initial foot positions keyed by leg name (populated by get_initial_positions)
        self.initial_positions = {
            "left_front": (0.0, 0.0, 0.0),
            # "right_front": (0.0, 0.0, 0.0),
            # "right_back": (0.0, 0.0, 0.0),
            # "left_back": (0.0, 0.0, 0.0),
        }

        # Trajectory data keyed by leg name — each entry will hold
        # precomputed arrays of (x, y, z) positions over time
        self.trajectories = {}
        self.total_time = 0.0

    def get_initial_positions(self):
        """
        Compute the initial foot positions using forward kinematics.

        Reads the current servo/joint angles from each leg and applies
        forward kinematics to determine where each foot currently is
        in its local coordinate frame.  Results are stored in
        self.initial_positions.
        """
        # TODO: Implement forward kinematics to obtain initial foot positions
        init = LEFT_FRONT_LEG["initialization"]
        self.initial_positions["left_front"] = (init["x"], init["y"], init["z"])

    def get_min_jerk_trajectory(self, initial_pos, initial_vel, initial_acc, final_pos, final_vel, final_acc, duration):
        """5th order polynomial function, generated using t = 0 as initial time, as it makes calculations easier.

        Solves for coefficients [a0, a1, a2, a3, a4, a5] of:
            p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

        Parameters
        ----------
        initial_pos : float
            Position at t = 0.
        initial_vel : float
            Velocity at t = 0.
        initial_acc : float
            Acceleration at t = 0.
        final_pos : float
            Position at t = duration.
        final_vel : float
            Velocity at t = duration.
        final_acc : float
            Acceleration at t = duration.
        duration : float
            Total time of the trajectory.

        Returns
        -------
        list of float
            Coefficients [a0, a1, a2, a3, a4, a5].
        """
        time_end = duration

        a0 = initial_pos
        a1 = initial_vel
        a2 = initial_acc / 2

        # Get A and b for linear equation to get polynomial coefficients
        b = np.array([final_pos - a0 - a1*time_end - a2*time_end**2,
                      final_vel - a1 - 2*a2*time_end,
                      final_acc - 2*a2])

        A = np.array([[time_end**3,   time_end**4,    time_end**5],
                      [3*time_end**2, 4*time_end**3,  5*time_end**4],
                      [6*time_end,    12*time_end**2, 20*time_end**3]])

        x = np.linalg.solve(A, b)

        a3 = x[0]
        a4 = x[1]
        a5 = x[2]

        return [a0, a1, a2, a3, a4, a5]

    def generate_trajectory(self, total_time, target_positions=None):
        """
        Generate a simple straight-line test trajectory for each leg.

        The trajectory moves each foot from its initial position
        (stored in self.initial_positions to a target position over
        the specified duration using a minimum-jerk (5th-order
        polynomial) profile so that initial and final velocities
        and accelerations are all zero.

        Computes polynomial coefficients per axis (x, y, z) for each
        leg via get_min_jerk_trajectory and stores them for later
        evaluation in get_foot_position.

        Parameters
        ----------
        total_time : float
            Duration of the trajectory in seconds.
        target_positions : dict, optional
            Target (x, y, z) for each leg, keyed by leg name.
            If None, defaults are used.
        """
        self.total_time = total_time

        if target_positions is None:
            # Default target positions for testing (small motions)
            target_positions = {
                "left_front": (0.0, -40.0, 180.0),
                # "right_front": (30.0, -60.0, 200.0),
                # "right_back": (30.0, -60.0, 200.0),
                # "left_back": (30.0, -60.0, 200.0),
            }

        for leg_name, target in target_positions.items():
            start = self.initial_positions[leg_name]

            # Compute minimum-jerk coefficients for each axis (zero vel/acc at both ends)
            coeffs_x = self.get_min_jerk_trajectory(start[0], 0.0, 0.0, target[0], 0.0, 0.0, total_time)
            coeffs_y = self.get_min_jerk_trajectory(start[1], 0.0, 0.0, target[1], 0.0, 0.0, total_time)
            coeffs_z = self.get_min_jerk_trajectory(start[2], 0.0, 0.0, target[2], 0.0, 0.0, total_time)

            self.trajectories[leg_name] = {
                "coeffs_x": coeffs_x,
                "coeffs_y": coeffs_y,
                "coeffs_z": coeffs_z,
            }

    def get_foot_position(self, leg_name, t):
        """
        Return the target foot position for a given leg at time t.

        Evaluates the 5th-order polynomial from stored coefficients
        for each axis.

        Parameters
        ----------
        leg_name : str
            Name of the leg, e.g. 'left_front'.
        t : float
            Current time in seconds (0 <= t <= total_time).

        Returns
        -------
        tuple of (float, float, float)
            (x, y, z) target foot position in the leg's local frame (mm).
        """
        traj = self.trajectories[leg_name]

        # Clamp t to [0, total_time]
        t = max(0.0, min(t, self.total_time))

        # Evaluate polynomial: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        x = sum(a * t**i for i, a in enumerate(traj["coeffs_x"]))
        y = sum(a * t**i for i, a in enumerate(traj["coeffs_y"]))
        z = sum(a * t**i for i, a in enumerate(traj["coeffs_z"]))

        return (float(x), float(y), float(z))
