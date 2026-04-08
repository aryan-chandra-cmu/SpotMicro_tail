import numpy as np
from spot_config import LEFT_FRONT_LEG, LEFT_REAR_LEG, RIGHT_FRONT_LEG, RIGHT_REAR_LEG, WALKING_PARAMS

class StandingControl:
    """
    Hardware Standing transitioning controller.

    Computes initial foot positions via forward kinematics (or config values), generates
    a 5th-order minimum jerk trajectory from those values up to a predefined "standing" 
    coordinate, and acts as a one-way path evaluated over time `t`.
    """

    def __init__(self):
        # Initial foot positions keyed by leg name (populated by get_initial_positions)
        self.initial_positions = {
            "left_front":  (0.0, 0.0, 0.0),
            "left_rear":   (0.0, 0.0, 0.0),
            "right_front": (0.0, 0.0, 0.0),
            "right_rear":  (0.0, 0.0, 0.0),
        }

        self.trajectories = {}
        self.stand_duration = 0.0

    def get_initial_positions(self):
        """
        Compute or lookup the initial foot positions.
        """
        init_lf = LEFT_FRONT_LEG["initialization"]
        self.initial_positions["left_front"] = (init_lf["x"], init_lf["y"], init_lf["z"])

        init_lr = LEFT_REAR_LEG["initialization"]
        self.initial_positions["left_rear"] = (init_lr["x"], init_lr["y"], init_lr["z"])

        init_rf = RIGHT_FRONT_LEG["initialization"]
        self.initial_positions["right_front"] = (init_rf["x"], init_rf["y"], init_rf["z"])

        init_rr = RIGHT_REAR_LEG["initialization"]
        self.initial_positions["right_rear"] = (init_rr["x"], init_rr["y"], init_rr["z"])

    def get_min_jerk_trajectory(self, initial_pos, initial_vel, initial_acc, final_pos, final_vel, final_acc, duration):
        """
        Solves for coefficients [a0, a1, a2, a3, a4, a5] of a 5th-order polynomial.
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

    def generate_trajectory(self, stand_duration, target_positions=None):
        """
        Generate strict standalone test trajectory for standing.
        Instead of going UP and DOWN, this trajectory plots one single transition
        from current positions to target_positions and holds there.
        """
        self.stand_duration = stand_duration

        if target_positions is None:
            # Deriving targets directly from our WALKING_PARAMS offset definitions
            target_positions = {
                "left_front":  (0.0, WALKING_PARAMS["left_front"]["y_offset"], WALKING_PARAMS["left_front"]["ground_z"]),
                "left_rear":   (0.0, WALKING_PARAMS["left_rear"]["y_offset"], WALKING_PARAMS["left_rear"]["ground_z"]),
                "right_front": (0.0, WALKING_PARAMS["right_front"]["y_offset"], WALKING_PARAMS["right_front"]["ground_z"]),
                "right_rear":  (0.0, WALKING_PARAMS["right_rear"]["y_offset"], WALKING_PARAMS["right_rear"]["ground_z"]),
            }

        for leg_name, target in target_positions.items():
            start = self.initial_positions[leg_name]

            # Trajectory (start -> target) over the entire stand_duration
            up_x = self.get_min_jerk_trajectory(start[0], 0.0, 0.0, target[0], 0.0, 0.0, self.stand_duration)
            up_y = self.get_min_jerk_trajectory(start[1], 0.0, 0.0, target[1], 0.0, 0.0, self.stand_duration)
            up_z = self.get_min_jerk_trajectory(start[2], 0.0, 0.0, target[2], 0.0, 0.0, self.stand_duration)

            self.trajectories[leg_name] = {
                "x_coeffs": up_x,
                "y_coeffs": up_y,
                "z_coeffs": up_z,
            }

    def get_foot_position(self, leg_name, t):
        """
        Return the target foot position for a given leg at time t.
        Clamps explicitly at `self.stand_duration` to freeze tracking at the new standing state.
        """
        traj = self.trajectories[leg_name]

        # Clamp t to [0, stand_duration]
        eval_t = max(0.0, min(t, self.stand_duration))

        coeffs_x, coeffs_y, coeffs_z = traj["x_coeffs"], traj["y_coeffs"], traj["z_coeffs"]

        # Evaluate polynomial: p(eval_t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        x = sum(a * eval_t**i for i, a in enumerate(coeffs_x))
        y = sum(a * eval_t**i for i, a in enumerate(coeffs_y))
        z = sum(a * eval_t**i for i, a in enumerate(coeffs_z))

        return (float(x), float(y), float(z))
