# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# Date: 25th March 2026

"""
Offline Standing trajectory test and visualization.

Instantiates all four leg objects (without hardware) and StandingControl,
generates a dynamic upward tracking test trajectory, and plots per leg:
  Figure: 4 subplots — foot position (x, y, z), joint angles,
          joint velocities, and servo commands.
"""

import numpy as np
import matplotlib.pyplot as plt

from left_front_leg import LeftFrontLeg
from left_rear_leg import LeftRearLeg
from right_front_leg import RightFrontLeg
from right_rear_leg import RightRearLeg
from standing_control import StandingControl


def simulate_leg(leg, stand_ctrl, leg_name, total_time, dt, num_steps):
    """
    Run the offline standing simulation for a single leg.
    """
    time_arr = np.zeros(num_steps)
    pos_x = np.zeros(num_steps)
    pos_y = np.zeros(num_steps)
    pos_z = np.zeros(num_steps)
    knee_angles = np.zeros(num_steps)
    hip_angles = np.zeros(num_steps)
    shoulder_angles = np.zeros(num_steps)
    servo_knee = np.zeros(num_steps)
    servo_hip = np.zeros(num_steps)
    servo_shoulder = np.zeros(num_steps)

    t = 0.0
    for step in range(num_steps):
        time_arr[step] = t

        # Get foot position from polynomial
        x, y, z = stand_ctrl.get_foot_position(leg_name, t)
        
        pos_x[step] = x
        pos_y[step] = y
        pos_z[step] = z

        # Compute IK joint angles (returns hip, shoulder, knee)
        hip_deg, shoulder_deg, knee_deg = leg.compute_ik(x, y, z)
        knee_angles[step] = knee_deg
        hip_angles[step] = hip_deg
        shoulder_angles[step] = shoulder_deg

        # Compute servo/motor angles
        motor_hip, motor_shoulder, motor_knee = leg.joint_angles_to_motor_angles(
            hip_deg, shoulder_deg, knee_deg
        )
        servo_knee[step] = motor_knee
        servo_hip[step] = motor_hip
        servo_shoulder[step] = motor_shoulder

        t += dt

    # Compute joint velocities via numerical differentiation
    knee_vel = np.gradient(knee_angles, dt)
    hip_vel = np.gradient(hip_angles, dt)
    shoulder_vel = np.gradient(shoulder_angles, dt)

    return {
        "time": time_arr,
        "pos_x": pos_x, "pos_y": pos_y, "pos_z": pos_z,
        "knee_angles": knee_angles, "hip_angles": hip_angles, "shoulder_angles": shoulder_angles,
        "knee_vel": knee_vel, "hip_vel": hip_vel, "shoulder_vel": shoulder_vel,
        "servo_knee": servo_knee, "servo_hip": servo_hip, "servo_shoulder": servo_shoulder,
    }


def plot_leg(data, leg_title):
    """
    Create a figure window with 4 subplot groups for a single leg.
    """
    time_arr = data["time"]

    fig, axes = plt.subplots(4, 3, figsize=(14, 12), sharex=True)
    fig.suptitle(leg_title, fontsize=16, fontweight="bold")

    # --- Row 0: Foot position ---
    axes[0, 0].plot(time_arr, data["pos_x"], 'b-', linewidth=2)
    axes[0, 0].set_ylabel("x (mm)")
    axes[0, 0].set_title("Forward")
    axes[0, 0].grid(True)

    axes[0, 1].plot(time_arr, data["pos_y"], 'g-', linewidth=2)
    axes[0, 1].set_ylabel("y (mm)")
    axes[0, 1].set_title("Lateral")
    axes[0, 1].grid(True)

    axes[0, 2].plot(time_arr, data["pos_z"], 'r-', linewidth=2)
    axes[0, 2].set_ylabel("z (mm)")
    axes[0, 2].set_title("Downward")
    axes[0, 2].grid(True)

    # --- Row 1: Joint angles ---
    axes[1, 0].plot(time_arr, data["knee_angles"], 'b-', linewidth=2)
    axes[1, 0].set_ylabel("Knee (deg)")
    axes[1, 0].set_title("Joint Angles")
    axes[1, 0].grid(True)

    axes[1, 1].plot(time_arr, data["hip_angles"], 'g-', linewidth=2)
    axes[1, 1].set_ylabel("Hip (deg)")
    axes[1, 1].grid(True)

    axes[1, 2].plot(time_arr, data["shoulder_angles"], 'r-', linewidth=2)
    axes[1, 2].set_ylabel("Shoulder (deg)")
    axes[1, 2].grid(True)

    # --- Row 2: Joint velocities ---
    axes[2, 0].plot(time_arr, data["knee_vel"], 'b-', linewidth=2)
    axes[2, 0].set_ylabel("Knee vel (deg/s)")
    axes[2, 0].set_title("Joint Velocities")
    axes[2, 0].grid(True)

    axes[2, 1].plot(time_arr, data["hip_vel"], 'g-', linewidth=2)
    axes[2, 1].set_ylabel("Hip vel (deg/s)")
    axes[2, 1].grid(True)

    axes[2, 2].plot(time_arr, data["shoulder_vel"], 'r-', linewidth=2)
    axes[2, 2].set_ylabel("Shoulder vel (deg/s)")
    axes[2, 2].grid(True)

    # --- Row 3: Servo commands ---
    axes[3, 0].plot(time_arr, data["servo_knee"], 'b-', linewidth=2)
    axes[3, 0].set_ylabel("Servo Knee (deg)")
    axes[3, 0].set_title("Servo Commands")
    axes[3, 0].set_xlabel("Time (s)")
    axes[3, 0].grid(True)

    axes[3, 1].plot(time_arr, data["servo_hip"], 'g-', linewidth=2)
    axes[3, 1].set_ylabel("Servo Hip (deg)")
    axes[3, 1].set_xlabel("Time (s)")
    axes[3, 1].grid(True)

    axes[3, 2].plot(time_arr, data["servo_shoulder"], 'r-', linewidth=2)
    axes[3, 2].set_ylabel("Servo Shoulder (deg)")
    axes[3, 2].set_xlabel("Time (s)")
    axes[3, 2].grid(True)

    fig.tight_layout()


def main():
    # --- Instantiate all four legs (pca=None for offline IK computation) ---
    legs = {
        "left_front":  LeftFrontLeg(pca=None),
        "left_rear":   LeftRearLeg(pca=None),
        "right_front": RightFrontLeg(pca=None),
        "right_rear":  RightRearLeg(pca=None),
    }

    leg_titles = {
        "left_front":  "Left Front Leg",
        "left_rear":   "Left Rear Leg",
        "right_front": "Right Front Leg",
        "right_rear":  "Right Rear Leg",
    }

    # --- Instantiate Standing controller ---
    stand_ctrl = StandingControl()
    stand_ctrl.get_initial_positions()
    
    stand_duration = 2.0  # Transition duration
    stand_ctrl.generate_trajectory(stand_duration)

    # --- Simulation parameters ---
    total_time = 4.0    # Evaluate extending past transition phase (stand holds)
    loop_rate = 50      # Hz
    dt = 1.0 / loop_rate
    num_steps = int(total_time / dt)

    # --- Simulate and plot each leg ---
    for leg_name, leg in legs.items():
        data = simulate_leg(leg, stand_ctrl, leg_name, total_time, dt, num_steps)
        plot_leg(data, f"{leg_titles[leg_name]} (Standing Transition)")

    plt.show()

if __name__ == "__main__":
    main()
