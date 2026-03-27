# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 25th March 2026

"""
Offline IK trajectory test and visualization.

Instantiates the LeftFrontLeg (without hardware) and IKTesting objects,
generates a test trajectory, and plots:
  Figure 1: Foot position trajectory (x, y, z)
  Figure 2: Joint angle trajectory (knee, hip, shoulder)
  Figure 3: Joint velocity trajectory (numerical differentiation)
  Figure 4: Servo command trajectory (motor angles)
"""

import numpy as np
import matplotlib.pyplot as plt

from left_front_leg import LeftFrontLeg
from ik_testing import IKTesting


def main():
    # --- Instantiate leg (pca=None for offline IK computation) ---
    leg = LeftFrontLeg(pca=None)

    # --- Instantiate IK testing controller ---
    ik_test = IKTesting()
    ik_test.get_initial_positions()

    # --- Generate trajectory ---
    total_time = 10.0   # seconds
    ik_test.generate_trajectory(total_time)

    # --- Simulate the control loop and collect data ---
    loop_rate = 50      # Hz
    dt = 1.0 / loop_rate
    num_steps = int(total_time / dt)

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

        # Get foot position from trajectory
        x, y, z = ik_test.get_foot_position("left_front", t)
        pos_x[step] = x
        pos_y[step] = y
        pos_z[step] = z

        # Compute IK joint angles (returns knee, hip, shoulder)
        knee_deg, hip_deg, shoulder_deg = leg.compute_ik(x, y, z)
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

    # --- Compute joint velocities via numerical differentiation ---
    knee_vel = np.gradient(knee_angles, dt)
    hip_vel = np.gradient(hip_angles, dt)
    shoulder_vel = np.gradient(shoulder_angles, dt)

    # ================================================================
    # Figure 1: Foot position trajectory
    # ================================================================
    fig1, axes1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle("Foot Position Trajectory", fontsize=14)

    axes1[0].plot(time_arr, pos_x, 'b-', linewidth=2)
    axes1[0].set_ylabel("x (mm)")
    axes1[0].set_title("Forward")
    axes1[0].grid(True)

    axes1[1].plot(time_arr, pos_y, 'g-', linewidth=2)
    axes1[1].set_ylabel("y (mm)")
    axes1[1].set_title("Lateral (inward +)")
    axes1[1].grid(True)

    axes1[2].plot(time_arr, pos_z, 'r-', linewidth=2)
    axes1[2].set_ylabel("z (mm)")
    axes1[2].set_title("Downward")
    axes1[2].set_xlabel("Time (s)")
    axes1[2].grid(True)

    fig1.tight_layout()

    # ================================================================
    # Figure 2: Joint angle trajectory
    # ================================================================
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle("Joint Angle Trajectory", fontsize=14)

    axes2[0].plot(time_arr, knee_angles, 'b-', linewidth=2)
    axes2[0].set_ylabel("Knee (deg)")
    axes2[0].grid(True)

    axes2[1].plot(time_arr, hip_angles, 'g-', linewidth=2)
    axes2[1].set_ylabel("Hip (deg)")
    axes2[1].grid(True)

    axes2[2].plot(time_arr, shoulder_angles, 'r-', linewidth=2)
    axes2[2].set_ylabel("Shoulder (deg)")
    axes2[2].set_xlabel("Time (s)")
    axes2[2].grid(True)

    fig2.tight_layout()

    # ================================================================
    # Figure 3: Joint velocity trajectory
    # ================================================================
    fig3, axes3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig3.suptitle("Joint Velocity Trajectory", fontsize=14)

    axes3[0].plot(time_arr, knee_vel, 'b-', linewidth=2)
    axes3[0].set_ylabel("Knee vel (deg/s)")
    axes3[0].grid(True)

    axes3[1].plot(time_arr, hip_vel, 'g-', linewidth=2)
    axes3[1].set_ylabel("Hip vel (deg/s)")
    axes3[1].grid(True)

    axes3[2].plot(time_arr, shoulder_vel, 'r-', linewidth=2)
    axes3[2].set_ylabel("Shoulder vel (deg/s)")
    axes3[2].set_xlabel("Time (s)")
    axes3[2].grid(True)

    fig3.tight_layout()

    # ================================================================
    # Figure 4: Servo command trajectory
    # ================================================================
    fig4, axes4 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig4.suptitle("Servo Command Trajectory", fontsize=14)

    axes4[0].plot(time_arr, servo_knee, 'b-', linewidth=2)
    axes4[0].set_ylabel("Servo Knee (deg)")
    axes4[0].grid(True)

    axes4[1].plot(time_arr, servo_hip, 'g-', linewidth=2)
    axes4[1].set_ylabel("Servo Hip (deg)")
    axes4[1].grid(True)

    axes4[2].plot(time_arr, servo_shoulder, 'r-', linewidth=2)
    axes4[2].set_ylabel("Servo Shoulder (deg)")
    axes4[2].set_xlabel("Time (s)")
    axes4[2].grid(True)

    fig4.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
