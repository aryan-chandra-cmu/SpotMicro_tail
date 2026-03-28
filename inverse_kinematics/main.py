# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 19th March 2026

"""
Main entry point for the Spot Micro quadruped controller.

Initializes I2C buses, creates PCA9685 motor driver objects,
instantiates leg controllers, and runs the main control loop.
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685

from left_front_leg import LeftFrontLeg
from left_rear_leg import LeftRearLeg
from right_front_leg import RightFrontLeg
from right_rear_leg import RightRearLeg
# from gait_control import GaitControl
from ik_testing import IKTesting
from data_logger import DataLogger
from config.leg_config import LEFT_FRONT_LEG, LEFT_REAR_LEG, RIGHT_FRONT_LEG, RIGHT_REAR_LEG

def init_motor_drivers():
    """
    Initialize I2C bus and create PCA9685 motor driver objects.

    Creates one PCA9685 instance per driver board. Each board is
    identified by its I2C address. Currently only one driver is used
    (for the left front leg); additional drivers for other legs
    will be added here in the future.

    Returns
    -------
    dict
        Dictionary of PCA9685 objects keyed by a descriptive name.
        Example: {"left_front": <PCA9685 at 0x40>}
    """
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create PCA9685 driver for the left front leg
    pca_left_front = PCA9685(i2c, address=LEFT_FRONT_LEG["driver_address"])
    pca_left_front.frequency = 50  # 50 Hz for standard servos

    # Create PCA9685 driver for the left rear leg
    pca_left_rear = PCA9685(i2c, address=LEFT_REAR_LEG["driver_address"])
    pca_left_rear.frequency = 50  # 50 Hz for standard servos

    # Create PCA9685 driver for the right front leg
    pca_right_front = PCA9685(i2c, address=RIGHT_FRONT_LEG["driver_address"])
    pca_right_front.frequency = 50  # 50 Hz for standard servos

    # Create PCA9685 driver for the right rear leg
    pca_right_rear = PCA9685(i2c, address=RIGHT_REAR_LEG["driver_address"])
    pca_right_rear.frequency = 50  # 50 Hz for standard servos

    drivers = {
        "left_front":  pca_left_front,
        "left_rear":   pca_left_rear,
        "right_front": pca_right_front,
        "right_rear":  pca_right_rear,
    }

    return drivers

def main():
    """
    Main control loop.

    1. Initialize motor drivers (PCA9685 boards over I2C).
    2. Instantiate leg controllers, passing the relevant driver.
    3. Initialize legs to straight-leg configuration.
    4. Instantiate the IK testing controller.
    5. Run the control loop: query IK tester for foot targets,
       command each leg to reach its target.
    """
    # --- Hardware initialization ---
    print("Initializing motor drivers...")
    drivers = init_motor_drivers()

    # --- Leg instantiation ---
    print("Initializing legs...")
    left_front  = LeftFrontLeg(pca=drivers["left_front"])
    left_rear   = LeftRearLeg(pca=drivers["left_rear"])
    right_front = RightFrontLeg(pca=drivers["right_front"])
    right_rear  = RightRearLeg(pca=drivers["right_rear"])

    legs = {
        "left_front":  left_front,
        "left_rear":   left_rear,
        "right_front": right_front,
        "right_rear":  right_rear,
    }

    # --- Initialize servos to straight-leg position ---
    # WARNING: Hold the quadruped in the air before running!
    print("Initializing servos to straight-leg configuration...")
    for leg_name, leg in legs.items():
        leg.initialize()

    # Wait for user confirmation before proceeding
    while True:
        response = input("Legs initialized. Continue? (y/n): ").strip().lower()
        if response == "y":
            break
        elif response == "n":
            print("Aborting.")
            for name, pca in drivers.items():
                pca.deinit()
            return

    # --- IK testing controller ---
    ik_test = IKTesting()
    ik_test.get_initial_positions()

    # --- Gait controller (commented out for IK testing) ---
    # gait = GaitControl()
    # gait.generate_trajectory()

    # --- Control loop ---
    print("Starting control loop...")
    total_time = 10.0   # total run time in seconds
    loop_rate = 50      # Hz
    dt = 1.0 / loop_rate
    num_steps = int(total_time / dt)
    t = 0.0

    ik_test.generate_trajectory(total_time)

    # --- Data logger ---
    logger = DataLogger(headers=[
        "time (s)",
        "target_x (mm)", "target_y (mm)", "target_z (mm)",
        "joint_hip (deg)", "joint_shoulder (deg)", "joint_knee (deg)",
        "servo_hip (deg)", "servo_shoulder (deg)", "servo_knee (deg)",
    ])

    try:
        for step in range(num_steps):
            # Get latest foot targets from IK testing controller
            for leg_name, leg in legs.items():
                x, y, z = ik_test.get_foot_position(leg_name, t)
                joint_angles, motor_angles = leg.command_servos(x, y, z)

                theta_hip, theta_shoulder, theta_knee = joint_angles
                motor_hip, motor_shoulder, motor_knee = motor_angles

                logger.log(
                    t,
                    x, y, z,
                    theta_hip, theta_shoulder, theta_knee,
                    motor_hip, motor_shoulder, motor_knee,
                )

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nShutting down early...")
    finally:
        logger.save()
        # Deinitialize all PCA9685 boards
        for name, pca in drivers.items():
            pca.deinit()
        print("Motor drivers deinitialized")

if __name__ == "__main__":
    main()