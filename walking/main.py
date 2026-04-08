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
from gait_control import Trot
from standing_control import StandingControl
from data_logger import DataLogger
from orientation_estimator import OrientationEstimator
from spot_micro_tail import SpotMicroTail
from spot_config import LEFT_FRONT_LEG, LEFT_REAR_LEG, RIGHT_FRONT_LEG, RIGHT_REAR_LEG, TAIL

def init_i2c():
    """
    Initialize I2C bus, PCA9685 motor drivers, and the BNO055 IMU.

    Creates one PCA9685 instance per driver board on the shared I2C
    bus.  Also creates the OrientationEstimator (BNO055 IMU) on the
    same bus.

    Returns
    -------
    tuple of (dict, OrientationEstimator)
        (drivers, imu) — Dictionary of PCA9685 objects keyed by leg
        name, and the IMU orientation estimator.
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

    # Create BNO055 IMU orientation estimator
    # imu = OrientationEstimator(i2c)

    return drivers, None

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
    # --- I2C initialization ---
    print("Initializing I2C devices...")
    drivers, imu = init_i2c()

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

    # --- Tail instantiation ---
    print("Initializing tail...")
    tail = SpotMicroTail(pca_left=drivers["left_rear"], pca_right=drivers["right_rear"])

    # --- Initialize servos to straight-leg position ---
    # WARNING: Hold the quadruped in the air before running!
    print("Initializing servos to straight-leg configuration and tail to neutral...")
    for leg_name, leg in legs.items():
        leg.initialize()
    tail.initialize()

    # Wait for user confirmation before proceeding
    while True:
        response = input("Legs initialized. Continue to standing? (y/n): ").strip().lower()
        if response == "y":
            break
        elif response == "n":
            print("Aborting.")
            for name, pca in drivers.items():
                pca.deinit()
            return

    # --- Standing Phase ---
    print("Executing standing sequence...")
    stand_ctrl = StandingControl()
    stand_ctrl.get_initial_positions()
    
    stand_duration = 2.0  # seconds
    stand_ctrl.generate_trajectory(stand_duration)
    
    loop_rate = 50      # Hz
    dt = 1.0 / loop_rate
    num_stand_steps = int(stand_duration / dt)
    t_stand = 0.0

    for step in range(num_stand_steps):
        for leg_name, leg in legs.items():
            x, y, z = stand_ctrl.get_foot_position(leg_name, t_stand)
            leg.command_servos(x, y, z)
        t_stand += dt
        time.sleep(dt)

    # Wait for user confirmation before proceeding to walking gait
    while True:
        response = input("Standing complete. Continue to walking gait? (y/n): ").strip().lower()
        if response == "y":
            break
        elif response == "n":
            print("Aborting.")
            for name, pca in drivers.items():
                pca.deinit()
            return

    # --- Gait Controller ---
    trot = Trot()
    current_forward_speed = 0.0

    # --- Control loop ---
    print("Starting gait control loop...")
    total_time = 10.0   # total run time in seconds
    num_steps = int(total_time / dt)
    t = 0.0

    # --- Data logger ---
    logger = DataLogger(headers=[
        "time (s)",
        "target_x (mm)", "target_y (mm)", "target_z (mm)",
        "joint_hip (deg)", "joint_shoulder (deg)", "joint_knee (deg)",
        "servo_hip (deg)", "servo_shoulder (deg)", "servo_knee (deg)",
    ])

    try:
        for step in range(num_steps):
            # Get latest foot targets from the gait controller
            for leg_name, leg in legs.items():
                x, y, z = trot.get_foot_position(leg_name, t, current_forward_speed)
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