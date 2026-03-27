# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 19th March 2026

"""
Leg configuration for the Spot Micro quadruped.

Each leg config is a dictionary containing:
  - link_lengths: dict with l1 (hip offset), l2 (upper leg), l3 (lower leg) in mm
  - servos: dict keyed by joint name (hip, shoulder, knee), each containing:
      - channel:    PCA9685 channel number
      - min_pulse:  minimum pulse width in microseconds
      - max_pulse:  maximum pulse width in microseconds
      - min_angle:  angle (deg) corresponding to min_pulse
      - max_angle:  angle (deg) corresponding to max_pulse
  - driver_address: I2C address of the PCA9685 board for this leg (hex int)
"""

# Left front leg link lengths (mm)
LFL_L1 = 40.0    # hip offset — lateral distance from hip pivot to shoulder pivot
LFL_L2 = 120.0   # upper leg length
LFL_L3 = 120.0   # lower leg length

LEFT_FRONT_LEG = {
    "link_lengths": {
        "l1": LFL_L1,
        "l2": LFL_L2,
        "l3": LFL_L3,
    },
    "servos": {
        "hip": {
            "channel": 1,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "shoulder": {
            "channel": 2,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "knee": {
            "channel": 0,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
    },
    "driver_address": 0x40,

    "initialization": {
        "servo_knee_angle": 180,
        "servo_hip_angle": 90,
        "servo_shoulder_angle": 90,
        "x": 0.0,
        "y": -LFL_L1,
        "z": LFL_L2 + LFL_L3,
    },
    
    "safety_limits": {
        "knee": {"min": 110.0, "max": 240.0},       # IK joint angle limits (deg)
        "hip": {"min": 160.0, "max": 250.0},
        "shoulder": {"min": 70.0, "max": 110.0},
    },
}

# Future legs can be added here following the same structure:
# RIGHT_FRONT_LEG = { ... }
# RIGHT_BACK_LEG  = { ... }
# LEFT_BACK_LEG   = { ... }
