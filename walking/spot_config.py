# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 28th March 2026

"""
Configuration for the Spot Micro quadruped.

Contains dictionaries for:
  - Leg configs (LEFT_FRONT_LEG, LEFT_REAR_LEG, RIGHT_FRONT_LEG, RIGHT_REAR_LEG)
  - Tail config (TAIL)

Each leg config dictionary contains:
  - link_lengths: dict with l1 (hip offset), l2 (upper leg), l3 (lower leg) in mm
  - servos: dict keyed by joint name (hip, shoulder, knee), each containing:
      - channel:    PCA9685 channel number
      - min_pulse:  minimum pulse width in microseconds
      - max_pulse:  maximum pulse width in microseconds
      - min_angle:  angle (deg) corresponding to min_pulse
      - max_angle:  angle (deg) corresponding to max_pulse
  - driver_address: I2C address of the PCA9685 board for this leg (hex int)

The tail config dictionary contains:
  - servos: dict keyed by side (left, right), each containing servo params
  - pid_gains: dict with proportional, integral, derivative gains
  - safety_limits: servo angle limits for each side
  - driver_addresses: I2C addresses for left and right PCA9685 boards
"""

# ============================================================================
# LEG CONFIGURATIONS
# ============================================================================

# Left front leg link lengths (mm)
LFL_L1 = 50.0    # hip offset — lateral distance from hip pivot to shoulder pivot
LFL_L2 = 110.0   # upper leg length
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
            "channel": 0,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "knee": {
            "channel": 2,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
    },
    "driver_address": 0x41,

    "initialization": {
        "servo_knee_angle": 180,    # modify to ensure leg is straight
        "servo_hip_angle": 90,
        "servo_shoulder_angle": 90,
        "servo_offset_hip": 0.0,        # offset to correct servo zeroing (deg)
        "servo_offset_shoulder": 0.0,  # offset to correct servo zeroing (deg)
        "servo_offset_knee": 15.0,        # offset to correct servo zeroing (deg)
        "x": 0.0,
        "y": -LFL_L1,
        "z": LFL_L2 + LFL_L3 - 1,
    },
    
    "safety_limits": {
        "knee": {"min": 90.0, "max": 180.0},       # IK joint angle limits (deg)
        "hip": {"min": -60.0, "max": 60.0},
        "shoulder": {"min": 70.0, "max": 110.0},
    },
}

# Left rear leg link lengths (mm)
LRL_L1 = LFL_L1    # hip offset — lateral distance from hip pivot to shoulder pivot
LRL_L2 = LFL_L2   # upper leg length
LRL_L3 = LFL_L3   # lower leg length

LEFT_REAR_LEG = {
    "link_lengths": {
        "l1": LRL_L1,
        "l2": LRL_L2,
        "l3": LRL_L3,
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
            "channel": 0,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "knee": {
            "channel": 2,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
    },
    "driver_address": 0x42,

    "initialization": {
        "servo_knee_angle": 180,
        "servo_hip_angle": 90,
        "servo_shoulder_angle": 90,
        "servo_offset_hip": 0.0,        # offset to correct servo zeroing (deg)
        "servo_offset_shoulder": 0.0,  # offset to correct servo zeroing (deg)
        "servo_offset_knee": 0.0,        # offset to correct servo zeroing (deg)
        "x": 0.0,
        "y": -LRL_L1,
        "z": LRL_L2 + LRL_L3 - 1,
    },
    
    "safety_limits": {
        "knee": {"min": 90.0, "max": 180.0},       # IK joint angle limits (deg)
        "hip": {"min": -60.0, "max": 60.0},
        "shoulder": {"min": 70.0, "max": 110.0},
    },
}

# Right front leg link lengths (mm)
RFL_L1 = LFL_L1    # hip offset — lateral distance from hip pivot to shoulder pivot
RFL_L2 = LFL_L2   # upper leg length
RFL_L3 = LFL_L3   # lower leg length

RIGHT_FRONT_LEG = {
    "link_lengths": {
        "l1": RFL_L1,
        "l2": RFL_L2,
        "l3": RFL_L3,
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
            "channel": 0,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "knee": {
            "channel": 2,
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
        "servo_offset_hip": 0.0,        # offset to correct servo zeroing (deg)
        "servo_offset_shoulder": 0.0,    # offset to correct servo zeroing (deg)
        "servo_offset_knee": 0.0,        # offset to correct servo zeroing (deg)
        "x": 0.0,
        "y": RFL_L1,
        "z": RFL_L2 + RFL_L3 - 1,
    },
    
    "safety_limits": {
        "knee": {"min": 90.0, "max": 180.0},       # IK joint angle limits (deg)
        "hip": {"min": -60.0, "max": 60.0},
        "shoulder": {"min": 70.0, "max": 110.0},
    },
}

# Right rear leg link lengths (mm)
RRL_L1 = LFL_L1    # hip offset — lateral distance from hip pivot to shoulder pivot
RRL_L2 = LFL_L2   # upper leg length
RRL_L3 = LFL_L3   # lower leg length

RIGHT_REAR_LEG = {
    "link_lengths": {
        "l1": RRL_L1,
        "l2": RRL_L2,
        "l3": RRL_L3,
    },
    "servos": {
        "hip": {
            "channel": 14,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "shoulder": {
            "channel": 15,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "knee": {
            "channel": 13,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
    },
    "driver_address": 0x44,

    "initialization": {
        "servo_knee_angle": 180,
        "servo_hip_angle": 90,
        "servo_shoulder_angle": 90,
        "servo_offset_hip": 0.0,        # offset to correct servo zeroing (deg)
        "servo_offset_shoulder": 0.0,    # offset to correct servo zeroing (deg)
        "servo_offset_knee": 0.0,        # offset to correct servo zeroing (deg)
        "x": 0.0,
        "y": RRL_L1,
        "z": RRL_L2 + RRL_L3 - 1,
    },
    
    "safety_limits": {
        "knee": {"min": 90.0, "max": 180.0},       # IK joint angle limits (deg)
        "hip": {"min": -60.0, "max": 60.0},
        "shoulder": {"min": 70.0, "max": 110.0},
    },
}

# ============================================================================
# TAIL CONFIGURATION
# ============================================================================

TAIL = {
    "servos": {
        "left": {
            "channel": 3,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
        "right": {
            "channel": 3,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_angle": 0.0,
            "max_angle": 270.0,
        },
    },

    "driver_addresses": {
        "left":  0x41,
        "right": 0x43,
    },

    "pid_gains": {
        "kp": 0.0,
        "ki": 0.0,
        "kd": 0.0,
    },

    "initialization": {
        "servo_left_angle": 135.0,      # neutral position (deg)
        "servo_right_angle": 135.0,     # neutral position (deg)
    },

    "safety_limits": {
        "left":  {"min": 45.0, "max": 225.0},   # servo angle limits (deg)
        "right": {"min": 45.0, "max": 225.0},
    },
}

# ============================================================================
# WALKING CONFIGURATION
# ============================================================================

WALKING_PARAMS = {
    "step_duration": 1.0,
    "left_front": {
        "step_height": 20.0,  
        "y_offset": -RFL_L1,
        "ground_z": 200.0,
    },
    "right_front": {
        "step_height": 20.0,
        "y_offset": RFL_L1,
        "ground_z": 200.0,
    },
    "left_rear": {
        "step_height": 20.0,
        "y_offset": -LRL_L1,
        "ground_z": 200.0,
    },
    "right_rear": {
        "step_height": 20.0,
        "y_offset": RRL_L1,
        "ground_z": 200.0,
    },
}
