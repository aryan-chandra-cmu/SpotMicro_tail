# scripts/walk_spotmicro_mujoco.py
# DROP-IN FILE
#
# Forward trot gait that actually lifts feet.
# Uses your stand_targets as the center pose.
#
# Key idea:
# - Diagonal trot: FL+RR swing together, FR+RL swing together
# - Swing phase: hip moves FORWARD, knee FLEXES (more negative) to lift
# - Stance phase: hip moves BACKWARD, knee EXTENDS (less negative) to support/push
#
# Run:
#   (mujoco) python .\scripts\walk_spotmicro_mujoco.py

from __future__ import annotations
import os
import time
import math
from typing import Dict

import numpy as np
import mujoco
import mujoco.viewer


LEG_JOINTS = {
    "FL": ("front_left_shoulder", "front_left_leg", "front_left_foot"),
    "FR": ("front_right_shoulder", "front_right_leg", "front_right_foot"),
    "RL": ("rear_left_shoulder", "rear_left_leg", "rear_left_foot"),
    "RR": ("rear_right_shoulder", "rear_right_leg", "rear_right_foot"),
}


def build_actuator_for_joint_map(model: mujoco.MjModel) -> Dict[str, int]:
    m: Dict[str, int] = {}
    for a in range(model.nu):
        j_id = int(model.actuator_trnid[a, 0])
        if j_id >= 0:
            j_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            if j_name:
                m[j_name] = a
    return m


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def gait_phase(t: float, freq_hz: float, phase_offset_rad: float) -> float:
    return (2.0 * math.pi * freq_hz * t + phase_offset_rad) % (2.0 * math.pi)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(project_root, "assets", "robot.xml")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"robot.xml not found at: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    act_for_joint = build_actuator_for_joint_map(model)

    # ---- YOUR stand pose (center pose) ----
    stand_targets = {
        "front_left_shoulder":  0.0,
        "front_right_shoulder": 0.0,
        "rear_left_shoulder":   0.0,
        "rear_right_shoulder":  0.0,
        "front_left_leg":   0.85,
        "front_right_leg":  0.85,
        "rear_left_leg":    0.85,
        "rear_right_leg":   0.85,
        "front_left_foot":  -1.4,
        "front_right_foot": -1.4,
        "rear_left_foot":   -1.4,
        "rear_right_foot":  -1.4,
    }

    for jn in stand_targets:
        if jn not in act_for_joint:
            raise RuntimeError(
                f"No actuator found for joint '{jn}'. "
                f"Make sure <actuator><position joint=\"{jn}\" .../></actuator> exists."
            )

    # ---- Gait params (these WILL lift) ----
    freq_hz = 1.0

    # Hip swings forward/back around stand
    hip_swing_amp = 0.15

    # Knee: we want MORE FLEX during swing => more negative (since stand is -1.4)
    knee_flex_amp = 0.275   # swing flex
    knee_stance_amp = 0.075 # small extension during stance (less negative)

    # Optional small shoulder for clearance
    shoulder_amp = 0.03

    settle_seconds = 2.0
    ramp_seconds = 2.0

    # Diagonal trot
    # FL+RR together, FR+RL together
    phase_offset = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}

    # Spawn
    mujoco.mj_resetData(model, data)
    if model.nq >= 7:
        data.qpos[0:7] = np.array([0.0, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0], dtype=float)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()

        while viewer.is_running():
            t = time.time() - t0

            if t < settle_seconds:
                # hold stand
                for jn, q in stand_targets.items():
                    data.ctrl[act_for_joint[jn]] = q

            else:
                ramp = smoothstep((t - settle_seconds) / max(1e-6, ramp_seconds))
                tg = (t - settle_seconds)

                for leg, (jS, jH, jK) in LEG_JOINTS.items():
                    ph = gait_phase(tg, freq_hz, phase_offset[leg])

                    # Define swing vs stance using cosine:
                    # swing when cos(ph) > 0 (front half of cycle), stance otherwise.
                    c = math.cos(ph)
                    s = math.sin(ph)

                    is_swing = c > 0.0

                    # Hip:
                    # - During swing, move forward (positive direction)
                    # - During stance, move backward
                    hip = stand_targets[jH] - ramp * hip_swing_amp * s

                    # Knee:
                    # - During swing: FLEX more => add negative amount
                    # - During stance: extend slightly => add positive amount
                    if is_swing:
                        # 0..1 swing shape peaking mid-swing
                        swing_shape = c  # since c>0 here
                        knee = stand_targets[jK] - ramp * knee_flex_amp * swing_shape
                    else:
                        stance_shape = -c  # positive in stance
                        knee = stand_targets[jK] + ramp * knee_stance_amp * stance_shape

                    # Shoulder: small alternating lateral for clearance (optional)
                    # Keep symmetric left/right
                    shoulder = stand_targets[jS]
                    if leg in ("FL", "RL"):
                        shoulder += ramp * shoulder_amp * s
                    else:
                        shoulder -= ramp * shoulder_amp * s

                    data.ctrl[act_for_joint[jS]] = shoulder
                    data.ctrl[act_for_joint[jH]] = hip
                    data.ctrl[act_for_joint[jK]] = knee

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()