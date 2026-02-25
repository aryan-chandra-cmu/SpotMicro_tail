# scripts/walk_spotmicro_mujoco_log_torques.py
# DROP-IN FILE
#
# Runs the same forward trot gait as walk_spotmicro_mujoco.py
# Logs ACTUATOR TORQUES to CSV for 20 seconds of SIM TIME,
# while throttling playback to ~REAL TIME (no fast-forward).
#
# Output CSV columns:
# time,
# 4x foot (knee) torques: FL FR RL RR,
# 4x leg (hip pitch) torques: FL FR RL RR,
# 4x shoulder (ab/ad) torques: FL FR RL RR
#
# Run:
#   (mujoco) python .\scripts\walk_spotmicro_mujoco_log_torques.py

from __future__ import annotations

import os
import math
import csv
import time
from typing import Dict, Tuple

import numpy as np
import mujoco
import mujoco.viewer


LEG_JOINTS: Dict[str, Tuple[str, str, str]] = {
    "FL": ("front_left_shoulder", "front_left_leg", "front_left_foot"),
    "FR": ("front_right_shoulder", "front_right_leg", "front_right_foot"),
    "RL": ("rear_left_shoulder", "rear_left_leg", "rear_left_foot"),
    "RR": ("rear_right_shoulder", "rear_right_leg", "rear_right_foot"),
}


def build_actuator_for_joint_map(model: mujoco.MjModel) -> Dict[str, int]:
    """Map joint_name -> actuator_id, assuming each actuator is directly tied to a joint."""
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
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(project_root, "assets", "robot.xml")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"robot.xml not found at: {xml_path}")

    out_dir = os.path.join(project_root, "logs")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "spotmicro_joint_torques_20s.csv")

    # Load model/data
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
        "front_left_foot":  -1.6,
        "front_right_foot": -1.6,
        "rear_left_foot":   -1.4,
        "rear_right_foot":  -1.4,
    }

    for jn in stand_targets:
        if jn not in act_for_joint:
            raise RuntimeError(
                f"No actuator found for joint '{jn}'. "
                f"Make sure <actuator><position joint=\"{jn}\" .../></actuator> exists."
            )

    # ---- Gait params ----
    freq_hz = 1.0
    hip_swing_amp = 0.15
    knee_flex_amp = 0.275
    knee_stance_amp = 0.075
    shoulder_amp = 0.03

    settle_seconds = 2.0
    ramp_seconds = 2.0

    # Diagonal trot
    phase_offset = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}

    # Reset + initial base pose
    mujoco.mj_resetData(model, data)
    if model.nq >= 7:
        data.qpos[0:7] = np.array([0.0, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0], dtype=float)
    mujoco.mj_forward(model, data)

    # Prepare CSV
    header = [
        "time",
        "FL_foot_torque", "FR_foot_torque", "RL_foot_torque", "RR_foot_torque",
        "FL_leg_torque",  "FR_leg_torque",  "RL_leg_torque",  "RR_leg_torque",
        "FL_shoulder_torque", "FR_shoulder_torque", "RL_shoulder_torque", "RR_shoulder_torque",
    ]

    leg_order = ["FL", "FR", "RL", "RR"]

    def get_act_tau(joint_name: str) -> float:
        """Actuator torque (N*m for hinge joints) for the actuator tied to this joint."""
        a = act_for_joint[joint_name]
        return float(data.actuator_force[a])

    sim_duration = 20.0  # seconds of SIM time
    use_viewer = True

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        if use_viewer:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                # Wall-clock reference for real-time throttling
                t_wall0 = time.time()

                while viewer.is_running() and data.time < sim_duration:
                    t = float(data.time)

                    # ---- Control ----
                    if t < settle_seconds:
                        for jn, q in stand_targets.items():
                            data.ctrl[act_for_joint[jn]] = q
                    else:
                        ramp = smoothstep((t - settle_seconds) / max(1e-6, ramp_seconds))
                        tg = (t - settle_seconds)

                        for leg, (jS, jH, jK) in LEG_JOINTS.items():
                            ph = gait_phase(tg, freq_hz, phase_offset[leg])
                            c = math.cos(ph)
                            s = math.sin(ph)
                            is_swing = c > 0.0

                            hip = stand_targets[jH] - ramp * hip_swing_amp * s

                            if is_swing:
                                swing_shape = c  # c>0 in swing
                                knee = stand_targets[jK] - ramp * knee_flex_amp * swing_shape
                            else:
                                stance_shape = -c  # positive in stance
                                knee = stand_targets[jK] + ramp * knee_stance_amp * stance_shape

                            shoulder = stand_targets[jS]
                            if leg in ("FL", "RL"):
                                shoulder += ramp * shoulder_amp * s
                            else:
                                shoulder -= ramp * shoulder_amp * s

                            data.ctrl[act_for_joint[jS]] = shoulder
                            data.ctrl[act_for_joint[jH]] = hip
                            data.ctrl[act_for_joint[jK]] = knee

                    # Step sim
                    mujoco.mj_step(model, data)

                    # --- REAL-TIME THROTTLE (prevents fast-forward) ---
                    sim_t = float(data.time)
                    wall_t = time.time() - t_wall0
                    ahead = sim_t - wall_t
                    if ahead > 0.0:
                        time.sleep(min(ahead, 0.01))  # cap to stay responsive

                    # Log torques
                    foot_taus, leg_taus, shoulder_taus = [], [], []
                    for leg in leg_order:
                        jS, jH, jK = LEG_JOINTS[leg]
                        foot_taus.append(get_act_tau(jK))      # knee
                        leg_taus.append(get_act_tau(jH))       # hip pitch
                        shoulder_taus.append(get_act_tau(jS))  # shoulder

                    writer.writerow([t] + foot_taus + leg_taus + shoulder_taus)
                    viewer.sync()
        else:
            # Headless: will run as fast as possible (not real-time)
            while data.time < sim_duration:
                t = float(data.time)

                if t < settle_seconds:
                    for jn, q in stand_targets.items():
                        data.ctrl[act_for_joint[jn]] = q
                else:
                    ramp = smoothstep((t - settle_seconds) / max(1e-6, ramp_seconds))
                    tg = (t - settle_seconds)
                    for leg, (jS, jH, jK) in LEG_JOINTS.items():
                        ph = gait_phase(tg, freq_hz, phase_offset[leg])
                        c = math.cos(ph)
                        s = math.sin(ph)
                        is_swing = c > 0.0

                        hip = stand_targets[jH] - ramp * hip_swing_amp * s
                        if is_swing:
                            knee = stand_targets[jK] - ramp * knee_flex_amp * c
                        else:
                            knee = stand_targets[jK] + ramp * knee_stance_amp * (-c)

                        shoulder = stand_targets[jS]
                        shoulder += (ramp * shoulder_amp * s) if leg in ("FL", "RL") else (-ramp * shoulder_amp * s)

                        data.ctrl[act_for_joint[jS]] = shoulder
                        data.ctrl[act_for_joint[jH]] = hip
                        data.ctrl[act_for_joint[jK]] = knee

                mujoco.mj_step(model, data)

                foot_taus, leg_taus, shoulder_taus = [], [], []
                for leg in leg_order:
                    jS, jH, jK = LEG_JOINTS[leg]
                    foot_taus.append(get_act_tau(jK))
                    leg_taus.append(get_act_tau(jH))
                    shoulder_taus.append(get_act_tau(jS))

                writer.writerow([t] + foot_taus + leg_taus + shoulder_taus)

    print(f"\nSaved torque log to: {csv_path}")
    print("Units: time [s], torques [N*m] for hinge joints.")


if __name__ == "__main__":
    main()