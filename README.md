# SpotMicro Tail – Bio-Inspired Tail Stabilization for Quadruped Locomotion

![Image](https://drive.google.com/file/d/1Vcfd1dIPPc8JiihfI15v5Qrfnt6stJdU/view?usp=sharing)


Repository: [https://github.com/aryan-chandra-cmu/SpotMicro_tail](https://github.com/aryan-chandra-cmu/SpotMicro_tail)

---

## Overview

**SpotMicro_tail** explores the integration of a bio-inspired tail into a SpotMicro-style quadruped robot to improve:

* Yaw stability
* Turning agility
* Disturbance rejection
* Dynamic locomotion performance

This project builds on the open-source SpotMicro platform and extends it with a controllable tail modeled and simulated in **MuJoCo**.

The long-term vision is to investigate how active tail control improves gait efficiency and robustness in legged robots.


---

## Motivation

Animals such as cheetahs, kangaroos, and lizards use their tails to:

* Counteract angular momentum
* Stabilize body orientation during running
* Improve maneuverability
* Recover from disturbances

This repository investigates whether similar benefits can be realized in a small quadruped platform.

---

## Current Features

* ✅ SpotMicro MuJoCo simulation
* ✅ Rigid tail model integrated into robot body
* ✅ Open-loop gait experiments
* ✅ Torque logging and analysis
* ✅ Preliminary stability experiments

---

## Repository Structure

```
SpotMicro_tail/
│
├── assets/              # URDF/XML files, meshes
├── scripts/             # Simulation and control scripts
├── logs/                # Torque logs and experiment data
├── rigid_tail/          # Tail-integrated robot models
└── README.md
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/aryan-chandra-cmu/SpotMicro_tail.git
cd SpotMicro_tail
```

### 2. Create environment

```bash
conda create -n mujoco python=3.10
conda activate mujoco
pip install mujoco numpy matplotlib
```

---

## Running Simulation

Example:

```bash
python scripts/walk_spotmicro_mujoco.py
```

To log actuator torques:

```bash
python scripts/walk_spotmicro_mujoco_log_torques.py
```

---

## Experimental Goals

The current experiments aim to answer:

* Does adding a tail reduce body yaw oscillations?
* Does it reduce actuator torques in turning gaits?
* Can the tail improve recovery from lateral disturbances?
* Does it improve gait symmetry?

---

## Upcoming Work

### 1. Tendon-Driven Tail

We will soon integrate a **tendon-driven tail mechanism** to better mimic biological tails.

Planned improvements:

* Multi-segment flexible tail
* Tendon routing with passive elasticity
* Energy-efficient actuation
* Improved angular momentum redistribution

This will allow studying compliant vs rigid tail behavior.

---

### 2. Reinforcement Learning Experiments

Future work includes:

* Training RL policies (PPO) for tail-leg coordination
* Comparing:

  * No tail
  * Passive tail
  * Actively controlled tail
* Evaluating:

  * Energy efficiency
  * Tracking performance
  * Disturbance rejection
  * Turning performance

We plan to experiment with:

* MuJoCo PPO training
* Command-tracking curriculum
* Stability-aware reward shaping

---

## Research Directions

Potential research questions:

* Can tail control reduce cost of transport?
* How does tail inertia influence gait frequency?
* Can tail oscillation synchronize with trot phase?
* Does tail help with sim-to-real robustness?

---

## Long-Term Vision

* Hardware implementation of tendon-driven tail
* Closed-loop control using IMU feedback
* Integration with MPC or RL-based whole-body control
* Publication-quality experimental benchmarking

---

## Contributors

* Aryan Chandra
* Collaborators working on bio-inspired locomotion & control

---

## License

MIT License

---

## Contact

For questions or collaboration:

Open an issue on GitHub or reach out via LinkedIn.

---

**Bio-inspired control meets quadruped robotics.
Exploring stability through tails.**
