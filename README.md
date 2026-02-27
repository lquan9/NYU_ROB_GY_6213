# ROB-GY 6213: Robot Navigation and Localization
This repository was originally forked from: https://github.com/cmclarkk/NYU_ROB_GY_6213. </br>
Primarily used to solve labs for the NYU course ROB-GY 6213: Robot Localization and Navigation.

[![Arduino Build](https://github.com/lquan9/NYU_ROB_GY_6213/actions/workflows/arduino_ci.yml/badge.svg)](https://github.com/lquan9/NYU_ROB_GY_6213/actions)
![Pylint](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/lquan9/d09cfb9f249c9995b9fa1943d90a1d71/raw/pylint.json)
[![Python Version](https://img.shields.io/badge/python-%203.12-blue)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

| Authors              | Email                |
|-------------------|----------------------|
| **Jotheesh Kummathi**    | jrk8067@nyu.edu       |
| **Long Quang**    | lq2146@nyu.edu       |

> <sup>
> Authors are with New York University - Tandon School of Engineering
> <br> 6 MetroTech Center, Brooklyn, NY. 
> </sup>
> 
> [Code](https://github.com/lquan9/NYU_ROB_GY_6213) | [Docs](./docs)

<!--![BTF Robot](docs/assets/a_robot_image.jpg)-->

# Quick Start
## Pre-requisites
1. [Git LFS](https://git-lfs.github.com/)
2. [Python Virtual Env](https://docs.python.org/3/tutorial/venv.html)

<details>
<summary><strong>Click for instructions on installing prerequisites</strong></summary>

---
**1. Git-LFS**

We rely on GIT Large File Storage for handling binary files that are required for building and using Phoenix. E.g., pre-compiled system dependencies that aren't available from public servers and trained neural network weights.

```bash
#To install Git-LFS
sudo apt install git-lfs
git lfs install
```

**2. Python3 Virtual Environment**
```bash
sudo apt update
sudo apt install python3.12-venv
```
---
</details>

### Setup Python workspace
```bash
python3 -m venv rob6213_env
. rob6213_env/bin/activate
```
### Install dependencies
```bash
pip install -e .[dev]
```

### Run
```bash
python3 -m robot_python_code.robot_gui
```

#### Headless
```bash
gui
```

## Labs (WIP)

### Lab 01 — Robot Bring-Up & Remote Control
**Goal:** Build the robot and connect it to your computer to enable remote control.

**Key subgoals:**
- Robot chassis construction
- WiFi router setup
- Fork the base code
- Arduino software upload & testing
- GUI control

[Lab 01 details](docs/lab-01.md)

---

### Lab 02 — Motion Model & Calibration
**Goal:** Characterize robot motion with mathematical equations and probabilistic uncertainty.

**Key subgoals:**
- Setup: choose floor & tune steering
- Analysis: map encoder counts to distance
- Calibration: map steering commands to rotational velocity
- Implementation: motion model + sampling
- Documentation: report writing & video recording

[Lab 02 details](docs/lab-02.md)

---

### Lab 03 — EKF Localization
**Goal:** Design an Extended Kalman Filter (EKF) and use it for online/offline localization experiments.

**Key subgoals:**
- EKF design
- Offline localization experiments
- Online localization experiments

[Lab 03 details](docs/lab-03.md)

---

### Lab 04 — Particle Filter Localization
**Goal:** Design a Particle Filter (PF) and use it for online/offline localization experiments.

**Key subgoals:**
- PF design
- Offline localization experiments
- Online localization experiments

[Lab 04 details](docs/lab-04.md)

---

### Lab 05 — Autonomous Navigation System
**Goal:** Build an autonomous navigation system that drives from any A to B without human intervention.

**Key subgoals:**
- Localization algorithm design or selection
- RRT motion planning implementation
- Trajectory tracking implementation
- Offline navigation experiments
- Online navigation experiments

**Component breakdown:**
- Localization: estimate robot pose in the map
- RRT planning: collision-free path from start to goal
- Trajectory tracking: follow path with controller (e.g., Pure Pursuit or PID)

[Lab 05 details](#)

---

## References
