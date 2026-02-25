# Lab 03 — EKF Localization

## Lab Details

- **Overview:** Design an Extended Kalman Filter (EKF) and use it for online and offline localization experiments. The EKF fuses wheel encoder odometry with overhead camera measurements to estimate the robot's pose $(x, y, \theta)$ in real time.
- **Filter State:** $\mathbf{x} = [x, y, \theta]^T$
- **Control Input:** $\mathbf{u} = [\text{encoder\_counts},\; \text{steering\_command}]^T$
- **Measurement:** $\mathbf{z} = [x_{\text{cam}}, y_{\text{cam}}, \theta_{\text{cam}}]^T$ (from ArUco marker detection)

---

## How to Run

### Prerequisites
Complete the [Quick Start](../README.md#quick-start) setup, then ensure the package is installed with dev dependencies:

```bash
# Activate virtual environment
source rob6213_env/bin/activate

# Install (if not already)
pip install -e .[dev]
```

### Launch the GUI

**Option A — Direct launch:**
```bash
gui
```

**Option B — Python module:**
```bash
python3 -m robot_python_code.robot_gui
```

**Option C — Systemd service (on the robot):**
```bash
sudo ./configs/install-service.sh
# GUI will be available at http://<robot-ip>:8080
```

Once running, navigate to the **EKF** tab in the GUI to access both offline and online experiments.

### Standalone EKF (no GUI)
You can also run the EKF offline directly from the command line:
```bash
python3 -m robot_python_code.extended_kalman_filter
```

---

## EKF Implementation

The Extended Kalman Filter is implemented in [`src/robot_python_code/extended_kalman_filter.py`](../src/robot_python_code/extended_kalman_filter.py).

### Key Components

| Component | Method | Description |
|-----------|--------|-------------|
| **Motion Model** | `g_function()` | Bicycle-model prediction using encoder + steering |
| **Prediction Step** | `prediction_step()` | Propagates state mean and covariance forward |
| **Correction Step** | `correction_step()` | Fuses camera measurement via Kalman gain |
| **Process Noise** | `get_R()` | Motion noise covariance from calibration parameters |
| **Measurement Noise** | `get_Q()` | Camera observation noise covariance |
| **Jacobians** | `get_G_x()`, `get_G_u()`, `get_H()` | Linearization of motion and observation models |

### Parameters (from `parameters.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `counts_to_m` | 3518 | Encoder counts per meter (from Lab 02 calibration) |
| `distance_variance_a` | 0.0001 | Base distance variance |
| `distance_variance_b` | 0.01 | Distance variance scaling factor |
| `steering_to_w` | 0.0024 | Steering command to angular velocity conversion |
| `steering_variance_a` | 0.0001 | Base steering variance |
| `steering_variance_b` | -0.000635 | Steering variance scaling factor |
| `wheelbase` | 0.1444 m | Distance between front and rear axles |
| `covariance_plot_scale` | 100 | Scale factor for covariance ellipse visualization |

---

## Experiments

### Experiment 1 — Offline EKF: Wheel Encoders Only (Prediction Only)

Run the EKF on pre-recorded data using **only the prediction step** (no camera correction). This demonstrates pure dead-reckoning drift over time.

#### How to run
1. Launch the GUI and navigate to the **EKF** tab
2. Under **Offline EKF — Replay Logged Data**, select a data file
3. Ensure **"Use Camera Correction"** is toggled **OFF**
4. Click **"Run Offline EKF"**

#### GUI Walkthrough

<!-- TODO: Record a GIF demonstrating the offline EKF prediction-only workflow -->
![Experiment 1 — Offline EKF Prediction Only](assets/lab03/exp1_offline_prediction_only.gif)

#### Results

| Metric | Value |
|--------|-------|
| Data File | <!-- TODO --> |
| Total Frames | <!-- TODO --> |
| Mean Position Error (m) | <!-- TODO --> |
| Max Position Error (m) | <!-- TODO --> |
| Final Drift (m) | <!-- TODO --> |

**Trajectory Plot:**
<!-- TODO: Add exported trajectory plot -->
![Exp 1 — Trajectory](assets/lab03/exp1_trajectory.png)

**Error Over Time:**
<!-- TODO: Add exported error plot -->
![Exp 1 — Error](assets/lab03/exp1_error.png)

#### Analysis
<!-- TODO: Discuss observed drift, covariance growth, and lack of correction.
     Expected behavior: the covariance ellipse should grow unbounded over time
     and the estimated trajectory should diverge from the camera ground truth. -->

---

### Experiment 2 — Offline EKF: Camera + Wheel Encoders (Full EKF)

Run the EKF on the same pre-recorded data with **both prediction and correction steps** enabled. The camera provides periodic pose measurements that bound the estimation error.

#### How to run
1. Launch the GUI and navigate to the **EKF** tab
2. Under **Offline EKF — Replay Logged Data**, select the **same data file** as Experiment 1
3. Toggle **"Use Camera Correction"** to **ON**
4. Click **"Run Offline EKF"**

#### GUI Walkthrough

<!-- TODO: Record a GIF demonstrating the offline EKF with camera correction -->
![Experiment 2 — Offline EKF Full](assets/lab03/exp2_offline_full_ekf.gif)

#### Results

| Metric | Value |
|--------|-------|
| Data File | <!-- TODO --> |
| Total Frames | <!-- TODO --> |
| Mean Position Error (m) | <!-- TODO --> |
| Max Position Error (m) | <!-- TODO --> |
| Final Drift (m) | <!-- TODO --> |

**Comparison with Experiment 1:**

| Metric | Exp 1 (Prediction Only) | Exp 2 (Full EKF) | Improvement |
|--------|-------------------------|-------------------|-------------|
| Mean Position Error (m) | <!-- TODO --> | <!-- TODO --> | <!-- TODO --> |
| Max Position Error (m) | <!-- TODO --> | <!-- TODO --> | <!-- TODO --> |
| Final Drift (m) | <!-- TODO --> | <!-- TODO --> | <!-- TODO --> |

**Trajectory Plot:**
<!-- TODO: Add exported trajectory plot -->
![Exp 2 — Trajectory](assets/lab03/exp2_trajectory.png)

**Error Over Time:**
<!-- TODO: Add exported error plot -->
![Exp 2 — Error](assets/lab03/exp2_error.png)

#### Analysis
<!-- TODO: Discuss how camera correction bounds the covariance ellipse,
     reduces drift, and keeps the estimate close to ground truth.
     Compare quantitatively against Experiment 1. -->

---

### Experiment 3 — Online EKF: Live Camera + Wheel Encoders

Run the EKF **live on the robot** using real-time wheel encoder data and camera observations. The robot must be physically connected and the camera stream active.

#### How to run
1. Launch the GUI (either locally or via the systemd service on the robot)
2. On the **Robot Control** tab, toggle **"Robot Connect"** to establish UDP communication
3. Navigate to the **EKF** tab
4. The **Online EKF — Live Localization** plot updates automatically at 10 Hz
5. Drive the robot using the speed and steering controls
6. Use **"Clear Trail"** to reset the plotted trajectory

#### GUI Walkthrough

<!-- TODO: Record a GIF demonstrating the online EKF live localization -->
![Experiment 3 — Online EKF Live](assets/lab03/exp3_online_live_ekf.gif)

#### Results

| Metric | Value |
|--------|-------|
| Duration (s) | <!-- TODO --> |
| Path Type | <!-- TODO: e.g., straight line, figure-8, square --> |
| Qualitative Tracking | <!-- TODO: Good / Fair / Poor --> |
| Covariance Behavior | <!-- TODO: Bounded / Growing --> |

**Live Localization Screenshot:**
<!-- TODO: Add screenshot of the live EKF plot during operation -->
![Exp 3 — Live Plot](assets/lab03/exp3_live_screenshot.png)

#### Analysis
<!-- TODO: Discuss real-time performance, latency, how well the filter
     tracks the robot, any observed issues with the camera measurement
     rate, and how the covariance behaves during motion vs. stationary periods. -->

---

## Summary

| Experiment | Sensors | Mode | Correction | Expected Drift |
|------------|---------|------|------------|----------------|
| 1 | Encoders only | Offline | None | Unbounded (grows over time) |
| 2 | Encoders + Camera | Offline | Camera | Bounded (corrected by measurements) |
| 3 | Encoders + Camera | Online | Camera | Bounded (real-time correction) |

---

## File References

| File | Purpose |
|------|---------|
| [`src/robot_python_code/extended_kalman_filter.py`](../src/robot_python_code/extended_kalman_filter.py) | EKF implementation |
| [`src/robot_python_code/robot_gui.py`](../src/robot_python_code/robot_gui.py) | GUI with EKF tab |
| [`src/robot_python_code/parameters.py`](../src/robot_python_code/parameters.py) | Calibration and filter parameters |
| [`src/robot_python_code/data_handling.py`](../src/robot_python_code/data_handling.py) | Data loading and trial file management |
| [`src/robot_python_code/motion_models.py`](../src/robot_python_code/motion_models.py) | Motion model definitions |
| [`configs/robot-gui.service`](../configs/robot-gui.service) | Systemd service for headless GUI deployment |
| [`configs/install-service.sh`](../configs/install-service.sh) | Service installation script |