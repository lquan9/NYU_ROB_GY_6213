# Lab 04: Particle Filter Localization

In this lab, you will implement a **Particle Filter** to localize your robot within a global coordinate frame fixed within a known map. You will leverage a **2D scanning lidar** as your main source of exteroceptive sensing.

---

## Lab 04 Goals

The goals of Lab 04 are to design a Particle Filter and use it for online and offline robot localization experiments. The subgoals are:

- Lidar setup
- Map Creation
- PF Design
- Offline localization experiments
- Online localization experiments

### Key Deliverables

1. **Lab 04 Video**: Document at least 2 different online localization experiments (show robot and GUI).
2. **Lab 04 Report**: Document all four subgoals, including methods, results, and analysis.
3. **Lab 04 Code**: Submit your `particle_filter.py` file.

---

## 0. New Code

The new base code is available on the GitHub repo. Key changes include:

- **`robot_gui.py`**: Now displays a live lidar stream in the GUI.
- **`particle_filter.py`**: Main file for your implementation, including offline PF processing functions.
- **`robot_arduino_code.ino`**: Updated to collect and transmit lidar data.
- **`data_handling.py`**: Additional plotting and data loading functions.

---

## 1. LIDAR Setup

The lidar provides relative range and bearing measurements with respect to mapped walls.

**Steps:**

1. Remove the top deck and mount the lidar using M2.5 or M3 screws from your chassis kit.
2. Wire the lidar to the Arduino using the diagram and breakout cables provided below.

### Wiring Reference

| Pin Number | Wire Color | Description |
| --- | --- | --- |
| D12 | Purple | Osoyoo Power Board - IN 1 |
| D11 | Green | Osoyoo Power Board - IN 2 |
| D10 | Orange | Servo - PWM |
| D9  | Black  | Osoyoo Power Board - EN A |
| D8  | White  | Osoyoo Power Board - IN 4 |
| D7  | Yellow | Osoyoo Power Board - IN 3 |
| D6  | Red    | Osoyoo Power Board - EN B |
| D5  | Orange | Motor Encoder - S1 |
| D4  | Green  | Motor Encoder - S2 |
| D3  | Blue   | Lidar - CTRL MTR |
| C18 | Green  | Lidar - RX |
| C19 | Yellow | Lidar - TX |

#### Power & Ground Connections

| Connection Point | Wire Color | Description |
| --- | --- | --- |
| Female Red 5V      | Purple     | Lidar - 5V |
| Female Black GND   | Gray/Black | Lidar - GND |
| Female Red 5V      | Red        | Servo - 5V |
| Female Black GND   | Brown      | Servo - GND |
| Male Red 5V        | White      | Encoder - 5V (Hall) |
| Male Black GND     | Yellow     | Encoder - GND (Hall) |

3. Power on the robot; ensure the lidar spins after connecting to Wi-Fi.
4. Run `robot_gui.py` and verify the lidar scan is visible.
5. **Characterize the sensor**: Record data at known distances to establish bias and calculate **variance** (critical for PF weighting).

---

## 2. Map Creation

1. Choose your navigation environment.
2. Define the map in `parameters.py`. Walls are defined by end-point coordinates: $[x_1, y_1, x_2, y_2]$.

---

## 3. Particle Filter Design

Implement your PF in `particle_filter.py` to estimate the state $x_t = [x, y, \theta]^T$.

### Step 1: PF Preliminaries

- **Particle Definition**: Each particle has a state and weight.
- **Initialization**: Implement `randomize_uniformly` and `randomize_around_initial_state`.
- **Map Geometry**: Implement `get_distance_to_wall` to calculate predicted lidar ranges.

### Step 2: Prediction

- **Control input ($u_t$)**: Define your motion vector.
- **Transition Function**: Implement $g(x_{t-1}, u_t)$ in `propagate_state`.
- **Noise**: Add randomness to $u_t$ before propagation to account for uncertainty.

### Step 3: Correction

- **Measurement ($z_t$)**: Actual lidar readings.
- **Weight Calculation**: Update particle weights based on the likelihood of the measurement given the map.
- **Resampling**: Implement the `resample` function in `ParticleSet` to duplicate high-weight particles and eliminate low-weight ones.

### Step 4: State Estimate

- Calculate the mean state in `update_mean_state`.
- **Note**: Always use `angle_wrap` for angular calculations.

---

## 4. Offline Localization

Test your filter using logged data.

1. **Simple Trajectory**: Debug the filter on a predictable path.
2. **Validation**: Compare estimates against "truth" measurements.
3. **Ablation**: Test the prediction step alone (no corrections) to see divergence, then enable corrections.
4. **Robustness**: Test "Unknown Start" (global localization) and "Out-of-Frame" scenarios where camera data is lost.

---

## 5. Online Localization

1. **Real-time Tracking**: Ensure the state estimate and confidence ellipse appear in the GUI.
2. **Complex Trajectory**: Log data and record a video of the robot performing in the environment alongside the GUI screen capture.
3. **Final Video**: Create a picture-on-picture video for submission.

---