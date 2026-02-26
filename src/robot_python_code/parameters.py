"""Parameters TODO: change to yaml"""

from pathlib import Path
# External libraries
import math
import numpy as np

# UDP parameters
localIP = "192.168.0.196" # Put your laptop computer's IP here
arduinoIP = "192.168.0.198" # Put your arduino's IP here
localPort = 4010
arduinoPort = 4010
bufferSize = 1024

# ── ArUco markers ───────────────────────────────────────────────
# Robot tag (on top of the robot — the thing we're tracking)
robot_marker_id = 0
robot_marker_length = 0.10     # 100 mm

# World-origin tag (on the floor — defines the global frame)
world_marker_id = 1            # change to 2 if you use ID 2
world_marker_length = 0.20     # adjust to actual printed size (meters)

# Legacy alias
marker_length = robot_marker_length

# ── Camera A  ──────────────────────────────────────────
camera_a = {
    'name': 'Camera A',
    'source': None,            # None → uses camera_id; or "http://<ip>:8090/video"
    'camera_id': 0,
    'camera_matrix': np.array([
        [644.10406494, 0.0,          641.55847168],
        [0.0,          643.3069458,  372.71740723],
        [0.0,          0.0,          1.0         ]
    ], dtype=np.float32),
    'dist_coeffs': np.array(
        [-0.05591936, 0.06711996, 0.00015107, 0.00067795, -0.02167729], dtype=np.float32),
}

# ── Camera B — set source to None to disable ───────
camera_b = {
    'name': 'Camera B',
    'source': None,            # e.g. "http://<ip-b>:8090/video"
    'camera_id': 0,
    'camera_matrix': np.array([
        [644.10406494, 0.0,          641.55847168],
        [0.0,          643.3069458,  372.71740723],
        [0.0,          0.0,          1.0         ]
    ], dtype=np.float32),
    'dist_coeffs': np.array(
        [-0.05591936, 0.06711996, 0.00015107, 0.00067795, -0.02167729], dtype=np.float32),
}

# ── Backward-compat aliases (Camera A) ─────────────────────────
camera_id = camera_a['camera_id']
camera_source = camera_a['source']
camera_matrix = camera_a['camera_matrix']
dist_coeffs = camera_a['dist_coeffs']


def relative_pose(rvec_ref, tvec_ref, rvec_target, tvec_target):
    """Compute target pose in reference tag's frame.

    Compute the target tag's position and yaw in the reference tag's coordinate system.

    Returns [x, y, theta] of target in reference frame (2D ground plane).
    """
    import cv2 as _cv2
    # Build 4x4 transforms: tag-in-camera
    R_ref, _ = _cv2.Rodrigues(np.array(rvec_ref, dtype=np.float64))
    R_tgt, _ = _cv2.Rodrigues(np.array(rvec_target, dtype=np.float64))
    t_ref = np.array(tvec_ref, dtype=np.float64).reshape(3, 1)
    t_tgt = np.array(tvec_target, dtype=np.float64).reshape(3, 1)

    # T_ref_in_cam and T_tgt_in_cam
    # target_in_ref = inv(T_ref) @ T_tgt
    # inv(T_ref): R^T, -R^T @ t
    R_ref_inv = R_ref.T
    t_ref_inv = -R_ref.T @ t_ref

    # Position of target in reference frame
    pos = R_ref_inv @ t_tgt + t_ref_inv
    x_world = float(pos[0])
    y_world = float(pos[1])

    # Rotation of target in reference frame
    R_rel = R_ref_inv @ R_tgt
    theta_world = float(math.atan2(R_rel[1, 0], R_rel[0, 0]))

    return [x_world, y_world, theta_world]


def camera_to_world(camera_signal, cam_cfg=None):
    """Legacy transform"""
    # For new data, use relative_pose()
    tx = camera_signal[0]
    ty = camera_signal[1]
    rz = camera_signal[5]
    return [tx, ty, rz]

# Robot parameters
num_robot_sensors = 2 # encoder, steering
num_robot_control_signals = 2 # speed, steering

# Logging parameters
DEBUG_PRINTS = False
max_num_lines_before_write = 50
datapath = Path(__file__).resolve().parent.parent.parent / 'data' / 'data_straight' / 'btf'
data_name_list = ['time', 'control_signal', 'robot_sensor_signal', 'camera_sensor_signal', 'state_mean', 'state_covariance']

# Experiment trial parameters
trial_type = "steering" # "steering" or "distance"
extra_trial_log_time = 2000 # milliseconds
trial_max_speed = 30
trial_time = 3000 # milliseconds
trial_input = -10 # delta for steering, u_x for distance

# MM parameters
counts_to_m = 3518
distance_variance_a = 0.0001              #0.0001
distance_variance_b = 0.01            #0.01
steering_to_w = 0.0024
steering_variance_a = 0.0001
steering_variance_b = -0.000635
wheelbase = 0.1444
track_width = 0.150
wheel_radius = 0.034
max_steer_deg = 20.0

# KF parameters
I3 = np.array([[1, 0, 0],[0, 1, 0], [0, 0, 1]])
covariance_plot_scale = 100
