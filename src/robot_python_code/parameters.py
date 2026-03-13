"""Robot parameters and camera calibration utilities."""

from pathlib import Path
import math
import numpy as np

#  Network
localIP     = "192.168.0.195"
arduinoIP   = "192.168.0.198"
localPort   = 4010
arduinoPort = 4010
bufferSize  = 1024

# Camera 
camera_id     = 0
camera_source = None   # set to MJPEG URL string if using a network cam
marker_length = 0.15   # 150mm marker, 6x6 ArUco, ID 0

# Intel RealSense intrinsics
camera_matrix = np.array([
    [644.10406494,   0.0,         641.55847168],
    [  0.0,        643.3069458,  372.71740723],
    [  0.0,          0.0,          1.0       ],
], dtype=np.float32)

dist_coeffs = np.array(
    [-0.05591936, 0.06711996, 0.00015107, 0.00067795, -0.02167729],
    dtype=np.float32
)

# Camera-to-world transform — calibrated from (0,0) and (0,0.5).
# Row 1 negated to fix x-axis reflection (camera has axis flip vs world frame).
camera_A = np.array([[-1.146789, 0.688073],
                     [-0.101937, 1.172273]])
camera_b = np.array([0.041284, 0.403670])
camera_theta_offset = 0.000000  # computed from calibration data at 0° heading


def camera_to_world(camera_signal):
    tx, ty, rz = camera_signal[0], camera_signal[1], camera_signal[5]
    world_xy    = camera_A @ np.array([tx, ty]) + camera_b
    return [float(world_xy[0]), float(world_xy[1]), rz + camera_theta_offset]


# def calibrate_camera_transform(world_0, cam_0, world_1, cam_1, world_2, cam_2):
#     """Compute camera_A, camera_b, and camera_theta_offset from THREE calibration points.

#     Using 3 points solves the full affine transform — no manual axis negation needed.
#     Choose points with BOTH x and y displacement, e.g.:
#         point 0: (0, 0)
#         point 1: (0.5, 0)    ← x displacement
#         point 2: (0, 0.5)    ← y displacement

#     Args:
#         world_N: [x, y]       known world position
#         cam_N:   [tx, ty, rz] camera raw reading (divide "Camera raw" by 100)

#     Example:
#         >>> from robot_python_code import parameters
#         >>> parameters.calibrate_camera_transform(
#         ...     [0, 0],     [tx0, ty0, rz0],
#         ...     [0.5, 0],   [tx1, ty1, rz1],
#         ...     [0, 0.5],   [tx2, ty2, rz2])
#     """
#     w0 = np.array(world_0, float)
#     w1 = np.array(world_1, float)
#     w2 = np.array(world_2, float)
#     c0 = np.array(cam_0[:2], float)
#     c1 = np.array(cam_1[:2], float)
#     c2 = np.array(cam_2[:2], float)

#     # Displacements from point 0
#     dc1, dc2 = c1 - c0, c2 - c0
#     dw1, dw2 = w1 - w0, w2 - w0

#     # Solve A @ [dc1 | dc2] = [dw1 | dw2]  →  A = W @ C^-1
#     C = np.column_stack([dc1, dc2])  # 2x2
#     W = np.column_stack([dw1, dw2])  # 2x2

#     if abs(np.linalg.det(C)) < 1e-10:
#         print("ERROR: calibration points are collinear in camera frame.")
#         print("Use 3 points with BOTH x and y displacement.")
#         return
#     A = W @ np.linalg.inv(C)
#     b = w0 - A @ c0

#     # Theta offset: average of (actual_heading - camera_rz) at all points
#     # Uses the 0-heading point (point 0) if all 3 have same heading
#     theta_off = -cam_0[2] if len(cam_0) > 2 else 0.0

#     # Verify all 3 points
#     preds = [A @ c + b for c in [c0, c1, c2]]
#     worlds = [w0, w1, w2]
#     errs = [np.linalg.norm(p - w) for p, w in zip(preds, worlds)]

#     det = np.linalg.det(A)
#     print("=" * 55)
#     print("   Camera Calibration Results (3-point)")
#     print("=" * 55)
#     print(f"camera_A = np.array([[{A[0,0]:.6f}, {A[0,1]:.6f}],")
#     print(f"                     [{A[1,0]:.6f}, {A[1,1]:.6f}]])")
#     print(f"camera_b = np.array([{b[0]:.6f}, {b[1]:.6f}])")
#     print(f"camera_theta_offset = {theta_off:.6f}")
#     print()
#     print(f"det(A) = {det:.4f}  ({'reflection' if det < 0 else 'rotation'}+scale)")
#     print()
#     for i, (c, p, w, e) in enumerate(zip([c0,c1,c2], preds, worlds, errs)):
#         print(f"Point {i}: cam {tuple(c.round(3))} → world {tuple(p.round(3))}  "
#               f"(target {tuple(w)})  err={e:.4f}m")

#     return A, b, theta_off


#    Robot I/O 
num_robot_sensors         = 2  # encoder, steering
num_robot_control_signals = 2  # speed, steering

#    Logging 
DEBUG_PRINTS             = False
max_num_lines_before_write = 50
datapath = Path(__file__).resolve().parent.parent.parent / 'data' / 'data_straight' / 'btf'
data_name_list = ['time', 'control_signal', 'robot_sensor_signal',
                  'camera_sensor_signal', 'state_mean', 'state_covariance']

#    Trial 
trial_type          = "steering"   # "steering" or "distance"
extra_trial_log_time = 2000        # ms
trial_max_speed     = 50
trial_time          = 5000         # ms
trial_input         = -10         # delta for steering, u_x for distance

#    Motion model 
counts_to_m          = 3518
distance_variance_a  = 0.0001
distance_variance_b  = 0.01
steering_to_w        = 0.0024
steering_variance_a  = 0.001
steering_variance_b  = 0.05  #-0.000635 
wheelbase            = 0.1444
track_width          = 0.150
wheel_radius         = 0.034
max_steer_deg        = 20.0
# Servo asymmetry correction: right turns (positive steering) have less deflection.
# Tune by comparing right-turn arc radius vs left-turn arc radius.
# 1.0 = symmetric (no correction). < 1.0 = right turns are physically smaller.
steering_scale_right = 0.7     # start here, tune up/down until right≈left radius

#    Kalman filter 
I3                   = np.eye(3)
covariance_plot_scale = 100

# TODO:
# PF parameters, modify the map and num particles as you see fit.
num_particles = 100
# lidar measurement noise variance in meters squared
#not tuned yet
distance_variance = 1.5

# in parameters.py
pf_known_start = False
pf_start_x = 0.0
pf_start_y = 0.0
pf_start_theta = 0.0
pf_start_stdev = 0.1


wall_corner_list = [
    [0.6096, 1.8288, 0, 1.8288],
    [0, 1.8288, 0, 0],
    [0, 0, 1.8288, 0],
    [1.8288, 0, 1.8288, -0.4826],
    [1.8288, -0.4826, 2.794, -0.4826],
    [2.794, -0.4826, 2.794, 0],
    [2.794, 0, 3.6576, 0],
    [3.6576, 0, 3.6576, 1.8288],
    [3.6576, 1.8288, 3.2004, 2.2352],
    [3.2004, 2.2352, 3.2004, 2.9718],
    [3.2004, 2.9718, 0.762, 2.9718],
    [0.762, 2.9718, 0.6096, 1.8288]
]