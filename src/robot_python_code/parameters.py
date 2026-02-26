"""Robot parameters and camera calibration utilities."""

from pathlib import Path
import math
import numpy as np

# ── Network ───────────────────────────────────────────────────────
localIP     = "192.168.0.196"
arduinoIP   = "192.168.0.198"
localPort   = 4010
arduinoPort = 4010
bufferSize  = 1024

# ── Camera ────────────────────────────────────────────────────────
camera_id     = 0
camera_source = None   # set to MJPEG URL string if using a network cam
marker_length = 0.10   # 100mm marker, 6x6 ArUco, ID 0

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

# Camera-to-world transform — run calibrate_camera_transform() to get real values.
# These are placeholders assuming camera at world (3, 2), facing -y.
camera_A = np.array([
    [-0.818397, -0.081163],
    [ 0.081163, -0.818397],
])
camera_b           = np.array([-0.331079, -0.421711])
camera_theta_offset = 0.0


def camera_to_world(camera_signal):
    tx, ty, rz = camera_signal[0], camera_signal[1], camera_signal[5]
    world_xy    = camera_A @ np.array([tx, ty]) + camera_b
    return [float(world_xy[0]), float(world_xy[1]), rz + camera_theta_offset]


def calibrate_camera_transform(world_0, cam_0, world_1, cam_1):
    """Compute camera_A, camera_b, and camera_theta_offset from two calibration points.

    Place the robot at two known world positions. At each position, read the
    raw camera output from the console (the "Camera raw" line, divide by 100)
    and pass those as cam_0 / cam_1.

    Example:
        >>> from robot_python_code import parameters
        >>> parameters.calibrate_camera_transform(
        ...     [0, 0],   [-1.80,  0.35, 0.55],
        ...     [1.1, 0], [-3.05, -0.64, 0.35])

    Copy the printed values back into this file.
    """
    w0, w1 = np.array(world_0, float), np.array(world_1, float)
    c0, c1 = np.array(cam_0[:2], float), np.array(cam_1[:2], float)

    dc, dw = c1 - c0, w1 - w0

    cam_dist = np.linalg.norm(dc)
    if cam_dist < 1e-6:
        print("ERROR: calibration points are too close in camera frame")
        return

    scale     = np.linalg.norm(dw) / cam_dist
    rotation  = math.atan2(dw[1], dw[0]) - math.atan2(dc[1], dc[0])
    cos_r, sin_r = scale * math.cos(rotation), scale * math.sin(rotation)

    A = np.array([[cos_r, -sin_r],
                  [sin_r,  cos_r]])
    b = w0 - A @ c0

    # theta_offset needs manual tuning — set it after checking rz at both poses
    theta_off = 0.0

    pred_0, pred_1 = A @ c0 + b, A @ c1 + b

    print("=" * 55)
    print("   Camera Calibration Results")
    print("=" * 55)
    print(f"camera_A = np.array([[{A[0,0]:.6f}, {A[0,1]:.6f}],")
    print(f"                     [{A[1,0]:.6f}, {A[1,1]:.6f}]])")
    print(f"camera_b = np.array([{b[0]:.6f}, {b[1]:.6f}])")
    print(f"camera_theta_offset = {theta_off:.6f}")
    print()
    print(f"Scale: {scale:.4f}   Rotation: {math.degrees(rotation):.1f}°")
    print()
    print(f"Point 0: cam {tuple(c0.round(3))} → world {tuple(pred_0.round(3))}  "
          f"(target {tuple(w0)})  err={np.linalg.norm(pred_0-w0):.4f}m")
    print(f"Point 1: cam {tuple(c1.round(3))} → world {tuple(pred_1.round(3))}  "
          f"(target {tuple(w1)})  err={np.linalg.norm(pred_1-w1):.4f}m")

    return A, b, theta_off


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
trial_max_speed     = 30
trial_time          = 3000         # ms
trial_input         = -10          # delta for steering, u_x for distance

#    Motion model 
counts_to_m          = 3518
distance_variance_a  = 0.0001
distance_variance_b  = 0.01
steering_to_w        = 0.0024
steering_variance_a  = 0.0001
steering_variance_b  = -0.000635
wheelbase            = 0.1444
track_width          = 0.150
wheel_radius         = 0.034
max_steer_deg        = 20.0

#    Kalman filter 
I3                   = np.eye(3)
covariance_plot_scale = 100