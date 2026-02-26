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

# Camera parameters
camera_id = 0
marker_length = 0.10 # we are using 100mm markersize 6x6 ID is zero
# need to update the camera matrix and dist_coeffs acc to intel realsense 
camera_matrix = np.array([[644.10406494, 0.00000000e+00 ,641.55847168],      
 [0.00000000e+00 ,643.3069458, 372.71740723],
 [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]], dtype=np.float32)
dist_coeffs = np.array([-0.05591936, 0.06711996, 0.00015107, 0.00067795, -0.02167729], dtype=np.float32)

# Camera to world frame transform
# Measured: robot at world (0,0) gave tx=-1.76, ty=0.42
#           robot at world (1.2,0) gave tx=-3.16, ty=-0.69
camera_origin_tx = -1.69        # camera tx when robot at world (0,0)
camera_origin_ty =  0.39        # camera ty when robot at world (0,0)
camera_scale     =  0.6485      # average scale from both calibration points
camera_rotation_matrix = np.array([    # full 2x2 rotation matrix
    [-0.1176, -1.5289],
    [ 0.9079, -1.2357]
])

def camera_to_world(camera_signal):
    """Transform raw camera signal [tx,ty,tz,rx,ry,rz] to world frame [x, y, theta]."""
    tx = camera_signal[0]
    ty = camera_signal[1]
    rz = camera_signal[5]
    dx = tx - camera_origin_tx
    dy = ty - camera_origin_ty
    R = camera_rotation_matrix
    x_world = camera_scale * (R[0, 0] * dx + R[0, 1] * dy)
    y_world = -camera_scale * (R[1, 0] * dx + R[1, 1] * dy)
    theta_world = rz
    return [x_world, y_world, theta_world]

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
