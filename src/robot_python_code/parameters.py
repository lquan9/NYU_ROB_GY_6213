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
# Camera source: use an integer for a local device (e.g. 0, 1),
# or an MJPEG stream URL for a network camera:
#   camera_source = "http://192.168.0.100:8090/video"
# When set to None, falls back to camera_id.
camera_source = None
marker_length = 0.10 # we are using 100mm markersize 6x6 ID is zero
# need to update the camera matrix and dist_coeffs acc to intel realsense 
camera_matrix = np.array([[1.41089024e+03, 0.00000000e+00 ,5.34757040e+02],      
 [0.00000000e+00 ,1.40977771e+03, 4.63300611e+02],
 [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]], dtype=np.float32)
dist_coeffs = np.array([-0.32511173, -0.09273864 ,-0.00295959 , 0.00111094 , 0.2446519 ], dtype=np.float32)


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
trial_max_speed = 40
trial_time = 5000 # milliseconds
trial_input = 2 # delta for steering, u_x for distance

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
