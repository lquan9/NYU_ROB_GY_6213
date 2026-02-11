"""Parameters TODO: change to yaml"""

from pathlib import Path
# External libraries
import math

# UDP parameters
localIP = "10.42.0.1" # Put your laptop computer's IP here
arduinoIP = "10.42.0.114" # Put your arduino's IP here
localPort = 4010
arduinoPort = 4010
bufferSize = 1024

# Camera parameters
camera_id = 0
marker_length = 0.071

# Robot parameters
num_robot_sensors = 2 # encoder, steering
num_robot_control_signals = 2 # speed, steering

# Logging parameters
DEBUG_PRINTS = False
max_num_lines_before_write = 50
datapath = Path(__file__).resolve().parent.parent.parent / 'data' / 'data_straight' / 'btf'
data_name_list = ['time', 'control_signal', 'robot_sensor_signal', 'camera_sensor_signal']

# Experiment trial parameters
trial_type = "steering" # "steering" or "distance"
extra_trial_log_time = 2000 # milliseconds
trial_max_speed = 40
trial_time = 4000 # milliseconds
trial_input = -15 # delta for steering, u_x for distance

# MM parameters
# counts_to_m: distance traveled per encoder count (2*pi*r / counts_per_revolution)
counts_to_m = 0.000 #TODO: calibrate
distance_variance_a = 0.0001
distance_variance_b = 0.01
steering_to_w = 0.0 #TODO: calibrate
steering_variance_a = 0.001
steering_variance_b = 0.01
wheelbase = 0.1444
track_width = 0.150
wheel_radius = 0.034
max_steer_deg = 30.0
