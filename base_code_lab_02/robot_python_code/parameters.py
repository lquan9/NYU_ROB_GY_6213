# External libraries
import math

# UDP parameters
localIP = "10.42.0.1" # Put your laptop computer's IP here
arduinoIP = "10.42.0.114" # Put your arduino's IP here
# localIP = "192.168.1.3" # Put your laptop computer's IP here
# arduinoIP = "192.168.1.5" # Put your arduino's IP here
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
max_num_lines_before_write = 50
filename_start = './data/robot_data'
data_name_list = ['time', 'control_signal', 'robot_sensor_signal', 'camera_sensor_signal']

# Experiment trial parameters
trial_type = "steering" # "steering" or "distance"
extra_trial_log_time = 2000 # milliseconds
trial_max_speed = 40
trial_time = 4000 # milliseconds
trial_input = -15 # delta for steering, u_x for distance

# MM parameters
counts_to_m = 0.
distance_variance_a = 0.
distance_variance_b = 0.
steering_to_w = 0.
steering_variance_a = 0.
steering_variance_b = 0.
wheelbase = 0.1444
track_width = 0.150
wheel_radius = 0.034
max_steer_deg = 0.
