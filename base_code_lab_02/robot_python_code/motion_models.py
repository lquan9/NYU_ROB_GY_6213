# External Libraries
import math
import random
import parameters

# Motion Model constants
COUNTS_TO_METERS = parameters.counts_to_m
DISTANCE_VARIANCE_A = parameters.distance_variance_a
DISTANCE_VARIANCE_B = parameters.distance_variance_b
STEERING_TO_W = parameters.steering_to_w
STEERING_VARIANCE_A = parameters.steering_variance_a
STEERING_VARIANCE_B = parameters.steering_variance_b

# A function for obtaining variance in distance travelled as a function of distance travelled
def variance_distance_travelled_s(distance, a=DISTANCE_VARIANCE_A, b=DISTANCE_VARIANCE_B):
    var_s = max (0.0, a + b * abs(distance))

    return var_s

# Function to calculate distance from encoder counts
def distance_travelled_s(encoder_counts, counts_to_m=COUNTS_TO_METERS):
    s = encoder_counts * counts_to_m

    return s

# A function for obtaining variance in rotational velocity as a function of steering angle command
def variance_rotational_velocity_w(steering_angle_command, a=STEERING_VARIANCE_A, b=STEERING_VARIANCE_B):
    var_w = max(0.0, a + b * abs(steering_angle_command))

    return var_w

# A function for calculating rotational velocity from steering angle command
def rotational_velocity_w(steering_angle_command, steering_to_w=STEERING_TO_W):
    w = steering_angle_command * steering_to_w
    
    return w

# This class is an example structure for implementing your motion model.
class MyMotionModel:

    # Constructor, change as you see fit.
    def __init__(self, initial_state, last_encoder_count):
        self.state = initial_state
        self.last_encoder_count = last_encoder_count

    # This is the key step of your motion model, which implements x_t = f(x_{t-1}, u_t)
    def step_update(self, encoder_counts, steering_angle_command, delta_t):
        # Add student code here
        
        return self.state
    
    # This is a great tool to take in data from a trial and iterate over the data to create 
    # a robot trajectory in the global frame, using your motion model.
    def traj_propagation(self, time_list, encoder_count_list, steering_angle_list):
        x_list = [self.state[0]]
        y_list = [self.state[1]]
        theta_list = [self.state[2]]
        self.last_encoder_count = encoder_count_list[0]
        for i in range(1, len(encoder_count_list)):
            delta_t = time_list[i] - time_list[i-1]
            new_state = self.step_update(encoder_count_list[i], steering_angle_list[i], delta_t)
            x_list.append(new_state[0])
            y_list.append(new_state[1])
            theta_list.append(new_state[2])

        return x_list, y_list, theta_list
    

    # Coming soon
    def generate_simulated_traj(self, duration):
        delta_t = 0.1
        t_list = []
        x_list = []
        y_list = []
        theta_list = []
        t = 0
        encoder_counts = 0
        while t < duration:

            t += delta_t 
        return t_list, x_list, y_list, theta_list
            