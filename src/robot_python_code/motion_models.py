"""Motion Models for our robot"""

# External Libraries
import math
import random
from dataclasses import dataclass

# Local libraries
from . import parameters

# Motion Model constants
COUNTS_TO_METERS = parameters.counts_to_m
DISTANCE_VARIANCE_A = parameters.distance_variance_a
DISTANCE_VARIANCE_B = parameters.distance_variance_b
STEERING_TO_W = parameters.steering_to_w
STEERING_VARIANCE_A = parameters.steering_variance_a
STEERING_VARIANCE_B = parameters.steering_variance_b

@dataclass
class AckermannParams:
    """
    AckermannParams
    """
    wheelbase: float = parameters.wheelbase
    track_width: float = parameters.track_width
    wheel_radius: float = parameters.wheel_radius
    max_steer_deg: float = parameters.max_steer_deg

def variance_distance_travelled_s(distance, a=DISTANCE_VARIANCE_A, b=DISTANCE_VARIANCE_B):
    """A function for obtaining variance in distance travelled as a function of distance travelled"""
    var_s = max (0.0, a + b * abs(distance))

    return var_s

def distance_travelled_s(encoder_counts, counts_to_m=COUNTS_TO_METERS):
    """Function to calculate distance from encoder counts"""
    s = encoder_counts * counts_to_m

    return s

def variance_rotational_velocity_w(steering_angle_command, a=STEERING_VARIANCE_A, b=STEERING_VARIANCE_B):
    """A function for obtaining variance in rotational velocity as a function of steering angle command"""
    var_w = max(0.0, a + b * abs(steering_angle_command))

    return var_w

def rotational_velocity_w(steering_angle_command, steering_to_w=STEERING_TO_W):
    """A function for calculating rotational velocity from steering angle command"""
    w = steering_angle_command * steering_to_w

    return w

class AckermannMM:
    """Ackermann Kinematic Motion Model"""
    def __init__(self, initial_state,
                 params: AckermannParams | None = None,
                 last_encoder_count=0):
        self.params = params or AckermannParams()
        self.state = initial_state
        self.last_encoder_count = last_encoder_count

    def step_update(self, delta_t,
                    encoder,
                    v=0.0,
                    steer=0.0,
        )-> tuple[float, float, float]:
        """Step update takes encoder, steering, and delta_t to produce a new state estimate"""

        # encoder delta update
        delta_count = encoder - self.last_encoder_count
        self.last_encoder_count = encoder

        # distance travelled
        s = distance_travelled_s(delta_count)
        var_s = variance_distance_travelled_s(s)
        s += random.gauss(0, math.sqrt(var_s))

        # rotational v update
        w = rotational_velocity_w(steer)
        var_w = variance_rotational_velocity_w(steer)
        w += random.gauss(0, math.sqrt(var_w))

        # clamp to max steer
        steer = max(-self.params.max_steer_deg,
                    min(self.params.max_steer_deg, steer))

        # v from s
        v = s / delta_t if delta_t > 0 else 0.0

        # kinematic update
        x, y, theta = self.state

        # compute angular velocity: \omega = V_g * tan(\delta) / L
        omega = 0.0
        steer_rad = math.radians(steer)

        if abs(steer_rad) > 1e-6 and self.params.wheelbase > 1e-6:
            omega = v * math.tan(steer_rad) / self.params.wheelbase

        # discrete-time state update
        theta += omega * delta_t
        x += v * math.cos(theta) * delta_t
        y += v * math.sin(theta) * delta_t

        self.state = [x, y, theta]
        return self.state

    def traj_propagation(self, time_list, encoder_count_list, steering_angle_list):
        """This is a great tool to take in data from a trial and iterate over the data to create
        a robot trajectory in the global frame, using your motion model."""
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
