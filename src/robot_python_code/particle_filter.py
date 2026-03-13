# External libraries
import copy
import matplotlib.pyplot as plt
import math
import numpy as np
from pathlib import Path
import random

# Local libraries
from robot_python_code import parameters, data_handling, motion_models


# Helper function to make sure all angles are between -pi and pi
def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

# Helper class to store and manipulate your states.
class State:

    # Constructor
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # Get the euclidean distance between 2 states
    def distance_to(self, other_state):
        return math.sqrt(math.pow(self.x - other_state.x, 2) + math.pow(self.y - other_state.y, 2))
        
    # Get the distance squared between two states
    def distance_to_squared(self, other_state):
        return math.pow(self.x - other_state.x, 2) + math.pow(self.y - other_state.y, 2)

    # return a deep copy of the state.
    def deepcopy(self):
        return copy.deepcopy(self)
        
    # Print the state
    def print(self):
        print("State: ", self.x, self.y, self.theta)


# Class to store walls as objects (specifically when represented as line segments in a 2D map.)
class Wall:

    # Constructor
    def __init__(self, wall_corners):
        self.corner1 = State(wall_corners[0], wall_corners[1], 0)
        self.corner2 = State(wall_corners[2], wall_corners[3], 0)
        self.corner1_mm = State(wall_corners[0] * 1000, wall_corners[1] * 1000, 0)
        self.corner2_mm = State(wall_corners[2] * 1000, wall_corners[3] * 1000, 0)

        self.m = (wall_corners[3] - wall_corners[1]) / (0.0001 + wall_corners[2] - wall_corners[0])
        self.b = wall_corners[3] - self.m * wall_corners[2]
        self.b_mm = wall_corners[3] * 1000 - self.m * wall_corners[2] * 1000
        self.length = self.corner1.distance_to(self.corner2)
        self.length_mm_squared = self.corner1_mm.distance_to_squared(self.corner2_mm)

        if self.m > 1000:
            self.vertical = True
        else:
            self.vertical = False
        if abs(self.m) < 0.1:
            self.horizontal = True
        else:
            self.horizontal = False


# A class to store 2D maps
class Map:

    def __init__(self, wall_corner_list):
        self.wall_list = []
        for wall_corners in wall_corner_list:
            self.wall_list.append(Wall(wall_corners))
        min_x = 999999
        max_x = -99999
        min_y = 999999
        max_y = -99999
        for wall in self.wall_list:
            min_x = min(min_x, min(wall.corner1.x, wall.corner2.x))
            max_x = max(max_x, max(wall.corner1.x, wall.corner2.x))
            min_y = min(min_y, min(wall.corner1.y, wall.corner2.y))
            max_y = max(max_y, max(wall.corner1.y, wall.corner2.y))
        border = 0.5
        self.plot_range = [min_x - border, max_x + border, min_y - border, max_y + border]
        self.particle_range = [min_x, max_x, min_y, max_y]

    # Get the closest distance from a state to any wall in the map
    def closest_distance_to_walls(self, state):
        closest_distance = 999999999999
        for wall in self.wall_list:
            closest_distance = self.get_distance_to_wall(state, wall, closest_distance)
        return closest_distance
        
    # Function to get distance to a wall from a state, in the direction of the state's theta angle.
    # Or return the distance currently believed to be the closest if its closer.
    def get_distance_to_wall(self, state, wall, closest_distance):
        # ray origin is the particle position
        rx = state.x
        ry = state.y
        # ray direction from the particle heading angle
        dx = math.cos(state.theta)
        dy = math.sin(state.theta)

        # wall segment endpoints
        x1 = wall.corner1.x
        y1 = wall.corner1.y
        x2 = wall.corner2.x
        y2 = wall.corner2.y

        # wall direction vector
        wx = x2 - x1
        wy = y2 - y1

        # solve ray_origin + t*ray_dir = wall_start + u*wall_dir using 2d cross product
        denom = dx * wy - dy * wx

        # ray and wall are parallel, no intersection
        if abs(denom) < 1e-10:
            return closest_distance

        # t = distance along ray, u = position along wall segment [0,1]
        t = ((x1 - rx) * wy - (y1 - ry) * wx) / denom
        u = ((x1 - rx) * dy - (y1 - ry) * dx) / denom

        # t must be positive (in front of particle) and u in [0,1] (on the segment)
        if t > 0 and 0 <= u <= 1:
            if t < closest_distance:
                return t

        return closest_distance


class Particle:

    def __init__(self):
        self.state = State(0, 0, 0)
        self.weight = 1
        
    # Function to create a new random particle state within a range
    def randomize_uniformly(self, xy_range):
        # scatter particle randomly across the whole map
        x = random.uniform(xy_range[0], xy_range[1])
        y = random.uniform(xy_range[2], xy_range[3])
        theta = random.uniform(-math.pi, math.pi)
        self.state = State(x, y, theta)
        self.weight = 1

    # Function to create a new random particle state with a normal distribution
    def randomize_around_initial_state(self, initial_state, state_stdev):
        # scatter particle in a gaussian cloud around a known starting pose
        x = random.gauss(initial_state.x, state_stdev.x)
        y = random.gauss(initial_state.y, state_stdev.y)
        theta = angle_wrap(random.gauss(initial_state.theta, state_stdev.theta))
        self.state = State(x, y, theta)
        self.weight = 1
        
    # Function to take a particle and "randomly" propagate it forward according to a motion model.
    def propagate_state(self, last_state, delta_encoder_counts, steering, delta_t):
        # get distance from encoder counts with noise
        s = motion_models.distance_travelled_s(delta_encoder_counts)
        var_s = motion_models.variance_distance_travelled_s(s)
        s_noisy = s + random.gauss(0, math.sqrt(var_s))

        # get angular velocity from steering with noise
        w = motion_models.rotational_velocity_w(steering)
        var_w = motion_models.variance_rotational_velocity_w(steering)
        w_noisy = w + random.gauss(0, math.sqrt(var_w))

        # kinematic update from last state
        theta = angle_wrap(last_state.theta + w_noisy * delta_t)
        x = last_state.x + s_noisy * math.cos(theta)
        y = last_state.y + s_noisy * math.sin(theta)

        self.state = State(x, y, theta)
        
    # Function to determine a particles weight based how well the lidar measurement matches up with the map.
    def calculate_weight(self, lidar_signal, map):
        weight = 1.0
        for i in range(len(lidar_signal.angles)):
            measured_distance = lidar_signal.convert_hardware_distance(lidar_signal.distances[i])
            ray_angle = lidar_signal.convert_hardware_angle(lidar_signal.angles[i])

            # skip bad readings outside useful range
            if measured_distance <= 0.05 or measured_distance >= 10.0:
                continue

            # build a state pointing in the direction of this lidar ray
            ray_state = State(self.state.x, self.state.y, angle_wrap(self.state.theta + ray_angle))

            expected_distance = map.closest_distance_to_walls(ray_state)

            # ray hit nothing in the map, skip it
            if expected_distance > 100:
                continue

            weight *= self.gaussian(expected_distance, measured_distance)

        # floor weight to avoid full collapse
        self.weight = max(weight, 1e-300)

    def gaussian(self, expected_distance, distance):
        return math.exp(-math.pow(expected_distance - distance, 2) / 2 / parameters.distance_variance)

    # Deep copy the particle
    def deepcopy(self):
        return copy.deepcopy(self)
        
    # Print the particle
    def print(self):
        print("Particle: ", self.state.x, self.state.y, self.state.theta, " w: ", self.weight)


# This class holds the collection of particles.
class ParticleSet:
    
    # Constructor, which calls the known start or unknown start initialization.
    def __init__(self, num_particles, xy_range, initial_state, state_stdev, known_start_state):
        self.num_particles = num_particles
        self.particle_list = []
        if known_start_state:
            self.generate_initial_state_particles(initial_state, state_stdev)
        else:
            self.generate_uniform_random_particles(xy_range)
        self.mean_state = State(0, 0, 0)
        self.update_mean_state()
        
    # Function to reset particles and random locations in the workspace.
    def generate_uniform_random_particles(self, xy_range):
        for i in range(self.num_particles):
            random_particle = Particle()
            random_particle.randomize_uniformly(xy_range)
            self.particle_list.append(random_particle)

    # Function to reset particles, normally distributed around the initial state. 
    def generate_initial_state_particles(self, initial_state, state_stdev):
        for i in range(self.num_particles):
            random_particle = Particle()
            random_particle.randomize_around_initial_state(initial_state, state_stdev)
            self.particle_list.append(random_particle)

    # Function to resample the particles set, i.e. make a new one with more copies of particles with higher weights.  
    def resample(self, max_weight):
        total_weight = sum(p.weight for p in self.particle_list)
        if total_weight <= 0:
            return

        normalized_weights = [p.weight / total_weight for p in self.particle_list]

        # systematic resampling, evenly spaced draws starting from a random offset
        n = self.num_particles
        step = 1.0 / n
        start = random.uniform(0, step)

        cumulative = []
        running = 0
        for w in normalized_weights:
            running += w
            cumulative.append(running)

        new_particles = []
        cumulative_index = 0
        for i in range(n):
            target = start + i * step
            while cumulative_index < n - 1 and cumulative[cumulative_index] < target:
                cumulative_index += 1
            new_particles.append(self.particle_list[cumulative_index].deepcopy())

        self.particle_list = new_particles

    def update_mean_state(self):
        n = len(self.particle_list)
        if n == 0:
            return

        mean_x = sum(p.state.x for p in self.particle_list) / n
        mean_y = sum(p.state.y for p in self.particle_list) / n

        # circular mean for theta so angle wrapping doesnt break the average
        sin_sum = sum(math.sin(p.state.theta) for p in self.particle_list)
        cos_sum = sum(math.cos(p.state.theta) for p in self.particle_list)
        mean_theta = math.atan2(sin_sum / n, cos_sum / n)

        self.mean_state.x = mean_x
        self.mean_state.y = mean_y
        self.mean_state.theta = mean_theta

    def print_particles(self):
        for particle in self.particle_list:
            particle.print()
        print()

# Class to hold the particle filter and its functions.
class ParticleFilter:
    
    # Constructor
    def __init__(self, num_particles, map, initial_state, state_stdev, known_start_state, encoder_counts_0):
        self.map = map
        self.particle_set = ParticleSet(num_particles, map.particle_range, initial_state, state_stdev, known_start_state)
        self.state_estimate = self.particle_set.mean_state
        self.state_estimate_list = []
        self.last_time = 0
        self.last_encoder_counts = encoder_counts_0

    # Update the states given new measurements
    def update(self, odometery_signal, measurement_signal, delta_t):
        self.prediction(odometery_signal, delta_t)
        # if len(measurement_signal.angles)>0:
        if measurement_signal is not None and len(measurement_signal.angles) > 0:
            self.correction(measurement_signal)
        self.particle_set.update_mean_state()
        self.state_estimate_list.append(self.state_estimate.deepcopy())

    # Predict the current state from the last state.
    def prediction(self, odometry_signal, delta_t):
        current_encoder_counts = odometry_signal[0]
        steering = odometry_signal[1]
        delta_encoder_counts = current_encoder_counts - self.last_encoder_counts
        self.last_encoder_counts = current_encoder_counts

        for particle in self.particle_set.particle_list:
            last_state = particle.state.deepcopy()
            particle.propagate_state(last_state, delta_encoder_counts, steering, delta_t)

    def correction(self, measurement_signal):
        max_weight = 0
        for particle in self.particle_set.particle_list:
            particle.calculate_weight(measurement_signal, self.map)
            if particle.weight > max_weight:
                max_weight = particle.weight

        self.particle_set.resample(max_weight)

    def print_state_estimate(self):
        print("Mean state: ", self.particle_set.mean_state.x, self.particle_set.mean_state.y, self.particle_set.mean_state.theta)


# Class to help with plotting PF data.
class ParticleFilterPlot:

    # Constructor
    def __init__(self, map):
        self.dir_length = 0.1
        fig, ax = plt.subplots()
        self.ax = ax
        self.fig = fig
        self.map = map

    # Clear and update the plot with new PF data
    def update(self, state_mean, particle_set, lidar_signal, hold_show_plot):
        plt.clf()
        
        # Plot walls
        for wall in self.map.wall_list:
            plt.plot([wall.corner1.x, wall.corner2.x], [wall.corner1.y, wall.corner2.y], 'k')

        # Plot lidar
        for i in range(len(lidar_signal.angles)):
            distance = lidar_signal.convert_hardware_distance(lidar_signal.distances[i])
            angle = lidar_signal.convert_hardware_angle(lidar_signal.angles[i]) + state_mean.theta
            x_ray = [state_mean.x, state_mean.x + distance * math.cos(angle)]
            y_ray = [state_mean.y, state_mean.y + distance * math.sin(angle)]
            plt.plot(x_ray, y_ray, 'r')

        plt.plot(state_mean.x, state_mean.y, 'ro')
        plt.plot([state_mean.x, state_mean.x + self.dir_length * math.cos(state_mean.theta)],
                 [state_mean.y, state_mean.y + self.dir_length * math.sin(state_mean.theta)], 'r')
        x_particles, y_particles = self.to_plot_data(particle_set)
        plt.plot(x_particles, y_particles, 'g.')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.axis(self.map.plot_range)
        plt.grid()
        if hold_show_plot:
            plt.show()
        else:
            plt.draw()
            plt.pause(0.1)

    # Helper function to make the particles easy to plot.
    def to_plot_data(self, particle_set):
        x_list = []
        y_list = []
        for p in particle_set.particle_list:
            x_list.append(p.state.x)
            y_list.append(p.state.y)
        return x_list, y_list


# Function used to test your PF offline with logged data.
def offline_pf():
    
    # Make a map of walls
    map = Map(parameters.wall_corner_list)

    # Get data to filter
    # filename = './data/robot_data_0_0_25_02_26_21_41_33.pkl'
    project_root = Path(__file__).resolve().parent.parent.parent
    filename = str(project_root / 'data' / 'data_straight' / 'btf' / 'robot_data_50_-10_12_03_26_19_35_53.pkl')
    pf_data = data_handling.get_file_data_for_pf(filename)

    particle_filter = ParticleFilter(
        parameters.num_particles, map,
        initial_state=State(parameters.pf_start_x, parameters.pf_start_y, parameters.pf_start_theta),
        state_stdev=State(parameters.pf_start_stdev, parameters.pf_start_stdev, parameters.pf_start_stdev),
        known_start_state=parameters.pf_known_start,
        encoder_counts_0=pf_data[0][2].encoder_counts
    )

    particle_filter_plot = ParticleFilterPlot(map)

    # Loop over pf data
    for t in range(1, len(pf_data)):
        row = pf_data[t]
        delta_t = pf_data[t][0] - pf_data[t - 1][0]
        u_t = np.array([row[2].encoder_counts, row[2].steering])
        z_t = row[2]

        particle_filter.update(u_t, z_t, delta_t)
        particle_filter_plot.update(particle_filter.particle_set.mean_state, particle_filter.particle_set, z_t, False)

    particle_filter_plot.update(particle_filter.particle_set.mean_state, particle_filter.particle_set, z_t, False)


if __name__ == '__main__':

    offline_pf()
