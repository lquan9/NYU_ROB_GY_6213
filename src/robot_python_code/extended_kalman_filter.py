# External libraries
import numpy as np
import math
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Local libraries
from robot_python_code import parameters
from robot_python_code import data_handling

# Main class
class ExtendedKalmanFilter:
    def __init__(self, x_0, Sigma_0, encoder_counts_0):
        self.state_mean = x_0
        self.state_covariance = Sigma_0
        self.predicted_state_mean = [0,0,0]
        self.predicted_state_covariance = parameters.I3 * 1.0
        self.last_encoder_counts = encoder_counts_0

    # Call the prediction and correction steps
    def update(self, u_t, z_t, delta_t, use_correction=True):
        # Always run prediction
        self.prediction_step(u_t, delta_t)

        # Only run correction if camera measurement is available AND we want it
        if use_correction and z_t is not None:
            self.correction_step(z_t)
        else:
            # Prediction only: accept predicted as final
            self.state_mean = self.predicted_state_mean
            self.state_covariance = self.predicted_state_covariance

    # Set the EKF's predicted state mean and covariance matrix
    def prediction_step(self, u_t, delta_t):
        x_tm1 = self.state_mean
        # Run motion model to get predicted state and distance travelled
        x_bar_t, s = self.g_function(x_tm1, u_t, delta_t)
        # Calculate predicted state covariance
        G_x = self.get_G_x(x_tm1, s)
        G_u = self.get_G_u(x_tm1, delta_t)
        # Get motion model covariance
        R_t = self.get_R(s)
        
        # Propagate covariance:
        # Sigma_bar = G_x * Sigma * G_x^T + G_u * R * G_u^T
        Sigma_tm1 = np.array(self.state_covariance, dtype=float)
        Sigma_bar = G_x @ Sigma_tm1 @ G_x.T + G_u @ R_t @ G_u.T

        self.predicted_state_mean = x_bar_t
        self.predicted_state_covariance = Sigma_bar

    # Set the EKF's corrected state mean and covariance matrix
    def correction_step(self, z_t):
        # Get predicted state mean and covariance
        x_bar_t = np.array(self.predicted_state_mean, dtype=float)
        Sigma_bar = np.array(self.predicted_state_covariance, dtype=float)

        H = self.get_H()          # 3x3: dh/dx
        Q_t = self.get_Q()        # 3x3: camera measurement noise

        # Kalman Gain: K = Sigma_bar * H^T * (H * Sigma_bar * H^T + Q)^-1
        S = H @ Sigma_bar @ H.T + Q_t
        K = Sigma_bar @ H.T @ np.linalg.inv(S)

        # Measurement function h(x) - what we expect to see
        h_x = self.get_h_function(x_bar_t)

        # Innovation: difference between actual measurement and expected
        z_t = np.array(z_t, dtype=float)
        innovation = z_t - h_x

        # Wrap theta innovation to [-pi, pi] to avoid angle jumps
        innovation[2] = math.atan2(math.sin(innovation[2]), math.cos(innovation[2]))

        # Corrected state mean
        x_t = x_bar_t + K @ innovation

        # Corrected covariance: (I - K*H) * Sigma_bar
        I = np.eye(3)
        Sigma_t = (I - K @ H) @ Sigma_bar

        self.state_mean = x_t.tolist()
        self.state_covariance = Sigma_t

    # Function to calculate distance from encoder counts
    def distance_travelled_s(self, encoder_counts):
        # Get encoder counts from previous step
        encoder_counts_tm1 = self.last_encoder_counts
        # Calculate distance travelled
        s = (encoder_counts - encoder_counts_tm1) / parameters.counts_to_m
        # Update last encoder counts
        self.last_encoder_counts = encoder_counts
        return s    
            
    # Function to calculate rotational velocity from steering and dist travelled or speed
    def rotational_velocity_w(self, steering_angle_command):  
        w = steering_angle_command * parameters.steering_to_w
        return w

    # The nonlinear transition equation that provides new states from past states
    def g_function(self, x_tm1, u_t, delta_t):
        x, y, theta = x_tm1[0], x_tm1[1], x_tm1[2]
        encoder_counts = u_t[0]
        steering_angle = u_t[1]

        # Distance travelled this step
        s = self.distance_travelled_s(encoder_counts)

        # Linear velocity
        v = s / delta_t if delta_t > 0 else 0.0

        # Angular velocity from Ackermann geometry
        steer_rad = math.radians(steering_angle)
        if abs(steer_rad) > 1e-6 and parameters.wheelbase > 1e-6:
            omega = v * math.tan(steer_rad) / parameters.wheelbase
        else:
            omega = 0.0

        # Discrete-time state update
        theta_t = theta + omega * delta_t
        x_t = x + v * math.cos(theta_t) * delta_t
        y_t = y + v * math.sin(theta_t) * delta_t
        

        return [x_t, y_t, theta_t], s
    
    # The nonlinear measurement function
    def get_h_function(self, x_t):
        return np.array(x_t, dtype=float)
    
    # This function returns a matrix with the partial derivatives dg/dx
    # g outputs x_t, y_t, theta_t, and we take derivatives wrt inputs x_tm1, y_tm1, theta_tm1
    def get_G_x(self, x_tm1, s): 
        theta = x_tm1[2]

        G_x = np.array([
            [1, 0, -s * math.sin(theta)],
            [0, 1,  s * math.cos(theta)],
            [0, 0,  1                  ]
        ], dtype=float)

        return G_x      

    # This function returns a matrix with the partial derivatives dg/du
    def get_G_u(self, x_tm1, delta_t):
        theta = x_tm1[2]

        G_u = np.array([
            [math.cos(theta), 0      ],
            [math.sin(theta), 0      ],
            [0,               delta_t]
        ], dtype=float)

        return G_u                

    # This function returns a matrix with the partial derivatives dh_t/dx_t
    def get_H(self):
        H = np.eye(3, dtype=float)
        return H
    
    # This function returns the R_t matrix which contains transition function covariance terms.
    def get_R(self, s):
        var_s = parameters.distance_variance_a   # 0.0015 m² constant
        var_w = parameters.distance_variance_b   # 0.000366 rad²/s²

        R = np.array([
            [var_s, 0    ],
            [0,     var_w]
        ], dtype=float)

        return R

    # This function returns the Q_t matrix which contains measurement covariance terms.
    def get_Q(self):
        var_x     = 0.01    # meters² - replace after camera experiment
        var_y     = 0.01    # meters² - replace after camera experiment
        var_theta = 0.05   # rad²/s² (0.12 deg²/s² converted)
        return np.array([
            [var_x,  0,      0        ],
            [0,      var_y,  0        ],
            [0,      0,      var_theta]
        ], dtype=float)

class KalmanFilterPlot:

    def __init__(self):
        self.dir_length = 0.1
        fig, ax = plt.subplots()
        self.ax = ax
        self.fig = fig

    def update(self, state_mean, state_covaraiance):
        plt.clf()

        # Plot covariance ellipse
        lambda_, v = np.linalg.eig(state_covaraiance)
        lambda_ = np.sqrt(lambda_)
        xy = (state_mean[0], state_mean[1])
        angle=np.rad2deg(np.arctan2(*v[:,0][::-1]))
        ell = Ellipse(xy, alpha=0.5, facecolor='red',width=lambda_[0], height=lambda_[1], angle = angle)
        ax = self.fig.gca()
        ax.add_artist(ell)
        
        # Plot state estimate
        plt.plot(state_mean[0], state_mean[1],'ro')
        plt.plot([state_mean[0], state_mean[0]+ self.dir_length*math.cos(state_mean[2]) ], [state_mean[1], state_mean[1]+ self.dir_length*math.sin(state_mean[2]) ],'r')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.axis([-4, 12, -6, 6])
        plt.grid()
        plt.draw()
        plt.pause(0.1)


# Code to run your EKF offline with a data file.
def offline_efk(use_correction=True):

    # Get data to filter
    
    # filename = './data/data_straight/btf/robot_data_40_15_10_02_26_23_13_58.pkl'
    filename = '/home/jeddak1729/Documents/rlan/labs/btf-robot/data/data_straight/btf/robot_data_60_10_10_02_26_23_37_05.pkl'

    ekf_data = data_handling.get_file_data_for_kf(filename)

    # Instantiate PF with no initial guess
    x_0 = [ekf_data[0][3][0]+.5, ekf_data[0][3][1], ekf_data[0][3][5]]
    Sigma_0 = np.diag([0.25, 0.25, 0.1]) # Sigma_0 = parameters.I3
    encoder_counts_0 = ekf_data[0][2].encoder_counts
    extended_kalman_filter = ExtendedKalmanFilter(x_0, Sigma_0, encoder_counts_0)

    # Create plotting tool for ekf
    kalman_filter_plot = KalmanFilterPlot()

    # Loop over data
    for t in range(1, len(ekf_data)):
        row = ekf_data[t]
        delta_t = ekf_data[t][0] - ekf_data[t-1][0]           # time step
        u_t = np.array([row[2].encoder_counts, row[2].steering])  # encoder, steering
        z_t = np.array([row[3][0], row[3][1], row[3][5]])         # camera x, y, theta

        # Run EKF for this time step
        # Pass use_correction=False for Stage 1 (prediction only)
        # Pass use_correction=True  for Stage 2 (full EKF)
        extended_kalman_filter.update(u_t, z_t, delta_t, use_correction=use_correction)

        # Plot
        kalman_filter_plot.update(extended_kalman_filter.state_mean, extended_kalman_filter.state_covariance[0:2, 0:2])



if __name__ == "__main__":
    offline_efk(use_correction=False)   # Start with prediction only!
