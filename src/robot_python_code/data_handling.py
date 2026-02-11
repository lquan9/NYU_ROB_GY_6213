"""Data handling"""
# External Libraries
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Local libraries
from robot_python_code import motion_models, robot, parameters

def get_file_data(filename):
    """ Open a file and return data in a form ready to plot"""
    data_loader = robot.DataLoader(filename)
    data_dict = data_loader.load()

    # The dictionary should have keys ['time', 'control_signal', 'robot_sensor_signal', 'camera_sensor_signal']
    time_list = data_dict['time']
    control_signal_list = data_dict['control_signal']
    robot_sensor_signal_list = data_dict['robot_sensor_signal']
    encoder_count_list = []
    velocity_list = []
    steering_angle_list = []
    measured_steering_list = []

    if parameters.DEBUG_PRINTS:
        print(f"Data dict keys: {data_dict.keys()}")
        print(f"Number of sensor signals: {len(robot_sensor_signal_list)}")
        print(f"First sensor signal type: {type(robot_sensor_signal_list[0])}")
        if hasattr(robot_sensor_signal_list[0], '__dict__'):
            print(f"First sensor signal attributes: {robot_sensor_signal_list[0].__dict__}")

    for row in robot_sensor_signal_list:
        encoder_count_list.append(row.encoder_counts)
        measured_steering_list.append(row.steering)
    for row in control_signal_list:
        velocity_list.append(row[0])
        steering_angle_list.append(row[1])

    if parameters.DEBUG_PRINTS:
        print(f"Encoder from sensors: {encoder_count_list[:5]} ... {encoder_count_list[-3:]}")
        print(f"Measured steering from sensors: {measured_steering_list[:5]} ... {measured_steering_list[-3:]}")
        print(f"Commanded steering from controls: {steering_angle_list[:5]} ... {steering_angle_list[-3:]}")

    return time_list, encoder_count_list, velocity_list, steering_angle_list

def get_trial_files(trial_data_dir):
    """Return sorted trial log files from data path."""
    trial_path = Path(trial_data_dir)
    if not trial_path.exists():
        return []
    return sorted(str(path) for path in trial_path.glob('robot_data_*.pkl'))

def check_trial_has_motion(trial_filename):
    """Check if a trial file has motion data"""
    try:
        _, encoder_count_list, _, _ = get_file_data(trial_filename)
        encoder_change = max(encoder_count_list) - min(encoder_count_list)
        has_motion = encoder_change > 10
        return has_motion, encoder_change
    except Exception as e:
        print(f"Error checking {trial_filename}: {e}")
        return False, 0

def normalize_time(time_list):
    """Normalize time to start at 0 and convert from milliseconds to seconds."""
    if len(time_list) > 0:
        time_start = time_list[0]
        time_normalized = [(t - time_start) for t in time_list]

        if len(time_normalized) > 1 and time_normalized[-1] > 1000:
            time_normalized = [t / 1000.0 for t in time_normalized]
        return time_normalized
    return time_list

def plot_trial_basics(fig, trial_filename):
    """For a given trial, plot the encoder counts, velocities, steering angles"""
    time_list, encoder_count_list, velocity_list, steering_angle_list = get_file_data(trial_filename)

    time_normalized = normalize_time(time_list)

    # determine time range
    if hasattr(parameters, 'trial_time') and parameters.trial_time:
        max_time = math.ceil(parameters.trial_time / (1000.0) * 1.5)
    elif len(time_normalized) > 0:
        min_len = min(len(time_normalized), len(encoder_count_list), len(velocity_list), len(steering_angle_list))
        time_normalized = time_normalized[:min_len]
        encoder_count_list = encoder_count_list[:min_len]
        velocity_list = velocity_list[:min_len]
        steering_angle_list = steering_angle_list[:min_len]
        max_time = time_normalized[-1]
    else:
        max_time = 10

    fig.patch.set_facecolor('black')
    fig.clf()

    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(time_normalized, encoder_count_list)
    ax1.set_title('Encoder Values', color='white')
    ax1.set_xlabel('Time (s)', color='white')
    ax1.set_ylabel('Encoder Counts', color='white')
    ax1.set_facecolor('black')
    ax1.tick_params(colors='white')
    ax1.grid(True, color='gray', alpha=0.3)
    ax1.set_xlim(0, max_time)

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(time_normalized, velocity_list)
    ax2.set_title('Speed', color='white')
    ax2.set_xlabel('Time (s)', color='white')
    ax2.set_ylabel('Speed', color='white')
    ax2.set_facecolor('black')
    ax2.tick_params(colors='white')
    ax2.grid(True, color='gray', alpha=0.3)
    ax2.set_xlim(0, max_time)

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(time_normalized, steering_angle_list)
    ax3.set_title('Steering', color='white')
    ax3.set_xlabel('Time (s)', color='white')
    ax3.set_ylabel('Steering', color='white')
    ax3.set_facecolor('black')
    ax3.tick_params(colors='white')
    ax3.grid(True, color='gray', alpha=0.3)
    ax3.set_xlim(0, max_time)

    fig.tight_layout()


def run_my_model_on_trial(fig, trial_filename, plot_color='c-'):
    """Plot a trajectory using the motion model, input data from a single trial."""
    if parameters.DEBUG_PRINTS:
        print(f"Loading file: {trial_filename}")
    time_list, encoder_count_list, velocity_list, steering_angle_list = get_file_data(trial_filename)
    if parameters.DEBUG_PRINTS:
        print(f"Data loaded - {len(time_list)} time steps")
        print(f"Time range: {time_list[0]} to {time_list[-1]}")
        print(f"Encoder range: {encoder_count_list[0]} to {encoder_count_list[-1]}")
        print(f"Encoder samples (first 10): {encoder_count_list[:10]}")
        print(f"Encoder samples (last 10): {encoder_count_list[-10:]}")
        print(f"Velocity samples (first 10): {velocity_list[:10]}")
        print(f"Steering samples (first 10): {steering_angle_list[:10]}")

    time_normalized = normalize_time(time_list)
    if parameters.DEBUG_PRINTS:
        print(f"Time normalized: {time_normalized[0]:.3f} to {time_normalized[-1]:.3f} seconds")
        print(f"First 5 time steps: {time_normalized[:5]}")

    motion_model = motion_models.AckermannMM([0, 0, 0])
    if parameters.DEBUG_PRINTS:
        print(f"Motion model initialized at [0, 0, 0]")
    x_list, y_list, _ = motion_model.traj_propagation(time_normalized,
                                                               encoder_count_list,
                                                               steering_angle_list)
    if parameters.DEBUG_PRINTS:
        print(f"Trajectory computed - {len(x_list)} points")
        print(f"X range: {min(x_list):.4f} to {max(x_list):.4f}")
        print(f"Y range: {min(y_list):.4f} to {max(y_list):.4f}")
        print(f"First 5 positions: X={x_list[:5]}, Y={y_list[:5]}")
        print(f"Last 5 positions: X={x_list[-5:]}, Y={y_list[-5:]}")

    fig.patch.set_facecolor('black')
    fig.clf()

    ax = fig.add_subplot(1, 1, 1)
    ax.plot(x_list, y_list, plot_color, linewidth=2)
    ax.set_title('Motion Model Predicted XY Traj (m)', color='white')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.set_facecolor('black')
    ax.tick_params(colors='white')
    ax.grid(True, color='gray', alpha=0.3)

    # set axis limits based on data
    if len(x_list) > 0 and len(y_list) > 0:
        x_margin = (max(x_list) - min(x_list)) * 0.1 if max(x_list) != min(x_list) else 0.5
        y_margin = (max(y_list) - min(y_list)) * 0.1 if max(y_list) != min(y_list) else 0.5
        ax.set_xlim(min(x_list) - x_margin, max(x_list) + x_margin)
        ax.set_ylim(min(y_list) - y_margin, max(y_list) + y_margin)
    ax.set_aspect('equal', adjustable='box')

    fig.tight_layout()


def plot_many_trial_predictions(trial_dir):
    """Iterate through many trials and plot them as trajectories with motion model"""
    if parameters.DEBUG_PRINTS:
        print(f"Plotting trials from directory: {trial_dir}")

    plot_color_list = ['r.','k.','g.','c.', 'b.',
                       'r.','k.','g.','c.', 'b.',
                       'r.','k.','g.','c.', 'b.',
                       'r.','k.','g.','c.', 'b.']
    count = 0
    for item in trial_dir.iterdir():
        trial_filename = item.name
        plot_color = plot_color_list[count]
        run_my_model_on_trial(trial_dir + trial_filename, False, plot_color)
        count += 1
    plt.show()

def run_my_model_to_predict_distance(trial_filename):
    """ Calculate the predicted distance from single trial for a motion model."""
    time_list, encoder_count_list, _, steering_angle_list = get_file_data(trial_filename)
    motion_model = motion_models.AckermannMM([0,0,0], 0)
    x_list, _, _ = motion_model.traj_propagation(time_list, encoder_count_list, steering_angle_list)
    distance = x_list[-30]

    return distance

def get_diff_squared(m_list,p_list):
    """Calculate the differences between two lists and square them"""
    diff_squared_list = []
    for i in range(len(m_list)):
        diff_squared = math.pow(m_list[i]-p_list[i],2)
        diff_squared_list.append(diff_squared)

    coefficients = np.polyfit(m_list, diff_squared_list, 2)
    p=np.poly1d(coefficients)

    plt.plot(m_list, diff_squared_list,'ko')
    plt.plot(m_list, p(m_list),'ro')
    plt.title("Error Squared (m^2)")
    plt.xlabel('Measured distance travelled (m)')
    plt.ylabel('(Actual - Predicted)^2 (m^2)')
    plt.show()

    return diff_squared_list


def process_files_and_plot(file_data_list, data_directory):
    """Open files, plot them to predict with the motion model and compare with real values"""
    predicted_distance_list = []
    measured_distance_list = []
    for row in file_data_list:
        trial_filename = row[0]
        measured_distance = row[1]
        measured_distance_list.append(measured_distance)
        predicted_distance = run_my_model_to_predict_distance(data_directory + trial_filename)
        predicted_distance_list.append(predicted_distance)

    # Plot predicted and measured distance travelled.
    plt.plot(measured_distance_list+[0], predicted_distance_list+[0], 'ko')
    plt.plot([0,1.7],[0,1.7])
    plt.title('Distance Trials')
    plt.xlabel('Measured Distance (m)')
    plt.ylabel('Predicted Distance (m)')
    plt.legend(['Measured vs Predicted', 'Slope 1 Line'])
    plt.show()

    # Plot the associated variance
    get_diff_squared(measured_distance_list, predicted_distance_list)


def sample_model(fig, num_samples=200):
    """Sample and plot some simulated trials"""
    fig.patch.set_facecolor('black')
    fig.clf()

    ax = fig.add_subplot(1, 1, 1)
    traj_duration = 10
    for i in range(num_samples):
        model = motion_models.AckermannMM([0,0,0], 0)
        traj_x, traj_y, _ = model.generate_simulated_traj(traj_duration)
        ax.plot(traj_x, traj_y, 'k.', markersize=1)

    ax.set_title('Sampling the model', color='white')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.set_facecolor('black')
    ax.tick_params(colors='white')
    ax.grid(True, color='gray', alpha=0.3)

    fig.tight_layout()

def get_trial_metrics(trial_files):
    """Compute aggregate metrics for trial files used in dashboard plots."""
    trial_metrics = []
    for trial_file in trial_files:
        trial_time, trial_encoder, trial_velocity, trial_steering = get_file_data(trial_file)
        if len(trial_time) < 2:
            continue
        duration = trial_time[-1] - trial_time[0]
        net_encoder = trial_encoder[-1] - trial_encoder[0]
        avg_speed = float(np.mean(trial_velocity))
        avg_abs_steer = float(np.mean(np.abs(trial_steering)))
        trial_metrics.append({
            'filename': Path(trial_file).name,
            'duration': duration,
            'net_encoder': net_encoder,
            'avg_speed': avg_speed,
            'avg_abs_steer': avg_abs_steer,
        })

    return trial_metrics

def plot_trial_aggregates(fig, trial_metrics):
    """Render aggregate scatter and duration plots."""
    fig.patch.set_facecolor('black')
    fig.clf()

    if not trial_metrics:
        ax = fig.add_subplot(1, 1, 1)
        ax.text(0.5, 0.5, 'No valid trial metrics found', ha='center', va='center', color='white')
        ax.set_facecolor('black')
        ax.set_axis_off()
        return

    durations = [row['duration'] for row in trial_metrics]
    encoder_net = [row['net_encoder'] for row in trial_metrics]
    avg_speeds = [row['avg_speed'] for row in trial_metrics]
    avg_abs_steers = [row['avg_abs_steer'] for row in trial_metrics]

    ax1 = fig.add_subplot(1, 2, 1)
    scatter = ax1.scatter(avg_speeds, encoder_net, c=avg_abs_steers, cmap='viridis')
    ax1.set_title('Net Encoder vs Avg Speed', color='white')
    ax1.set_xlabel('Avg Speed Command', color='white')
    ax1.set_ylabel('Net Encoder Counts', color='white')
    ax1.set_facecolor('black')
    ax1.tick_params(colors='white')
    ax1.grid(True, color='gray', alpha=0.3)
    cbar = fig.colorbar(scatter, ax=ax1, label='Avg |Steering|')
    cbar.ax.yaxis.set_tick_params(color='white')
    cbar.ax.yaxis.label.set_color('white')
    cbar.outline.set_edgecolor('white')
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(range(len(trial_metrics)), durations, 'o-', color='orange')
    ax2.set_title('Trial Duration', color='white')
    ax2.set_xlabel('Trial Index', color='white')
    ax2.set_ylabel('Duration (s)', color='white')
    ax2.set_facecolor('black')
    ax2.tick_params(colors='white')
    ax2.grid(True, color='gray', alpha=0.3)

    fig.tight_layout()

######### MAIN ########

# Some sample data to test with
files_and_data = [
    ['robot_data_60_0_28_01_26_13_41_44.pkl', 67/100], # filename, measured distance in meters
    ['robot_data_60_0_28_01_26_13_43_41.pkl', 68/100],
    ['robot_data_60_0_28_01_26_13_37_15.pkl', 113/100],
    ['robot_data_60_0_28_01_26_13_35_18.pkl', 107/100],
    ['robot_data_60_0_28_01_26_13_41_10.pkl', 65/100],
    ['robot_data_60_0_28_01_26_13_42_55.pkl', 70/100],
    ['robot_data_60_0_28_01_26_13_39_36.pkl', 138/100],
    ['robot_data_60_0_28_01_26_13_42_19.pkl', 69/100],
    ['robot_data_60_0_28_01_26_13_36_10.pkl', 109/100],
    ['robot_data_60_0_28_01_26_13_33_20.pkl', 100/100],
    ['robot_data_60_0_28_01_26_13_34_28.pkl', 103/100],
    ]

# Plot the motion model predictions for a single trial
if False:
    filename = './data_straight/robot_data_60_0_28_01_26_13_36_10.pkl'
    run_my_model_on_trial(filename)

# Plot the motion model predictions for each trial in a folder
if False:
    directory = ('./data_straight/')
    plot_many_trial_predictions(directory)

# A list of files to open, process, and plot - for comparing predicted with actual distances
if False:
    directory = ('./data_straight/')    
    process_files_and_plot(files_and_data, directory)

# Try to sample with the motion model
if False:
    sample_model(200)
