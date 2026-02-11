"""Calibration script for motion model parameters"""

import json
import sys
from pathlib import Path
import numpy as np
from importlib.resources import files

sys.path.insert(0, str(Path(__file__).parent / 'src'))

from robot_python_code import data_handling, parameters

def get_config_path():
    """Get the path to the config files"""
    try:
        config_files = files('robot_python_code').joinpath('configs')
        if config_files.is_dir():
            return config_files / 'calibrate_base.json'
    except (TypeError, AttributeError, FileNotFoundError):
        pass

    # fallback
    config_path = Path(__file__).resolve().parents[2] / 'configs' / 'calibrate_base.json'
    if config_path.exists():
        return config_path

    return Path(__file__).parent / 'configs' / 'calibrate_base.json'

def load_config(config_file=None):
    """Load calibration configuration"""
    if config_file is None:
        config_file = get_config_path()

    config_path = Path(config_file)
    with open(config_path, 'r') as f:
        config = json.load(f)

    # Store config file location for resolving relative paths
    config['_config_dir'] = config_path.parent
    return config

def calibrate_encoder(config):
    """Calibrate counts_to_m parameter from straight-line trials"""

    data_dir = Path(config['data_directory'])
    # Resolve relative paths relative to config file location
    if not data_dir.is_absolute():
        data_dir = (config['_config_dir'] / data_dir).resolve()

    trials = config['encoder_trials']

    results = []

    for trial in trials:
        log_file = data_dir / trial['log_file']
        print(f"Processing: {log_file.name}")

        if not log_file.exists():
            print(f"Log file not found: {log_file}")
            continue

        # load trial data
        time_list, encoder_list, velocity_list, steering_list = data_handling.get_file_data(str(log_file))

        # encoder change
        encoder_start = encoder_list[0]
        encoder_end = encoder_list[-1]
        encoder_change = abs(encoder_end - encoder_start)

        # ground truth distance
        ground_truth_distance = trial['distance_m']

        # counts_to_m
        if encoder_change > 0:
            counts_to_m = ground_truth_distance / encoder_change
            if parameters.DEBUG_PRINTS:
                print(f"Ground truth distance: {ground_truth_distance:.3f} m")
                print(f"Encoder change: {encoder_change} counts")
                print(f"Calculated counts_to_m: {counts_to_m:.6f} m/count")

            results.append({
                'log_file': trial['log_file'],
                'counts_to_m': counts_to_m,
                'distance': ground_truth_distance,
                'encoder_change': encoder_change
            })
        else:
            print(f"No encoder change detected")

    if results:
        # calculate statistics
        counts_to_m_values = [r['counts_to_m'] for r in results]
        mean_counts_to_m = np.mean(counts_to_m_values)
        std_counts_to_m = np.std(counts_to_m_values)
        if parameters.DEBUG_PRINTS:
            print(f"Number of trials: {len(results)}")
            print(f"Mean counts_to_m: {mean_counts_to_m:.6f} m/count")
            print(f"Std deviation: {std_counts_to_m:.6f} m/count")
            print(f"Coefficient of variation: {(std_counts_to_m/mean_counts_to_m)*100:.2f}%")

        return {
            'parameter': 'counts_to_m',
            'value': mean_counts_to_m,
            'std': std_counts_to_m,
            'trials': results
        }
    else:
        print("No valid trials found for encoder calibration")
        return None

def calibrate_steering(config):
    """Calibrate steering from circular path trials"""

    data_dir = Path(config['data_directory'])
    # Resolve relative paths relative to config file location
    if not data_dir.is_absolute():
        data_dir = (config['_config_dir'] / data_dir).resolve()

    trials = config['calibration_trials']['steering_angular']

    results = []

    for trial in trials:
        if parameters.DEBUG_PRINTS:
            print(f"Processing: {trial['name']}")
        log_file = data_dir / trial['log_file']

        if not log_file.exists():
            print(f"Log file not found: {log_file}")
            continue

        time_list, encoder_list, velocity_list, steering_list = data_handling.get_file_data(str(log_file))

        measured_radius = trial['ground_truth']['measured_radius_m']
        steering_cmd = trial['ground_truth']['steering_command']

        # calculate distance traveled
        time_normalized = data_handling.normalize_time(time_list)
        total_time = time_normalized[-1] - time_normalized[0]

        encoder_change = abs(encoder_list[-1] - encoder_list[0])

        # use current counts_to_m estimate or from config
        if parameters.counts_to_m > 0:
            distance_traveled = encoder_change * parameters.counts_to_m
        else:
            print(f"Encoders not calibrated yet, skipping steering calibration")
            continue

        angle_traveled = distance_traveled / measured_radius
        avg_angular_velocity = angle_traveled / total_time if total_time > 0 else 0

        # steering_to_w
        if abs(steering_cmd) > 0:
            steering_to_w = avg_angular_velocity / abs(steering_cmd)

            results.append({
                'trial': trial['name'],
                'steering_to_w': steering_to_w,
                'steering_cmd': steering_cmd,
                'radius': measured_radius
            })

    if results:
        steering_to_w_values = [r['steering_to_w'] for r in results]
        mean_steering_to_w = np.mean(steering_to_w_values)
        std_steering_to_w = np.std(steering_to_w_values)

        return {
            'parameter': 'steering_to_w',
            'value': mean_steering_to_w,
            'std': std_steering_to_w,
            'trials': results
        }
    else:
        print("No valid trials found for steering calibration")
        return None

def save_results(config, encoder_result, steering_result):
    """Save calibration results"""
    if not config['output']['save_results']:
        return

    output_file = Path(config['output']['results_file'])

    results = {
        'encoder_calibration': encoder_result,
        'steering_calibration': steering_result
    }

    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)

    print(f"Results saved to: {output_file}")
