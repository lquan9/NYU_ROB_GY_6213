"""Calibration script for motion model parameters"""

import json
import sys
from pathlib import Path
import numpy as np
from importlib.resources import files
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')

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

        # counts_to_m (counts per meter)
        if encoder_change > 0:
            counts_to_m = encoder_change / ground_truth_distance
            if parameters.DEBUG_PRINTS:
                print(f"Ground truth distance: {ground_truth_distance:.3f} m")
                print(f"Encoder change: {encoder_change} counts")
                print(f"Calculated counts_to_m: {counts_to_m:.0f} counts/m")

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

        # identify outliers using z-score method
        z_scores = np.abs((np.array(counts_to_m_values) - mean_counts_to_m) / std_counts_to_m) if std_counts_to_m > 0 else np.zeros(len(counts_to_m_values))
        outlier_threshold = 1.2  # threshold
        outlier_indices = np.where(z_scores > outlier_threshold)[0]

        outlier_files = []
        for idx in outlier_indices:
            results[idx]['is_outlier'] = True
            results[idx]['z_score'] = z_scores[idx]
            outlier_files.append(results[idx]['log_file'])

        if len(outlier_indices) > 0:
            print(f"\n{len(outlier_indices)} outlier(s) detected (z-score > {outlier_threshold}):")
            for idx in outlier_indices:
                r = results[idx]
                print(f"- {r['log_file']}: counts_to_m = {r['counts_to_m']:.1f} (z-score: {z_scores[idx]:.2f})")
            print(f"Mean: {outlier_threshold} range: {mean_counts_to_m:.1f} +/- {outlier_threshold * std_counts_to_m:.1f} counts/m\n")

        encoder_counts = np.array([r['encoder_change'] for r in results])
        distances = np.array([r['distance'] for r in results])

        plt.style.use('dark_background')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        fig.patch.set_facecolor('black')

        # plot normal points
        normal_mask = np.ones(len(results), dtype=bool)
        normal_mask[outlier_indices] = False
        ax1.scatter(encoder_counts[normal_mask], distances[normal_mask],
                   color='cyan', s=50, label='Normal Data', zorder=3)

        if len(outlier_indices) > 0:
            ax1.scatter(encoder_counts[outlier_indices], distances[outlier_indices],
                       color='red', s=100, marker='x', linewidths=3,
                       label=f'Outliers (n={len(outlier_indices)})', zorder=4)

        # fitted line
        max_encoder = np.max(encoder_counts) * 1.1
        encoder_line = np.array([0, max_encoder])
        distance_line = encoder_line / mean_counts_to_m
        ax1.plot(encoder_line, distance_line, 'r-', linewidth=2, label=f's = e / {mean_counts_to_m:.1f}')

        ax1.set_xlabel('Encoder Counts', color='white', fontsize=12)
        ax1.set_ylabel('Distance [m]', color='white', fontsize=12)
        ax1.set_title('Distance vs Encoder', color='white', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3, color='gray')
        ax1.legend(facecolor='black', edgecolor='white')
        ax1.set_facecolor('black')

        # calc variance
        predicted_distances = encoder_counts / mean_counts_to_m
        residuals = distances - predicted_distances
        squared_errors = residuals ** 2

        # sigma_s^2 = a + b * e
        # fit sigma_s^2 = distance_variance_a + distance_variance_b * encoder_counts
        A = np.vstack([np.ones(len(encoder_counts)), encoder_counts]).T
        variance_coeffs, _, _, _ = np.linalg.lstsq(A, squared_errors, rcond=None)
        distance_variance_a = variance_coeffs[0]
        distance_variance_b = variance_coeffs[1]

        # variance vs encoder
        ax2.scatter(encoder_counts[normal_mask], squared_errors[normal_mask],
                   color='yellow', s=50, label='Normal Data', zorder=3)

        if len(outlier_indices) > 0:
            ax2.scatter(encoder_counts[outlier_indices], squared_errors[outlier_indices],
                       color='red', s=100, marker='x', linewidths=3,
                       label=f'Outliers (n={len(outlier_indices)})', zorder=4)

        # fit variance line
        predicted_variance = distance_variance_a + distance_variance_b * encoder_line
        ax2.plot(encoder_line, predicted_variance, 'r-', linewidth=2,
                label=f'sigma^2 = {distance_variance_a:.6f} + {distance_variance_b:.6f} * e')

        ax2.set_xlabel('Encoder Counts', color='white', fontsize=12)
        ax2.set_ylabel('Variance [m^2]', color='white', fontsize=12)
        ax2.set_title('Variance vs Encoder', color='white', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, color='gray')
        ax2.legend(facecolor='black', edgecolor='white')
        ax2.set_facecolor('black')

        plt.tight_layout()

        # Save plot
        output_dir = Path('docs') / 'assets' / 'calibration_plots'
        output_dir.mkdir(exist_ok=True, parents=True)
        plot_file = output_dir / 'encoder_calibration.png'
        plt.savefig(plot_file, facecolor='black', edgecolor='white', dpi=150)
        print(f"Calibration plot saved to: {plot_file}")
        plt.close()

        if parameters.DEBUG_PRINTS:
            print(f"Calibration Results:")
            print(f"Number of trials: {len(results)}")
            print(f"Mean counts_to_m: {mean_counts_to_m:.1f} counts/m")
            print(f"Std deviation: {std_counts_to_m:.1f} counts/m")
            print(f"Coefficient of variation: {(std_counts_to_m/mean_counts_to_m)*100:.2f}%")
            print(f"Distance variance_a: {distance_variance_a:.6f} m^2")
            print(f"Distance variance_b: {distance_variance_b:.6f} m^2/count")

            if len(outlier_indices) > 0:
                for i, r in enumerate(results):
                    outlier_marker = " [OUTLIER]" if r.get('is_outlier', False) else ""
                    print(f"{i+1}. {Path(r['log_file']).name}: {r['counts_to_m']:.1f} counts/m{outlier_marker}")

        return {
            'parameter': 'counts_to_m',
            'value': mean_counts_to_m,
            'std': std_counts_to_m,
            'distance_variance_a': distance_variance_a,
            'distance_variance_b': distance_variance_b,
            'outliers': outlier_files,
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
            distance_traveled = encoder_change / parameters.counts_to_m
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
