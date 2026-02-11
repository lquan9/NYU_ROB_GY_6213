#!/usr/bin/env python3
"""Quick script to check which trial files have actual motion data"""

from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from robot_python_code import data_handling, parameters

trial_data_dir = parameters.datapath
print(f"Trials in: {trial_data_dir}\n")

if not trial_data_dir.exists():
    trial_data_dir = Path(__file__).resolve().parent / 'data'
    print(f"Try {trial_data_dir}\n")

trial_files = data_handling.get_trial_files(trial_data_dir)
print(f"Found {len(trial_files)} trial files\n")

good_trials = []
for filename in trial_files:
    has_motion, encoder_change = data_handling.check_trial_has_motion(filename)
    status = "GOOD" if has_motion else "NO MOTION"
    print(f"{status} | Encoder: {encoder_change:6.0f} | {Path(filename).name}")
    if has_motion:
        good_trials.append(filename)

print(f"{len(good_trials)} trials with motion out of {len(trial_files)} total")
if good_trials:
    print(f"\nGood trial: {Path(good_trials[0]).name}")
