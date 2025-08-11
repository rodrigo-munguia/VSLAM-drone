import os
import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def load_file(file_path, trim_seconds=35.0):
    df = pd.read_csv(file_path)

    # Trim last `trim_seconds` seconds
    time_end = df['time'].max()
    trimmed_df = df[df['time'] <= (time_end - trim_seconds)]

    # Normalize time to start at 0
    time_start = trimmed_df['time'].min()
    trimmed_df['time'] = trimmed_df['time'] - time_start

    # Compute absolute errors
    errors = pd.DataFrame({
        'time': trimmed_df['time'],
        'X': (trimmed_df['X_gt'] - trimmed_df['X_e']).abs(),
        'Y': (trimmed_df['Y_gt'] - trimmed_df['Y_e']).abs(),
        'Z': (trimmed_df['Z_gt'] - trimmed_df['Z_e']).abs(),
        'XY': np.sqrt((trimmed_df['X_gt'] - trimmed_df['X_e'])**2 +
                      (trimmed_df['Y_gt'] - trimmed_df['Y_e'])**2)
    })

    return errors

def main():
    log_dir = "exp3"  # Hardcoded log folder
    files = glob.glob(os.path.join(log_dir, "*.csv"))

    # Group files by label prefix
    grouped_files = defaultdict(list)
    for file in files:
        basename = os.path.basename(file)
        if "_" in basename:
            label = basename.split('_')[0]
            grouped_files[label].append(file)

    # Per-label MAE time series
    label_mae_timeseries = defaultdict(list)

    for label, file_list in grouped_files.items():
        for file in file_list:
            mae_df = load_file(file)
            label_mae_timeseries[label].append(mae_df)

    # Create plots
    for axis in ['X', 'Y', 'Z', 'XY']:
        plt.figure(figsize=(10, 3))
        plt.title(f"MAE over Time - {axis}")
        plt.xlabel("Time (s)")
        plt.ylabel(f"{axis} MAE")
        

        for label, list_of_mae_dfs in label_mae_timeseries.items():
            # Stack all dataframes for the label
            all_times = np.array([])
            all_values = np.array([])

            for mae_df in list_of_mae_dfs:
                all_times = np.concatenate((all_times, mae_df['time'].values))
                all_values = np.concatenate((all_values, mae_df[axis].values))

            # Sort by time
            sorted_indices = np.argsort(all_times)
            all_times = all_times[sorted_indices]
            all_values = all_values[sorted_indices]

            # To align: group into bins or resample
            bin_size = 0.5  # seconds
            time_bins = np.arange(all_times.min(), all_times.max() + bin_size, bin_size)
            bin_indices = np.digitize(all_times, time_bins)

            binned_time = []
            binned_mae = []

            for i in range(1, len(time_bins)):
                values_in_bin = all_values[bin_indices == i]
                if len(values_in_bin) > 0:
                    binned_time.append(time_bins[i-1])
                    binned_mae.append(np.mean(values_in_bin))

            plt.plot(binned_time, binned_mae, label=label)

        plt.legend()
        plt.grid(True)
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
