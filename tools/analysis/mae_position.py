import os
import pandas as pd
import glob
from collections import defaultdict

def compute_mae_per_file(file_path):
    df = pd.read_csv(file_path)

    # Compute mean absolute errors between each ground truth and estimated column
    mae = {
        'X': (df['X_gt'] - df['X_e']).abs().mean(),
        'Y': (df['Y_gt'] - df['Y_e']).abs().mean(),
        'Z': (df['Z_gt'] - df['Z_e']).abs().mean()
    }
    return mae

def main():
    # Hardcoded directory containing CSV files
    log_dir = "exp3"
    files = glob.glob(os.path.join(log_dir, "*.csv"))

    # Group files by label prefix
    grouped_files = defaultdict(list)
    for file in files:
        basename = os.path.basename(file)
        if "_" in basename:
            label = basename.split('_')[0]
            grouped_files[label].append(file)

    # Compute per-group average MAEs
    for label, file_list in grouped_files.items():
        print(f"\nLabel: {label}")
        maes = {'X': [], 'Y': [], 'Z': []}
        
        for file in file_list:
            file_mae = compute_mae_per_file(file)
            formatted_mae = {axis: f"{file_mae[axis]:.4f}" for axis in file_mae}
            print(f"  {os.path.basename(file)}: {formatted_mae}")
            for axis in ['X', 'Y', 'Z']:
                maes[axis].append(file_mae[axis])

        avg_mae = {axis: sum(maes[axis]) / len(maes[axis]) for axis in ['X', 'Y', 'Z']}
        formatted_avg = {axis: f"{avg_mae[axis]:.4f}" for axis in avg_mae}
        print(f"  → Average MAE over {len(file_list)} files: {formatted_avg}")

if __name__ == "__main__":
    main()
