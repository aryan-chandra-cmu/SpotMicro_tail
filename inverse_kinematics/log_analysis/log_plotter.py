# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 27th March 2026

"""
Log plotter for the Spot Micro quadruped.

Reads CSV log files produced by DataLogger and generates plots
for every data category:
    1. Target foot positions (x, y, z) vs time
    2. Joint angles (hip, shoulder, knee) vs time
    3. Servo angles (hip, shoulder, knee) vs time

Usage
-----
    python log_plotter.py
"""

import os
import sys
import glob
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Category definitions – each entry maps a subplot title to its CSV columns.
# ---------------------------------------------------------------------------
CATEGORIES = {
    "Target Foot Position": {
        "columns": ["target_x (mm)", "target_y (mm)", "target_z (mm)"],
        "ylabel": "Position (mm)",
    },
    "Joint Angles": {
        "columns": ["joint_hip (deg)", "joint_shoulder (deg)", "joint_knee (deg)"],
        "ylabel": "Angle (deg)",
    },
    "Servo Angles": {
        "columns": ["servo_hip (deg)", "servo_shoulder (deg)", "servo_knee (deg)"],
        "ylabel": "Angle (deg)",
    },
}

TIME_COL = "time (s)"


def find_latest_log(logs_dir: str):
    """Return the path to the most recent ik_log_*.csv in *logs_dir*."""
    pattern = os.path.join(logs_dir, "ik_log_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(
            f"No ik_log_*.csv files found in {os.path.abspath(logs_dir)}"
        )
    return max(files)


def plot_log(csv_path: str):
    """
    Read a CSV log file and produce one figure window per category.

    Each figure contains subplots for the individual columns in that category.
    """
    df = pd.read_csv(csv_path)
    time = df[TIME_COL]

    for title, info in CATEGORIES.items():
        cols = info["columns"]
        fig, axes = plt.subplots(len(cols), 1, figsize=(10, 3 * len(cols)), sharex=True)

        for ax, col in zip(axes, cols):
            label = col.split(" (")[0]
            ax.plot(time, df[col], linewidth=1.5)
            ax.set_ylabel(info["ylabel"])
            ax.set_title(label)
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle(title, fontsize=14, fontweight="bold")
        plt.tight_layout()

    plt.show()


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(script_dir, "..", "logs")

    print("=== Spot Micro Log Plotter ===")
    print("  1) Plot the latest log file")
    print("  2) Enter a filename from the logs folder")
    choice = input("Select an option (1 or 2): ").strip()

    if choice == "1":
        csv_path = find_latest_log(logs_dir)
        print(f"Latest log found: {csv_path}")
    elif choice == "2":
        filename = input("Enter the log filename (e.g. ik_log_2026-03-27_160000.csv): ").strip()
        csv_path = os.path.join(logs_dir, filename)
    else:
        print("Invalid option. Exiting.")
        sys.exit(1)

    print(f"Reading {csv_path} ...")
    plot_log(csv_path)


if __name__ == "__main__":
    main()
