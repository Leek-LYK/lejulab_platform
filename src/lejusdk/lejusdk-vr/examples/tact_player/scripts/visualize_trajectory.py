#!/usr/bin/env python3
"""
Visualize tact trajectory from CSV files.
Usage: python3 visualize_trajectory.py /tmp/打招呼_45_trajectory.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(csv_path, joints=None, save_path=None):
    """Plot trajectory from CSV file."""
    df = pd.read_csv(csv_path)

    # Determine number of joints from columns
    pos_cols = [c for c in df.columns if c.endswith('_pos_deg')]
    n_joints = len(pos_cols)

    if joints is None:
        # Default: show left arm (j0-j6) and right arm (j7-j13)
        joints = list(range(min(14, n_joints)))

    # Split into left and right arm
    left_arm = [j for j in joints if j < 7]
    right_arm = [j for j in joints if j >= 7]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)

    # Left arm
    for j in left_arm:
        axes[0, 0].plot(df['time_s'], df[f'j{j}_pos_deg'], label=f'j{j}')
        axes[1, 0].plot(df['time_s'], df[f'j{j}_vel_deg_s'], label=f'j{j}')
        axes[2, 0].plot(df['time_s'], df[f'j{j}_acc_deg_s2'], label=f'j{j}')

    axes[0, 0].set_ylabel('Position (deg)')
    axes[0, 0].set_title('Left Arm (j0-j6)')
    axes[0, 0].legend(loc='upper right')
    axes[0, 0].grid(True)

    axes[1, 0].set_ylabel('Velocity (deg/s)')
    axes[1, 0].legend(loc='upper right')
    axes[1, 0].grid(True)

    axes[2, 0].set_ylabel('Acceleration (deg/s²)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].legend(loc='upper right')
    axes[2, 0].grid(True)

    # Right arm
    for j in right_arm:
        axes[0, 1].plot(df['time_s'], df[f'j{j}_pos_deg'], label=f'j{j}')
        axes[1, 1].plot(df['time_s'], df[f'j{j}_vel_deg_s'], label=f'j{j}')
        axes[2, 1].plot(df['time_s'], df[f'j{j}_acc_deg_s2'], label=f'j{j}')

    axes[0, 1].set_ylabel('Position (deg)')
    axes[0, 1].set_title('Right Arm (j7-j13)')
    axes[0, 1].legend(loc='upper right')
    axes[0, 1].grid(True)

    axes[1, 1].set_ylabel('Velocity (deg/s)')
    axes[1, 1].legend(loc='upper right')
    axes[1, 1].grid(True)

    axes[2, 1].set_ylabel('Acceleration (deg/s²)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].legend(loc='upper right')
    axes[2, 1].grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")

    plt.show()

def plot_keyframes(csv_path, joint_idx=9, save_path=None):
    """Plot keyframe positions with control points for a specific joint."""
    df = pd.read_csv(csv_path)

    j = joint_idx
    time_col = 'time_s'
    pos_col = f'j{j}_deg'
    left_x_col = f'j{j}_left_cp_x'
    left_y_col = f'j{j}_left_cp_y'
    right_x_col = f'j{j}_right_cp_x'
    right_y_col = f'j{j}_right_cp_y'

    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot keyframe positions
    ax.scatter(df[time_col], df[pos_col], s=100, c='blue', zorder=5, label='Keyframes')

    # Plot control points
    for i, row in df.iterrows():
        t = row[time_col]
        pos = row[pos_col]

        # Left control point (incoming)
        left_t = t + row[left_x_col] * 0.01  # Convert keyframe offset to seconds
        left_pos = pos + row[left_y_col]
        ax.scatter(left_t, left_pos, s=30, c='green', marker='^', zorder=4)
        ax.plot([left_t, t], [left_pos, pos], 'g--', alpha=0.5)

        # Right control point (outgoing)
        right_t = t + row[right_x_col] * 0.01
        right_pos = pos + row[right_y_col]
        ax.scatter(right_t, right_pos, s=30, c='red', marker='v', zorder=4)
        ax.plot([t, right_t], [pos, right_pos], 'r--', alpha=0.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (deg)')
    ax.set_title(f'Joint {j} Keyframes with Control Points')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")

    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_trajectory.py <trajectory.csv> [keyframes.csv]")
        sys.exit(1)

    trajectory_csv = sys.argv[1]
    base_name = trajectory_csv.replace('_trajectory.csv', '')

    print(f"Loading trajectory from: {trajectory_csv}")
    plot_trajectory(trajectory_csv, save_path=f"{base_name}_plot.png")

    # If keyframes CSV is provided, also plot it
    keyframes_csv = f"{base_name}_keyframes.csv"
    if len(sys.argv) > 2:
        keyframes_csv = sys.argv[2]

    try:
        print(f"\nLoading keyframes from: {keyframes_csv}")
        plot_keyframes(keyframes_csv, joint_idx=9, save_path=f"{base_name}_keyframes_plot.png")
    except Exception as e:
        print(f"Could not plot keyframes: {e}")
