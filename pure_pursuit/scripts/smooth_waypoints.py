#!/usr/bin/env python3
"""
Waypoint smoothing script using spline interpolation.

Usage:
    python3 smooth_waypoints.py <input.csv> <output.csv> [num_points] [smoothing]

Arguments:
    input.csv    - Input waypoints CSV (x, y, yaw, speed)
    output.csv   - Output smoothed waypoints CSV
    num_points   - Number of resampled waypoints (default: 500)
    smoothing    - Spline smoothing factor, higher = smoother (default: 5.0)
"""

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import sys
import os


def smooth_waypoints(input_path, output_path, num_points=500, smoothing=5.0, plot=True):
    # Load waypoints
    data = np.loadtxt(input_path, delimiter=',')
    x, y = data[:, 0], data[:, 1]

    print(f"Loaded {len(x)} waypoints from {input_path}")

    # Fit spline as closed loop
    tck, u = splprep([x, y], s=smoothing, per=True)

    # Resample at num_points evenly spaced points
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)

    # Compute yaw from spline derivs
    dx, dy = splev(u_new, tck, der=1)
    yaw_new = np.arctan2(dy, dx)

    # Save smoothed CSV -- set velocity to 0
    out = np.column_stack([x_new, y_new, yaw_new, np.zeros(num_points)])
    np.savetxt(output_path, out, fmt='%.6f', delimiter=', ')
    print(f"Saved {num_points} smoothed waypoints to {output_path}")

    # Plot comparison
    if plot:
        plot_path = os.path.splitext(output_path)[0] + '_comparison.png'
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(x, y, 'b.', markersize=3, label=f'Original ({len(x)} pts)', alpha=0.5)
        ax.plot(x_new, y_new, 'r-', linewidth=2, label=f'Smoothed ({num_points} pts)')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(f'Waypoint Smoothing Comparison (s={smoothing})')
        ax.grid(True)
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        print(f"Saved comparison plot to {plot_path}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    input_path  = sys.argv[1]
    output_path = sys.argv[2]
    num_points  = int(sys.argv[3])   if len(sys.argv) > 3 else 500
    smoothing   = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0

    smooth_waypoints(input_path, output_path, num_points, smoothing)