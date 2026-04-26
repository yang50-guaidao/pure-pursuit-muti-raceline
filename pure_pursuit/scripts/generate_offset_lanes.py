#!/usr/bin/env python3
"""Generate lane_left / lane_right / lane_optimal CSVs from a single 4-column raceline.

Input CSV format:  x, y, yaw, speed_ratio   (one row per waypoint)
Output CSVs keep the same 4-column format so the existing C++ loader works unchanged.

The path tangent is derived from successive points (not the yaw column) so the offset
direction stays correct even if the recorded yaw drifts from the true path tangent.

Usage:
    python3 generate_offset_lanes.py \
        --input  ../waypoints/race/just_a_try3.csv \
        --out-dir ../waypoints/lanes/ \
        --offset 0.3 \
        --speed-scale 0.85
"""
import argparse
import os
import sys
import numpy as np


def derive_tangents(xy: np.ndarray, closed: bool) -> np.ndarray:
    n = len(xy)
    if closed:
        prev_idx = np.roll(np.arange(n), 1)
        next_idx = np.roll(np.arange(n), -1)
        d = xy[next_idx] - xy[prev_idx]
    else:
        d = np.zeros_like(xy)
        d[1:-1] = xy[2:] - xy[:-2]
        d[0] = xy[1] - xy[0]
        d[-1] = xy[-1] - xy[-2]
    norms = np.linalg.norm(d, axis=1, keepdims=True)
    norms[norms < 1e-9] = 1.0
    return d / norms


def offset_lane(rows: np.ndarray, signed_offset: float, speed_scale: float, closed: bool) -> np.ndarray:
    xy = rows[:, :2]
    yaw = rows[:, 2]
    speed = rows[:, 3]

    tangent = derive_tangents(xy, closed)
    # Left normal of tangent (tx, ty) is (-ty, tx); right normal is (ty, -tx).
    # Caller controls sign via signed_offset (positive = left, negative = right).
    left_normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)

    new_xy = xy + signed_offset * left_normal
    new_speed = np.clip(speed * speed_scale, 0.0, 1.0)

    return np.column_stack([new_xy[:, 0], new_xy[:, 1], yaw, new_speed])


def detect_closed(xy: np.ndarray, threshold: float = 0.5) -> bool:
    return float(np.linalg.norm(xy[0] - xy[-1])) < threshold


def write_csv(path: str, rows: np.ndarray) -> None:
    with open(path, "w") as f:
        for row in rows:
            f.write(f"{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}, {row[3]:.6f}\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True, help="path to source 4-col CSV")
    p.add_argument("--out-dir", required=True, help="output directory")
    p.add_argument("--offset", type=float, default=0.3, help="lateral offset in meters")
    p.add_argument("--speed-scale", type=float, default=0.85, help="speed scale for off-optimal lanes")
    p.add_argument("--force-closed", action="store_true", help="treat path as closed loop regardless of endpoint distance")
    args = p.parse_args()

    if not os.path.isfile(args.input):
        sys.exit(f"input not found: {args.input}")

    rows = np.loadtxt(args.input, delimiter=",")
    if rows.ndim != 2 or rows.shape[1] != 4:
        sys.exit(f"expected Nx4 CSV, got shape {rows.shape}")

    closed = args.force_closed or detect_closed(rows[:, :2])
    print(f"loaded {len(rows)} waypoints, closed_loop={closed}")

    os.makedirs(args.out_dir, exist_ok=True)

    optimal_path = os.path.join(args.out_dir, "lane_optimal.csv")
    left_path = os.path.join(args.out_dir, "lane_left.csv")
    right_path = os.path.join(args.out_dir, "lane_right.csv")

    write_csv(optimal_path, rows)
    write_csv(left_path, offset_lane(rows, +args.offset, args.speed_scale, closed))
    write_csv(right_path, offset_lane(rows, -args.offset, args.speed_scale, closed))

    print(f"wrote {optimal_path}")
    print(f"wrote {left_path}  (offset=+{args.offset:.2f}m, speed_scale={args.speed_scale})")
    print(f"wrote {right_path} (offset=-{args.offset:.2f}m, speed_scale={args.speed_scale})")


if __name__ == "__main__":
    main()
