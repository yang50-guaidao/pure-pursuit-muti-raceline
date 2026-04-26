# pure-pursuit-muti-raceline

Multi-raceline pure pursuit controller for the [F1TENTH](https://f1tenth.org/) platform. Loads several precomputed raceline trajectories in parallel and switches between them at runtime when the active line is blocked by an opponent vehicle.

Built on top of a single-line C++ pure pursuit node and the `f1tenth_gym_ros` simulator.

## Features

- **Multi-lane runtime selection**: load N raceline CSVs and switch among them based on opponent occupancy. Last lane in the list has the highest priority.
- **Reactive lane switching**: every control tick, scan a forward window of waypoints on each lane; mark a lane as blocked if any waypoint is within `lane_occupied_dist` of the opponent. Pick the highest-priority unblocked lane, with a configurable cooldown to prevent flapping.
- **Backward compatibility**: set `opponent_topic: ""` and the node behaves as a single-lane pure pursuit identical to the upstream baseline.
- **Lane generator script**: `generate_offset_lanes.py` builds parallel left/right lanes from any 4-column raceline CSV by offsetting along the path normal.
- **Hot-swap waypoints**: `/pure_pursuit/update_raceline` service for replacing the active lane's waypoints at runtime.
- **RRT path override**: subscribes to `rrt_path` and follows it when fresh, otherwise falls back to raceline tracking.

## Repository layout

```
pure_pursuit/
├── src/pure_pursuit_node.cpp        # main C++ node
├── src/pure_pursuit_boost.cpp       # alternative implementation
├── scripts/generate_offset_lanes.py # CSV lane generator
├── scripts/waypoint_logger.py       # record waypoints from odom
├── scripts/smooth_waypoints.py      # spline-smooth a recorded path
├── waypoints/lanes/                 # generated multi-lane CSVs
├── waypoints/race/                  # source single-line racelines
├── config/zach_params/*.yaml        # ROS parameter files (sim & real)
└── launch/zach/{sim,real}_launch.py # launch entry points
```

CSV format is 4 columns: `x, y, yaw, speed_ratio` where `speed_ratio ∈ [0, 1]` is mapped to `[slow_speed, fast_speed]` at runtime.

## Build

Standard ROS 2 (Humble) workspace build:

```bash
cd ~/sim_ws
colcon build --packages-select pure_pursuit_multi
source install/setup.bash
```

## Run (simulation)

Requires `f1tenth_gym_ros` running with `num_agent: 2` to expose `/opp_racecar/odom`.

```bash
# terminal 1: simulator (configure num_agent: 2 in its sim.yaml first)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# terminal 2: the controller
ros2 launch pure_pursuit_multi sim_launch.py
```

## Generating lane CSVs

```bash
python3 src/pure_pursuit/scripts/generate_offset_lanes.py \
  --input  src/pure_pursuit/waypoints/race/<your_raceline>.csv \
  --out-dir src/pure_pursuit/waypoints/lanes/ \
  --offset 0.3 \
  --speed-scale 0.85
```

Output: `lane_left.csv`, `lane_right.csv`, `lane_optimal.csv` (the original line, copied verbatim). Re-run with different `--offset` to tune separation. Rebuild after changing CSV contents because the files are installed into the share directory at build time.

## Lane-switching parameters

In `pure_pursuit/config/zach_params/sim_params.yaml`:

| Parameter | Meaning |
|---|---|
| `opponent_topic` | Odometry topic for the opponent. Empty string disables switching. |
| `lane_occupied_dist` | Lane is blocked if any forward waypoint is within this radius (m) of the opponent. |
| `lane_lookahead_idx` | Number of forward waypoints scanned per lane when checking occupancy. |
| `min_switch_interval_sec` | Cooldown between consecutive lane switches. |

The lane priority order is the order of `lane_csv_paths` in the launch file (last entry = highest priority). The decision rule is: try lanes from highest to lowest priority and pick the first one that is not blocked. If all lanes are blocked, hold the current lane.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sub | `/ego_racecar/odom` (sim) or `/pf/viz/inferred_pose` (real) | Odometry / PoseStamped | ego pose |
| sub | `/opp_racecar/odom` (configurable) | Odometry | opponent pose |
| sub | `rrt_path` | Path | optional RRT override |
| pub | `/drive` | AckermannDriveStamped | control output |
| pub | `/waypoint_marker`, `/waypoint_path`, `/waypoint_path_colored`, `/waypoint_path_points` | Marker / Path | visualization (Foxglove / RViz) |
| pub | `/driven_path` | Path | actual driven trajectory |
| svc | `/pure_pursuit/update_raceline` | UpdateRaceline | hot-swap active lane |

## License

MIT — see [LICENSE](LICENSE).
