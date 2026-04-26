# pure-pursuit-muti-raceline

Multi-raceline pure pursuit controller for the [F1TENTH](https://f1tenth.org/) platform. Loads several precomputed raceline trajectories in parallel and switches between them at runtime when the active line is blocked by an opponent vehicle. Falls back to adaptive cruise when no clear lane is available.

Built on top of a single-line C++ pure pursuit node and the `f1tenth_gym_ros` simulator. Companion packages live in the same workspace:

- **`opp_dummy`** — drives the sim's opp car along a fixed lane so you can test overtaking without manually piloting it (Foxglove drag).
- **`opp_predictor`** — LiDAR-based opponent detector for real-car deployment; outputs `/opp_predict/state` so this controller works without sim ground truth.

---

## Features

- **Multi-lane runtime selection**: load N raceline CSVs and switch among them based on opponent occupancy. Last lane in the list has the highest priority.
- **Reactive lane switching**: every control tick, scan a forward window of waypoints on each lane; mark a lane as blocked if any waypoint is within `lane_occupied_dist` of the opponent. Pick the highest-priority unblocked lane, with a configurable cooldown to prevent flapping.
- **Adaptive cruise (follow-mode)**: when the active lane has the opp inside the forward window and we can't dodge, cap speed by ego-to-opp distance — linearly interpolated between `follow_brake_dist`→`follow_min_speed` and `follow_release_dist`→`follow_max_speed`. Prevents rear-ending the opp when stuck.
- **Sim or real opp source**: `opponent_msg_type` param toggles between `Odometry` (sim ground truth) and `PoseStamped` (real-car detector).
- **Backward compatibility**: set `opponent_topic: ""` and the node behaves as a single-lane pure pursuit identical to the upstream baseline.
- **Lane generator script**: `generate_offset_lanes.py` builds parallel left/right lanes from any 4-column raceline CSV by offsetting along the path normal.
- **Hot-swap waypoints**: `/pure_pursuit/update_raceline` service for replacing the active lane's waypoints at runtime.
- **RRT path override**: subscribes to `rrt_path` and follows it when fresh, otherwise falls back to raceline tracking.

---

## Repository layout

```
pure_pursuit/
├── src/pure_pursuit_node.cpp           # main C++ node
├── src/pure_pursuit_boost.cpp          # alternative implementation
├── scripts/generate_offset_lanes.py    # CSV lane generator
├── scripts/waypoint_logger.py          # record waypoints from odom
├── scripts/smooth_waypoints.py         # spline-smooth a recorded path
├── waypoints/lanes/                    # generated multi-lane CSVs
├── waypoints/race/                     # source single-line racelines
├── config/zach_params/*.yaml           # ROS parameter files (sim & real)
└── launch/zach/
    ├── sim_launch.py                   # ego only (sim mode)
    ├── sim_two_cars_launch.py          # ego + opp_dummy (full sim test)
    └── real_launch.py                  # ego on real car
```

CSV format is 4 columns: `x, y, yaw, speed_ratio` where `speed_ratio ∈ [0, 1]` is mapped to `[slow_speed, fast_speed]` at runtime.

---

## Build

```bash
cd ~/sim_ws
colcon build --packages-select pure_pursuit_multi opp_dummy opp_predictor
source install/setup.bash
```

Always build from the workspace root, never from inside `src/`. Building from inside `src/` creates nested install dirs that the outer launch never sees.

---

## Run in simulation

Make sure `f1tenth_gym_ros` has `num_agent: 2` set in [its sim.yaml](../f1tenth_gym_ros/config/sim.yaml) (and was built after that change).

### Full test (ego + autonomous opp)

```bash
# terminal 1
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# terminal 2 — starts both pure_pursuit_multi (ego) and opp_dummy (opp)
ros2 launch pure_pursuit_multi sim_two_cars_launch.py
```

Expected: opp drives `lane_optimal` at constant speed, ego (faster) approaches → switches to a free lane → overtakes → switches back to optimal. If all lanes are blocked, ego decelerates via follow-mode and tails the opp instead of crashing.

Console will print things like:
```
lane switch: lane_optimal -> lane_left  (blocked: yes min_d=0.21, switching to min_d=0.84)
follow-mode: gap=1.45m  speed 9.00 -> 3.27 m/s
```

### Ego only (drag opp manually in Foxglove)

```bash
ros2 launch pure_pursuit_multi sim_launch.py
```

Use Foxglove's **2D Pose Goal** tool publishing to `/goal_pose` to teleport opp.

---

## Tuning parameters

In `pure_pursuit/config/zach_params/sim_params.yaml` (or `real_params.yaml`):

### Lane switching
| Parameter | Meaning |
|---|---|
| `opponent_topic` | Topic for the opp pose. Empty string disables switching. |
| `opponent_msg_type` | `"Odometry"` (sim) or `"PoseStamped"` (real, from `opp_predictor`). |
| `lane_occupied_dist` | Lane is blocked if any forward waypoint is within this radius (m) of the opp. |
| `lane_lookahead_idx` | Number of forward waypoints scanned per lane when checking occupancy. |
| `min_switch_interval_sec` | Cooldown between consecutive lane switches (seconds). |

### Follow-mode (adaptive cruise)
| Parameter | Meaning |
|---|---|
| `follow_min_speed` | Speed cap when ego is bumper-to-bumper with opp. |
| `follow_max_speed` | Speed cap at `follow_release_dist`. Set roughly to opp's cruise speed for clean tailing. |
| `follow_brake_dist` | Distance ≤ this → cap to `follow_min_speed`. |
| `follow_release_dist` | Distance ≥ this → no cap (full lane speed). |

Lane priority order is the order of `lane_csv_paths` in the launch file (last entry = highest priority). The decision rule: try lanes from highest to lowest priority and pick the first one that is not blocked. If all are blocked, hold the current lane and let follow-mode handle speed.

---

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sub | `/ego_racecar/odom` (sim) or `/pf/viz/inferred_pose` (real) | Odometry / PoseStamped | ego pose |
| sub | `/opp_racecar/odom` or `/opp_predict/state` | Odometry / PoseStamped | opp pose, source depends on `opponent_msg_type` |
| sub | `rrt_path` | Path | optional RRT override |
| pub | `/drive` | AckermannDriveStamped | control output |
| pub | `/waypoint_marker`, `/waypoint_path`, `/waypoint_path_colored`, `/waypoint_path_points` | Marker / Path | visualization |
| pub | `/driven_path` | Path | actual driven trajectory |
| svc | `/pure_pursuit/update_raceline` | UpdateRaceline | hot-swap active lane |

---

## Next steps — real-car deployment with `opp_predictor`

> **Sim is already wired up; this is what to do next time you want to bring the same controller onto the real car.** The companion package `opp_predictor` does LiDAR-based opponent detection. It needs one-time map preparation, then a sim sanity check, then the real-car launch.

### Step 1 — generate track bound files (one-time per map)

The detector filters LiDAR returns to keep only points lying on the track (between inner and outer wall polygons). Generate the polygons for `racetrack_test`:

```bash
cd ~/sim_ws/src/opp_predictor
python3 scripts/lane_generator.py --map racetrack_test
```

Two OpenCV preview windows pop up (input/output, then bounds). Press any key in each window to continue, `q` to abort. Outputs land in `csv/racetrack_test/{inner,outer}_bound.{npy,csv}` and `track.{npy,csv}`.

If `cv2.ximgproc.thinning` errors out:
```bash
pip install opencv-contrib-python
```

After generating, rebuild so the new files reach `share/`:
```bash
cd ~/sim_ws && colcon build --packages-select opp_predictor
```

For other maps, copy `<map>.pgm` + `<map>.yaml` into `~/sim_ws/src/opp_predictor/maps/` first, then run with `--map <map>`.

### Step 2 — sim sanity check (verify the detector works before going to hardware)

Confirm the predictor + multi-lane controller close the loop **without hardware risk**. Edit [pure_pursuit_multi sim_params.yaml](pure_pursuit/config/zach_params/sim_params.yaml) temporarily:

```yaml
opponent_topic: "/opp_predict/state"
opponent_msg_type: "PoseStamped"
```

Then:

```bash
# terminal 1
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
# terminal 2: ego + opp_dummy as before
ros2 launch pure_pursuit_multi sim_two_cars_launch.py
# terminal 3: detector replacing /opp_racecar/odom ground truth
ros2 launch opp_predictor opp_predictor_launch.py real_test:=false
```

In Foxglove, subscribe to `/opp_predict/viz/pose` (Marker) — should see a pink sphere tracking the opp dummy. If lane switching still works correctly, the detector is good. **Revert sim_params.yaml back to `Odometry` + `/opp_racecar/odom` afterward** so daily sim debugging stays simple.

### Step 3 — real-car launch sequence

Three terminals, three commands, in this order:

```bash
# terminal 1: your existing real-car bringup (LiDAR driver, motor driver, particle_filter)
# (whatever you usually run on the car — must publish /scan and /pf/viz/inferred_pose)

# terminal 2: opponent detector
ros2 launch opp_predictor opp_predictor_launch.py
# (real_test defaults to true; uses /pf/viz/inferred_pose for ego pose)

# terminal 3: ego controller (real_launch already configured for PoseStamped opp input)
ros2 launch pure_pursuit_multi real_launch.py
```

### Step 4 — field tuning (these almost always need adjustment on hardware)

Edit `~/sim_ws/src/opp_predictor/config/opp_predictor_params.yaml`:

| Param | Symptom → fix |
|---|---|
| `cluster_size_tol: 15` | Detector misses opp at distance → lower (5-10). False positives from noise → raise (20-30). |
| `grid_xmin/xmax/ymin/ymax` | Walls register as opps → narrow the box so walls fall outside the search grid. |
| `cluster_dist_tol: 0.27` | Single car splits into two clusters → raise to ~0.4. |
| `disable_track_filter: false` | Set `true` if `lane_generator.py` won't run; relies on `grid_*` bounds alone to suppress walls. |

Then in `~/sim_ws/src/puresuit_muti-reacetrack/pure_pursuit/config/zach_params/real_params.yaml`:

| Param | Symptom → fix |
|---|---|
| `lane_occupied_dist: 0.5` | Real LiDAR is noisier; raise to 0.6-0.7 if false-positive switches happen. |
| `follow_brake_dist: 0.7` / `follow_release_dist: 3.0` | Too cautious on real car → lower brake_dist to ~0.5; too aggressive → raise to ~0.9. |

Each YAML change needs a `colcon build --packages-select <pkg>` because the configs are installed to `share/` at build time.

---

## License

MIT — see [LICENSE](LICENSE).
