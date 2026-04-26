# opp_predictor

LiDAR-based opponent detector for f1tenth real-car racing. Subscribes to `/scan` + ego pose, publishes opponent position as `PoseStamped` on `/opp_predict/state` for consumption by `pure_pursuit_multi`.

## Workflow

### 1. Generate track bound files (one-time, per map)

The detector filters detected obstacles to keep only points lying on the track (between inner and outer wall polygons). Generate the polygons from the map image:

```bash
cd ~/sim_ws/src/puresuit_muti-reacetrack/opp_predictor
python3 scripts/lane_generator.py --map racetrack_test
```

Two preview windows pop up (input/output, then bounds). Press any key to continue, `q` to abort. Outputs:

```
csv/racetrack_test/
├── inner_bound.npy / .csv
├── outer_bound.npy / .csv
├── track.npy / .csv
└── lane_*.csv          # racing lines (not used by detector)
```

If `cv2.ximgproc.thinning` errors out, install `opencv-contrib-python`:

```bash
pip install opencv-contrib-python
```

After generating, rebuild so the files get installed to share/:

```bash
cd ~/sim_ws && colcon build --packages-select opp_predictor
```

### 2. Run the detector

```bash
# Real car (uses /pf/viz/inferred_pose for ego, /scan for sensor):
ros2 launch opp_predictor opp_predictor_launch.py

# Sim test (uses /ego_racecar/odom for ego):
ros2 launch opp_predictor opp_predictor_launch.py real_test:=false

# Different map:
ros2 launch opp_predictor opp_predictor_launch.py map_name:=my_other_map
```

### 3. Verify

```bash
ros2 topic echo /opp_predict/state
```

Should print `(inf, inf)` when no opponent is detected and a real `(x, y)` when one is. Visualize `/opp_predict/viz/pose` (Marker) and `/opp_predict/viz/bbox` (MarkerArray) in RViz/Foxglove.

## Tuning (per LiDAR + per map)

Most-touched params in `config/opp_predictor_params.yaml`:

| Param | Default | Why you'd change it |
|---|---|---|
| `grid_xmin/xmax/ymin/ymax` | `[0,5]×[-2.5,2.5]` | Restrict detection zone — narrow it to suppress wall false-positives |
| `grid_safe_dist` | 0.10 m | Lower if LiDAR is precise; raise if noisy |
| `cluster_dist_tol` | 0.27 m | Approx car width; tune to your opponent's footprint |
| `cluster_size_tol` | 15 | Min points to count as opp; raise to reject sparse noise, lower to detect smaller targets |
| `disable_track_filter` | false | Set true to skip bound files (fallback if `lane_generator.py` won't run) |

## Bypass (no bound files)

If you can't get `lane_generator.py` to run, set `disable_track_filter: true` in the params YAML. The detector will accept any clustered point inside the grid; tune `grid_*` carefully so walls fall outside the grid.
