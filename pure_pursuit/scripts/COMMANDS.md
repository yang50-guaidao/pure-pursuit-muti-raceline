# Raceline Optimizer Commands

All commands run from `pure_pursuit/` directory:
```bash
cd ~/ros2_ws/roboracer_ws/src/lab5/lab-5-slam-and-pure-pursuit-team5/pure_pursuit
```

## Speed column = ratio (0-1), NOT absolute speed
The CSV stores a speed ratio: `0.0 = slow_speed`, `1.0 = fast_speed`.
Actual speed is computed at runtime by the C++ node using `slow_speed` and `fast_speed` params.
This means you can tune speed in your YAML / `ros2 param set` without regenerating waypoints.

---

## 1. show — Visualize any waypoint file
Shows trajectory on map + curvature profile + speed ratio profile.
```bash
python3 scripts/raceline_optimizer.py show waypoints/smoothed/real/real_waypoints_3_smoothed_100_2.csv ../maps/lev_hall_lap1.yaml
```

## 2. profile — Add physics-based speed ratios
Computes where to brake/accelerate using forward-backward integration.
Works on ANY waypoints (recorded, smoothed, optimized).
```bash
python3 scripts/raceline_optimizer.py profile waypoints/smoothed/real/real_waypoints_3_smoothed_100_2.csv \
  -o waypoints/race/levine_profiled.csv --mu 0.7 --show
```
- `--mu` friction coefficient (0.5-0.8, default 0.7). Lower = more conservative braking.
- `--amax` max acceleration m/s^2 (default 5.0)
- `--abrake` max braking m/s^2 (default 8.0)

## 3. extract — Pull centerline from SLAM map automatically
Uses distance transform + erosion to find the center of hallways/track.
No pre-existing waypoints needed — just the SLAM map.
```bash
python3 scripts/raceline_optimizer.py extract ../maps/lev_hall_lap1.yaml \
  -o waypoints/race/centerline.csv -n 100 --show
```
- `-n` number of waypoints (default 100)
- `--margin` wall margin in meters (default 0.20)

## 4. optimize — Minimum curvature raceline from centerline + map
Shifts centerline points laterally to minimize curvature (racing line).
Inflates walls by half car width so the optimizer respects car body.
Automatically profiles speed ratios after optimizing.
```bash
python3 scripts/raceline_optimizer.py optimize waypoints/race/centerline.csv ../maps/lev_hall_lap1.yaml \
  -o waypoints/race/mincurv.csv --margin 0.20 --car-width 0.30 --show
```
- `--car-width` car body width for wall inflation (default 0.30m)
- `--margin` additional safety margin beyond inflation (default 0.20m)
- `--mu` friction coefficient (default 0.7)
- Shows 3 panels: inflation zones (red) + raceline, speed-colored map, speed ratio profile

## 5. edit — Interactive trajectory editor
Drag waypoints on the map to shape your own racing line.
```bash
python3 scripts/raceline_optimizer.py edit waypoints/race/mincurv.csv \
  ../maps/lev_hall_lap1.yaml # Optionally: -o waypoints/race/custom.csv
```
Controls:
- **drag** = move nearest waypoint
- **shift+click** = insert new waypoint
- **right-click** = delete waypoint
- **p** = re-run velocity profiling
- **o** = re-run min-curvature optimization (needs map)
- **s** = save to output file
- **q** = quit (warns if unsaved)

---

## Typical workflows

**Path A — Record + Profile** (proven, what worked in Levine):
1. Drive track manually with `waypoint_logger.py`
2. Smooth: `python3 scripts/smooth_waypoints.py raw.csv smoothed.csv 100 5.0`
3. Profile: `raceline_optimizer.py profile smoothed.csv -o race.csv --show`
4. Fine-tune: `raceline_optimizer.py edit race.csv map.yaml -o final.csv`

**Path B — Auto from SLAM map**:
1. SLAM the track -> map.yaml/pgm
2. `raceline_optimizer.py extract map.yaml -o center.csv --show`
3. `raceline_optimizer.py optimize center.csv map.yaml -o race.csv --show`
4. Fine-tune: `raceline_optimizer.py edit race.csv map.yaml -o final.csv`

## C++ node setup
Set `use_waypoint_speed: true` in your launch file params, and point `csv_path` to the profiled CSV.
Tune `fast_speed` and `slow_speed` at runtime — waypoint ratios scale between them automatically.


### Record a rosbag - make sure we don't need more topics here!
```bash
ros2 bag record /map /pf/viz/inferred_pose /drive /waypoint_path /waypoint_marker /driven_path -o race_run_1
```