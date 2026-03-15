# dicerox_mapping

2D SLAM for Dicerox rescue robot using slam_toolbox (online async). ZED2 supplies odometry via visual-inertial odometry, Slamtec A1 supplies 2D laser scans.

## Pipeline

```
ZED2 (VIO @ 30 Hz)
  └─ publishes TF:  odom → zed_camera_link
  └─ publishes:     /zed/zed_node/odom  (nav_msgs/Odometry)

Slamtec A1 (@ ~6.5 Hz)
  └─ publishes:     /scan  (sensor_msgs/LaserScan, frame_id: laser)

Static TF (launch file)
  └─ zed_camera_link → laser  (fixed, measured offset)

slam_toolbox (async_slam_toolbox_node)
  ├─ subscribes:    /scan
  ├─ TF lookup:     odom → zed_camera_link → laser  (at each scan timestamp)
  ├─ publishes TF:  map → odom
  └─ publishes:     /map  (nav_msgs/OccupancyGrid, every 2s)
```

When a scan arrives, slam_toolbox calls `tf2::Buffer::lookupTransform("odom", "laser", scan.header.stamp)`. tf2 interpolates between the two nearest odom transforms (30 Hz buffer) to get an exact pose at the scan timestamp. No topic synchronization is needed between the two sensors — the frequency mismatch is handled entirely by the TF buffer.

The async node processes scans in a separate thread, so slow scan matching does not block the ROS executor or drop incoming scans.

## TF tree

```
map
 └── odom                        (published by slam_toolbox @ 50 Hz)
      └── zed_camera_link        (published by ZED driver @ 30 Hz)
           ├── laser             (static, from launch file)
           └── zed_camera_center
                ├── zed_left_camera_frame
                ├── zed_right_camera_frame
                └── ...
```

`base_frame` for slam_toolbox is `zed_camera_link`. The map is built in the `map` frame.

## Physical setup

Lidar is mounted **15 cm to the right** of the ZED camera, same heading and height.

```
zed_camera_link origin
        │
        ├── x: 0.0   (no forward offset)
        ├── y: -0.15  (right in ROS convention: negative Y)
        └── z: 0.0   (same height)
```

Static TF is published as `zed_camera_link → laser` with these values. Change `lidar_y` (and others) at launch if the mount changes.

## Hardware

| | ZED2 | Slamtec A1 |
|---|---|---|
| Topic | `/zed/zed_node/odom` | `/scan` |
| Rate | 30 Hz | ~6.5 Hz |
| Frame | `zed_camera_link` | `laser` |
| Max range | — | 12 m |
| Driver | `zed_wrapper` | `sllidar_ros2` |

## SLAM parameters

Key values in `config/slam_toolbox_params.yaml`:

| Parameter | Value | Notes |
|---|---|---|
| `resolution` | 0.05 m/cell | 5 cm cells |
| `max_laser_range` | 12.0 m | matches A1 spec |
| `map_update_interval` | 2.0 s | how often `/map` is published |
| `transform_publish_period` | 0.02 s | `map→odom` TF rate (50 Hz) |
| `minimum_travel_distance` | 0.1 m | new scan used after 10 cm motion |
| `minimum_travel_heading` | 0.1 rad | or ~5.7° rotation |
| `transform_timeout` | 0.5 s | max wait for TF before dropping scan |
| `tf_buffer_duration` | 30.0 s | TF history kept in buffer |
| `scan_buffer_size` | 10 | scans kept for loop closure candidates |

Solver is Ceres with `SPARSE_NORMAL_CHOLESKY` + `SCHUR_JACOBI` preconditioner. Loop closure runs whenever scan matching response exceeds `link_match_minimum_response_fine: 0.1`.

To resume from a saved map (localization or continued mapping), uncomment in the YAML:
```yaml
map_file_name: /home/robotec/maps/dicerox_map
map_start_pose: [0.0, 0.0, 0.0]
```
and set `mode: localization`.

## Running

**Jetson — one terminal per command, all must have `setup_jetson.bash` sourced:**
```bash
source ~/setup_jetson.bash

./launch_zed.sh
ros2 launch sllidar_ros2 sllidar_a1_launch.py
ros2 launch dicerox_mapping mapping_robot.launch.py
```

**Laptop — visualization:**
```bash
source setup_laptop.bash
ros2 launch dicerox_mapping mapping_viz.launch.py
```

`mapping_robot.launch.py` runs slam_toolbox without RViz. `mapping.launch.py` runs everything on one machine (useful for bag replay or testing on the laptop).

## Saving the map

While `mapping_robot.launch.py` is running, call the serialize service:
```bash
ros2 launch dicerox_mapping save_map.launch.py
# custom path:
ros2 launch dicerox_mapping save_map.launch.py map_name:=/home/robotec/maps/arena_01
```

Produces `<name>.posegraph` (graph + metadata) and `<name>.data` (raw scan data). Both files are needed to reload the map.

## Bag recording and replay

Record raw data during a run:
```bash
ros2 launch dicerox_mapping record_bag.launch.py
```
Records `/scan`, `/zed/zed_node/odom`, `/tf`, `/tf_static`. Bags go to `/home/robotec/bags/run_YYYYMMDD_HHMMSS/`.

Remap offline:
```bash
ros2 bag play <bag_dir> --clock
ros2 launch dicerox_mapping mapping.launch.py use_sim_time:=true
```

`--clock` publishes `/clock` from the bag timestamps. `use_sim_time:=true` makes all nodes consume it instead of wall clock.

## Build

```bash
cd ~/Projects/Robotics_2026/mapping
colcon build --packages-select dicerox_mapping
source install/setup.bash
```
