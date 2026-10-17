# go2_local_costmap

Local mapping and target tracking helpers for Unitree Go2 person following.

## What This Package Does

`local_costmap_node` converts a live 3D point cloud into a robot-centric 2D
`nav_msgs/OccupancyGrid` local map. The grid is the right first map type for
person following and obstacle avoidance because the robot only needs nearby
obstacles, not a saved global point cloud map.

`target_tracker_node` smooths a detected person position and publishes a stable
follow target. The detector can be YOLO + D435i depth, a clicked point, or any
node that publishes `geometry_msgs/PointStamped`.

## Current Go2 Topic Choice

Your topic list already contains:

```text
/utlidar/cloud_base
/utlidar/cloud
/utlidar/grid_map
/utlidar/height_map
/utlidar/voxel_map
```

For first tests, this package defaults to:

```text
/utlidar/cloud_base
```

because it is likely already expressed close to the robot base frame. For D435i,
change `cloud_topic` in `config/local_costmap.yaml` to the RealSense point cloud,
usually something like:

```text
/camera/camera/depth/color/points
```

## Outputs

```text
/go2/local_costmap          nav_msgs/OccupancyGrid
/go2/obstacle_summary       std_msgs/String JSON
/go2/obstacle_markers       visualization_msgs/MarkerArray
/go2/follow_target          geometry_msgs/PointStamped
/go2/follow_target_state    std_msgs/String JSON
```

`/go2/obstacle_summary` contains:

```json
{
  "nearest_front_m": 0.72,
  "nearest_left_m": 1.21,
  "nearest_right_m": null,
  "front_blocked": false,
  "front_slow": true,
  "suggested_vx_scale": 0.49,
  "suggested_steer_y": 1.0
}
```

Follow controller priority should be:

```text
hard stop > obstacle avoidance > person following
```

## Build

```bash
cd ~/unitree_go2_ws/unitree_go2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_local_costmap
source install/setup.bash
```

## Run

```bash
ros2 launch go2_local_costmap local_costmap.launch.py
```

To run with D435i point cloud:

```bash
ros2 run go2_local_costmap local_costmap_node --ros-args \
  -p cloud_topic:=/camera/camera/depth/color/points \
  -p target_frame:=base_link
```

## Visualize

```bash
rviz2
```

Add an `OccupancyGrid` display and select:

```text
/go2/local_costmap
```

Set the fixed frame to `base_link`.

Add a `MarkerArray` display and select:

```text
/go2/obstacle_markers
```

This shows the front obstacle point, slow/stop thresholds, steering hint, and a
small status text.

## Target Tracking Input

Publish a detected person point:

```bash
ros2 topic pub /go2/detected_person_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 1.5, y: 0.2, z: 0.5}}"
```

The tracker publishes a smoothed target on:

```text
/go2/follow_target
```

In the final system, a person detector should:

```text
RGB image + depth image
  -> detect person bbox
  -> take median depth near bbox center
  -> project to 3D
  -> transform to base_link
  -> publish /go2/detected_person_point
```

## Tuning Notes

- If the map is full of floor points, raise `min_z`.
- If obstacles disappear, lower `min_z` or raise `max_z`.
- If the robot stops too early, reduce `inflation_radius` or `slow_down_distance`.
- If the point cloud frame is not `base_link`, publish the correct TF from
  `base_link` to the camera or lidar frame and set `allow_untransformed_cloud`
  to `false` once TF is reliable.
