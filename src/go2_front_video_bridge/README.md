# go2_front_video_bridge

Bridge Unitree Go2's built-in front camera stream to standard ROS image topics.

The Go2 topic:

```text
/frontvideostream
```

uses:

```text
unitree_go/msg/Go2FrontVideoData
```

with fields:

```text
uint64 time_frame
uint8[] video720p
uint8[] video360p
uint8[] video180p
```

This package republishes one selected stream as:

```text
/go2/front_camera/image_compressed   sensor_msgs/CompressedImage
/go2/front_camera/image_raw          sensor_msgs/Image
/go2/front_camera/camera_info        sensor_msgs/CameraInfo
```

`image_raw` is only published when the bytes are JPEG frames and OpenCV can
decode them. If the Go2 firmware publishes H264/H265 bytes, this bridge will
still publish `CompressedImage`, but RViz2 will need a video decoder bridge.

## Build

```bash
cd ~/unitree_go2_ws/unitree_go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select go2_front_video_bridge
source install/setup.bash
```

If raw image decoding is needed:

```bash
sudo apt install -y python3-opencv
```

## Run

```bash
ros2 launch go2_front_video_bridge front_video_bridge.launch.py
```

Choose a smaller stream:

```bash
ros2 run go2_front_video_bridge front_video_bridge_node --ros-args \
  -p stream_field:=video360p
```

## View

First check the bridge output:

```bash
ros2 topic list | grep front_camera
ros2 topic hz /go2/front_camera/image_raw
ros2 topic hz /go2/front_camera/image_compressed
```

View raw decoded image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/go2/front_camera/image_raw
```

RViz2:

```bash
rviz2
```

Add an `Image` display and select:

```text
/go2/front_camera/image_raw
```

If `image_raw` has no data but `image_compressed` does, the Go2 stream is likely
not JPEG. In that case, inspect one frame's first bytes:

```bash
ros2 topic echo /frontvideostream --once
```

or use the bridge log warning to decide whether an H264/H265 decoder node is
needed.
