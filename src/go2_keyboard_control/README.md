# go2_keyboard_control

Keyboard teleoperation for Unitree Go2 ROS2 sport mode.

The node publishes `unitree_api/msg/Request` messages to `/api/sport/request`.
Move commands use sport API `1008` with JSON parameters:

```json
{"x": vx, "y": vy, "z": vyaw}
```

## Build

From the workspace root:

```bash
colcon build --packages-select go2_keyboard_control
source install/setup.bash
```

On Windows PowerShell after building:

```powershell
.\install\setup.ps1
```

## Run

```bash
ros2 run go2_keyboard_control keyboard_control
```

Keys:

| Key | Action |
| --- | --- |
| `w` / `x` | forward / backward |
| `a` / `d` | left / right |
| `q` / `e` | turn left / turn right |
| `space` or `s` | stop |
| `+` / `-` | increase / decrease linear speed |
| `[` / `]` | decrease / increase yaw speed |
| `b` | balance stand |
| `u` | stand up |
| `c` | stand down |
| `h` | print help |
| `Ctrl+C` | stop and exit |

Optional parameters:

```bash
ros2 run go2_keyboard_control keyboard_control --ros-args \
  -p linear_speed:=0.35 \
  -p yaw_speed:=0.8 \
  -p publish_rate:=10.0 \
  -p command_timeout:=0.5
```
