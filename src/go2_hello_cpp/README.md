# go2_hello_cpp

ROS2 C++ node for the Unitree Go2 hello command.

It publishes `unitree_api/msg/Request` to `/api/sport/request` with:

```cpp
request.header.identity.api_id = 1016;
```

## Build

```bash
colcon build --packages-select go2_hello_cpp
source install/setup.bash
```

Windows PowerShell:

```powershell
colcon build --packages-select go2_hello_cpp
.\install\setup.ps1
```

## Run

Publish once and exit:

```bash
ros2 run go2_hello_cpp hello_node
```

Publish repeatedly:

```bash
ros2 run go2_hello_cpp hello_node --ros-args -p repeat:=true -p period_ms:=1000
```
