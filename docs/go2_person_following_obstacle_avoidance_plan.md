# Unitree Go2 外接相机人跟随与自动避障实施方案

## 1. 目标与结论

目标是在 Unitree Go2 上实现：

- 外接摄像头识别人并稳定跟随；
- 跟随过程中自动避障；
- 在室内或半结构化环境中具备可恢复、可急停、可调参的安全运行能力；
- 基于当前 ROS 2 工作区逐步落地，而不是一上来重写底层运动控制。

推荐路线：

```text
外接 RGB-D 相机
  -> 人体检测 / 跟踪
  -> 目标相对位置估计
  -> 局部代价地图 costmap
  -> 局部规划 / 避障速度控制
  -> Go2 Sport API /api/sport/request
```

第一阶段不要做全局建图导航，也不要直接上低层关节控制。当前工作区已经有 `go2_keyboard_control`，可以发布 `/api/sport/request` 调 Go2 sport mode 的速度接口。人跟随和避障本质上只需要输出 `vx, vy, yaw_rate`，因此先复用 sport mode 是最快、风险最低、最容易实现的方案。

## 2. 硬件选择

### 2.1 外接摄像头

推荐使用 RGB-D 深度相机，而不是普通 USB 摄像头。

优先选择：

- Intel RealSense D435i / D455；
- Orbbec Gemini 2 / Astra 系列；
- 其他能稳定输出 `RGB image + depth image + camera_info + pointcloud` 的 ROS 2 相机。

原因：

- 人跟随需要知道人相对机器人的距离；
- 避障需要障碍物深度；
- 单目相机做人跟随可以实现，但距离估计和避障不稳定，工程风险高。

### 2.2 安装位置

建议相机安装在 Go2 机身前上方：

```text
高度：约 0.35 m 到 0.55 m
朝向：水平或轻微下俯 5 到 15 度
视场：覆盖机器人前方 0.3 m 到 5 m
固定：刚性支架，不能随走动明显晃动
```

必须标定相机坐标系到机器人机体坐标系：

```text
camera_link -> base_link
```

这个 TF 是整个系统能否稳定避障的关键。

## 3. 地图选择

### 3.1 推荐使用什么地图

人跟随场景首选局部代价地图，不建议一开始做全局点云地图。

推荐：

```text
2D local costmap + inflation layer
```

局部地图只维护机器人周围几米范围内的障碍物，例如：

```yaml
width: 6.0
height: 6.0
resolution: 0.05
robot_radius: 0.35
inflation_radius: 0.35
obstacle_range: 4.0
raytrace_range: 4.5
```

原因：

- 人跟随目标是动态的，人会动，障碍物也可能动；
- Go2 不一定需要提前知道全局地图；
- 局部避障比全局 SLAM 更容易调通；
- 室内跟随时，实时障碍物比历史地图更重要。

### 3.2 什么时候需要全局地图

以下情况才需要全局地图：

- 要从 A 点自动去 B 点；
- 要记住房间、走廊、门的位置；
- 要绕远路找路径；
- 要在人丢失后回到某个固定位置；
- 要做多楼层或大范围巡检。

如果只是“看到人并跟随，同时别撞东西”，不需要先建全局地图。

### 3.3 地图类型选择

推荐顺序：

```text
阶段 1：无全局地图，只用局部 costmap
阶段 2：2D 栅格地图 OccupancyGrid，用于室内导航
阶段 3：3D OctoMap / Voxblox / TSDF，用于复杂立体障碍
```

对 Go2 人跟随来说，阶段 1 已经可以跑通核心功能。

## 4. 点云是否需要转换

结论：需要转换，但不一定需要保存为点云地图。

相机通常输出：

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/depth/color/points   # PointCloud2
/camera/camera_info
```

避障模块需要把深度图或点云转换成局部代价地图：

```text
PointCloud2 / DepthImage
  -> 过滤地面、过高点、无效点
  -> 投影到 base_link 坐标系
  -> 投影到 2D costmap
  -> 膨胀障碍物
  -> 局部规划器输出安全速度
```

因此“点云地图需要转换吗”的答案是：

- 如果只做人跟随避障：不需要把点云转换成全局点云地图；
- 需要把实时点云转换成局部 costmap 的障碍层；
- 如果以后做 3D 建图或复杂地形通行，才需要把点云融合成 OctoMap、Voxblox、TSDF 或 PCD 地图。

## 5. 软件架构

建议新增以下 ROS 2 包：

```text
src/
  go2_keyboard_control/              # 已有，保留用于手动测试
  go2_person_detector/               # 人体检测与跟踪
  go2_target_tracker/                # 目标选择与相对位置估计
  go2_local_costmap/                 # 点云转局部障碍地图
  go2_follow_controller/             # 跟随控制 + 避障速度融合
  go2_safety_monitor/                # 急停、超时、限速、安全状态
  go2_bringup/                       # launch 和参数
```

节点关系：

```text
RGB-D Camera
  publishes:
    /camera/color/image_raw
    /camera/depth/image_rect_raw
    /camera/depth/color/points

go2_person_detector
  subscribes:
    /camera/color/image_raw
  publishes:
    /person_detections

go2_target_tracker
  subscribes:
    /person_detections
    /camera/depth/image_rect_raw
    /tf
  publishes:
    /follow_target

go2_local_costmap
  subscribes:
    /camera/depth/color/points
    /tf
  publishes:
    /local_costmap
    /obstacle_summary

go2_follow_controller
  subscribes:
    /follow_target
    /local_costmap
    /obstacle_summary
    /safety_state
  publishes:
    /api/sport/request

go2_safety_monitor
  subscribes:
    /follow_target
    /camera diagnostics
    /go2 state topics
  publishes:
    /safety_state
```

## 6. 人体检测与目标跟踪

### 6.1 检测模型

推荐使用 YOLO 系列人体检测模型：

```text
输入：RGB image
输出：person bounding boxes
```

第一版只检测 `person` 类别即可。

如果算力有限：

- 使用 YOLOv8n / YOLOv8s；
- 分辨率从 640x480 开始；
- 推理频率 10 到 20 Hz；
- 跟随控制频率 20 到 50 Hz。

### 6.2 目标选择

多个人同时出现时，按以下优先级选目标：

```text
1. 距离上一帧目标最近的人
2. 位于图像中心附近的人
3. 面积最大的人
4. 深度有效的人
```

输出目标消息建议包含：

```text
target_visible: bool
target_x: float   # base_link 下前向距离，单位 m
target_y: float   # base_link 下左右偏移，单位 m
target_z: float
confidence: float
last_seen_time
```

### 6.3 距离估计

不要只用检测框大小估计距离。应该用检测框中心区域的深度：

```text
person bbox
  -> 取中心 20% 到 40% 区域深度
  -> 去除 0、NaN、过远值
  -> 取中位数
  -> 反投影成相机坐标
  -> TF 转到 base_link
```

这样能显著减少深度噪声。

## 7. 跟随控制策略

### 7.1 跟随目标

建议保持：

```text
期望距离 follow_distance: 1.2 m
允许距离 deadband: 0.15 m
最大前进速度 max_vx: 0.5 m/s
最大横向速度 max_vy: 0.25 m/s
最大转向速度 max_yaw: 0.8 rad/s
```

第一版控制器：

```text
distance_error = target_x - follow_distance
lateral_error  = target_y

vx_cmd  = kp_x   * distance_error
vy_cmd  = kp_y   * lateral_error
yaw_cmd = kp_yaw * atan2(target_y, target_x)
```

再做限幅和平滑：

```text
vx_cmd  = clamp(vx_cmd,  -0.2, 0.5)
vy_cmd  = clamp(vy_cmd,  -0.25, 0.25)
yaw_cmd = clamp(yaw_cmd, -0.8, 0.8)
```

### 7.2 人丢失策略

```text
丢失 < 0.5 s：保持上一目标，速度逐渐衰减
丢失 0.5 到 2.0 s：原地小角速度搜索
丢失 > 2.0 s：停止
```

不要在人丢失后继续向前走。

## 8. 避障策略

### 8.1 第一版避障

第一版不需要复杂全局规划。使用扇区避障即可：

```text
前方区域：x = 0.2 到 1.2 m，|y| < 0.35 m
左前区域：x = 0.2 到 1.2 m，y = 0.2 到 0.8 m
右前区域：x = 0.2 到 1.2 m，y = -0.8 到 -0.2 m
```

规则：

```text
前方近距离有障碍：vx 降低或停止
左侧障碍更近：向右绕
右侧障碍更近：向左绕
两侧都堵：停止
目标在障碍物后面：不强行穿越
```

安全距离：

```text
hard_stop_distance: 0.45 m
slow_down_distance: 1.0 m
```

### 8.2 第二版避障

接入 Nav2 的 local costmap 和局部规划器：

```text
sensor_msgs/PointCloud2
  -> costmap_2d obstacle layer
  -> inflation layer
  -> DWB / TEB / MPPI local planner
  -> cmd_vel
```

对四足机器人，局部规划器输出的速度仍然是：

```text
geometry_msgs/Twist
vx, vy, yaw_rate
```

再由适配节点转换为 Go2 sport request：

```json
{"x": vx, "y": vy, "z": yaw_rate}
```

### 8.3 推荐实现顺序

```text
1. 先做人跟随，不避障，限速测试
2. 加前方急停障碍检测
3. 加左右绕障规则
4. 再接 Nav2 local costmap
5. 最后考虑全局地图
```

这样每一步都可验证，不会在 SLAM、检测、控制、避障同时出问题时无法定位。

## 9. 坐标系设计

必须统一 TF：

```text
map
  -> odom
    -> base_link
      -> camera_link
        -> camera_color_optical_frame
        -> camera_depth_optical_frame
```

第一阶段可以没有 `map`，但必须有：

```text
base_link -> camera_link
```

如果没有可靠里程计，也可以只做机器人自身局部坐标系下的跟随避障。

## 10. 与当前工作区的衔接

当前已有包：

```text
go2_keyboard_control
```

它已经能发布：

```text
/api/sport/request
```

运动参数格式：

```json
{"x": vx, "y": vy, "z": yaw_rate}
```

建议新增一个 `go2_follow_controller`，复用同样的发布方式。这样可以做到：

```text
键盘控制能跑
  -> 自动跟随控制替换键盘输入
  -> sport mode 运动链路不变
```

这是最稳的集成方式。

## 11. 参数建议

初始参数：

```yaml
follow_distance: 1.2
target_deadband: 0.15
max_vx: 0.35
max_reverse_vx: 0.10
max_vy: 0.20
max_yaw_rate: 0.6
cmd_rate: 20.0
target_timeout: 0.5
lost_stop_timeout: 2.0
slow_down_distance: 1.0
hard_stop_distance: 0.45
camera_timeout: 0.3
```

实机稳定后再提高：

```yaml
max_vx: 0.5
max_vy: 0.25
max_yaw_rate: 0.8
```

## 12. 安全机制

必须实现：

```text
相机超时 -> 停止
目标丢失超时 -> 停止
前方障碍过近 -> 停止
检测置信度过低 -> 降速或停止
速度命令超时 -> 停止
用户手动急停 -> 停止
节点退出 -> 发布 stop
```

停止方式：

```text
先发 sport stop API 1003
再持续发送 {"x":0,"y":0,"z":0}
```

当前 `go2_keyboard_control` 已经有类似逻辑，可以复用。

## 13. 分阶段里程碑

### M1：相机接入

完成标准：

- ROS 2 能看到 RGB、Depth、PointCloud2；
- RViz2 中点云方向正确；
- `camera_link -> base_link` TF 正确；
- 相机固定后走动不松动。

### M2：人体检测

完成标准：

- 能稳定检测 person；
- 能输出目标框；
- 多人场景下能保持同一目标；
- 目标丢失时状态正确。

### M3：目标三维定位

完成标准：

- 能输出人在 `base_link` 坐标系下的 `x, y`；
- 1 m 到 4 m 范围内距离误差可接受；
- 深度异常时不会输出跳变目标。

### M4：无避障跟随

完成标准：

- 空旷区域内能保持 1.2 m 左右距离；
- 人停机器人停；
- 人走机器人走；
- 人左右移动机器人能转向或横移跟随；
- 人丢失后机器人停止。

### M5：前方急停

完成标准：

- 前方 0.45 m 内有障碍时立刻停止；
- 障碍移开后可以恢复；
- 不会因为地面点云误判频繁急停。

### M6：局部绕障

完成标准：

- 前方有小障碍时能选择左或右绕行；
- 两侧堵住时停止；
- 不会为了跟人撞向障碍物。

### M7：Nav2 局部代价地图

完成标准：

- 点云能进入 local costmap；
- costmap 障碍物位置和实物一致；
- 局部规划器输出速度稳定；
- Go2 能按局部规划速度跟随。

### M8：全局地图，可选

完成标准：

- slam_toolbox 或 RTAB-Map 能建 2D 地图；
- Nav2 可以定位；
- 人跟随可以结合全局导航恢复路径。

## 14. 测试流程

测试顺序必须从低风险到高风险：

```text
1. 桌面回放相机 rosbag，验证检测和深度
2. Go2 不动，只打印跟随速度
3. Go2 架起或安全支撑，发送小速度
4. 空旷地面低速跟随
5. 加纸箱障碍，只测试急停
6. 加左右绕障
7. 多人和目标丢失测试
8. 光照变化测试
9. 长时间运行测试
```

每次测试都记录：

```text
/camera/*
/person_detections
/follow_target
/obstacle_summary
/api/sport/request
/tf
```

## 15. 关键风险

### 15.1 普通相机避障不可控

普通 RGB 相机只能检测人，不能可靠获得障碍物距离。要避障，强烈建议 RGB-D。

### 15.2 地面点云误判

相机下俯时会看到地面。必须过滤：

```text
z < -0.15 m 的点
z > 1.2 m 的点
距离太近的点
NaN 和 inf
```

具体阈值要根据 `base_link` 和相机安装高度调整。

### 15.3 目标深度跳变

检测框里可能混入背景深度。必须使用中位数、置信度和速度平滑。

### 15.4 Go2 运动惯性

Go2 sport mode 有内部控制器，速度命令不要突变。需要做加速度限制：

```text
max_acc_vx: 0.3 m/s^2
max_acc_vy: 0.3 m/s^2
max_acc_yaw: 0.8 rad/s^2
```

### 15.5 避障与跟随冲突

安全优先级必须高于跟随：

```text
hard stop > obstacle avoidance > target following
```

## 16. 最小可实现版本

最小版本不接 Nav2，也不做全局地图：

```text
RealSense / Orbbec
  -> YOLO person
  -> bbox depth median
  -> target x/y
  -> sector obstacle check from pointcloud
  -> P controller
  -> /api/sport/request
```

这个版本已经可以实现：

- 看到人；
- 保持距离；
- 前方有人或障碍时减速；
- 障碍太近时停止；
- 人丢失时停止。

这是建议先落地的版本。

## 17. 后续增强

可选增强：

- 使用 ByteTrack / DeepSORT 保持目标 ID；
- 使用 MediaPipe / YOLO-Pose 判断目标朝向；
- 使用 Nav2 MPPI 做更平滑的局部避障；
- 使用 RTAB-Map 做 RGB-D SLAM；
- 增加语音或手势开始跟随；
- 增加遥控器一键接管；
- 增加 rosbag 自动回放测试。

## 18. 最终建议

地图选择：

```text
先用局部 2D costmap，不建全局地图。
```

点云处理：

```text
实时点云需要转换成局部障碍地图；不需要先转换成全局点云地图。
```

控制方式：

```text
先使用 Go2 sport mode 速度接口 /api/sport/request。
```

实施路线：

```text
相机接入 -> 人体检测 -> 深度定位 -> 空旷跟随 -> 急停避障 -> 局部绕障 -> Nav2 local costmap -> 可选全局地图
```

只要按这个顺序推进，每一步都有可测标准，系统是可以实现的。
