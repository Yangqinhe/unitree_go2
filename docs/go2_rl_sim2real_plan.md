# Unitree Go2 强化学习行走与 Sim2Real 实施方案

本文档给出一个从零开始训练 Unitree Go2 行走策略，并迁移到真机的工程实施方案。目标不是一上来替换所有官方运动能力，而是先训练一个可控、可验证、可安全回退的低层 locomotion policy，再逐步扩展到复杂地形、导航和任务层。

## 1. 总体目标

目标能力：

- 平地速度跟踪：给定 `vx, vy, yaw_rate`，Go2 能稳定前进、后退、横移和转向。
- 抗扰动：轻微推搡、质量变化、地面摩擦变化下不摔倒。
- Sim2Real：策略先在 Isaac Lab/Isaac Sim 中训练，再在 MuJoCo 或 Unitree 官方仿真中做二次验证，最后部署到 Go2 真机。
- 安全运行：真机部署时必须有急停、姿态保护、关节限位、力矩限幅、超时停机和遥控器接管。

推荐路线：

```text
Isaac Lab 训练 PPO policy
        |
        v
导出 TorchScript / ONNX
        |
        v
MuJoCo / Unitree 官方仿真 Sim2Sim 验证
        |
        v
真机低层控制部署
        |
        v
逐步扩大速度、地形和扰动范围
```

## 2. 推荐技术栈

### 2.1 训练平台

首选：Isaac Lab + Isaac Sim。

原因：

- Isaac Lab 已经支持 Unitree Go2 这类四足机器人。
- 支持大规模并行环境，训练速度比单环境 MuJoCo 快很多。
- 内置强化学习任务、机器人资产、域随机化、传感器和 terrain 工具。
- 适合训练速度跟踪、地形行走、扰动恢复等 locomotion policy。

备选：Unitree 官方 `unitree_rl_gym` 或 `unitree_rl_lab`。

原因：

- 更接近 Unitree 官方训练链路。
- 对 Go2/H1/G1 等机器人有现成示例。
- 如果你想先复现实例，官方 RL 仓库比从零搭 Isaac Lab 任务更快。

Sim2Sim 验证：Unitree 官方 `unitree_mujoco`。

原因：

- 官方说明该仿真器基于 `unitree_sdk2` 和 MuJoCo，可用于低层控制和 sim-to-real 验证。
- 它支持 `LowCmd`、`LowState`、`SportModeState` 等 Unitree SDK2 消息。
- 可以在上真机前检查控制频率、关节方向、零位、PD 参数、限幅和消息链路。

### 2.2 真实机器人通信

Go2 真机低层策略部署建议走 Unitree SDK2 / ROS2 DDS 低层接口，而不是 `/api/sport/request` 高层运动接口。

区别：

- `/api/sport/request`：调用官方内置动作，比如站立、移动、打招呼、跳跃等。适合行为调用，不适合部署自训练 RL 低层步态。
- `/lowcmd` 或 SDK2 `LowCmd`：直接控制各关节的目标位置、速度、力矩、PD 增益。RL policy 真正部署行走通常需要这一层。

因此，你现在写的 `api_id=1016` 打招呼节点属于高层动作调用；RL 行走部署要切到低层控制。

## 3. 工程目录建议

建议将工作区拆成以下模块：

```text
unitree_go2_ws/
  src/
    go2_hello_cpp/                 # 当前高层 API 测试节点
    go2_keyboard_control/          # 当前键盘控制节点
    go2_lowlevel_bridge/           # 真机低层 ROS2/SDK2 接口
    go2_policy_runner/             # C++ 或 Python policy 推理节点
    go2_safety_monitor/            # 姿态/关节/通信安全监控
  rl/
    isaac_lab_go2/                 # Isaac Lab 训练任务配置
    policies/                      # 导出的策略模型
    logs/                          # 训练日志
    export/                        # TorchScript/ONNX
  sim/
    unitree_mujoco/                # Sim2Sim 验证
  docs/
    go2_rl_sim2real_plan.md
```

如果训练在另一台 GPU 主机上进行，ROS2 真机工作区和训练仓库可以分开，但模型导出格式和观测/动作定义必须严格同步。

## 4. 控制架构

### 4.1 分层结构

推荐三层：

```text
任务层 / 人类输入
  - 键盘、手柄、导航栈、速度命令
  - 输出 command: vx, vy, yaw_rate

RL 策略层
  - 输入机器人状态和速度命令
  - 输出 12 个关节目标位置，或关节残差 action

低层执行层
  - 读取 LowState
  - 发送 LowCmd
  - PD 控制、限幅、急停、站立初始化
```

### 4.2 推荐动作空间

初期不要让策略直接输出力矩。推荐让策略输出关节目标位置残差：

```text
policy_action: 12 维，范围 [-1, 1]
target_q = default_joint_pos + action_scale * policy_action
```

低层发送：

```text
q_des  = target_q
dq_des = 0
kp     = 每个关节固定 PD 刚度
kd     = 每个关节固定 PD 阻尼
tau_ff = 0 或小范围前馈
```

优点：

- 比直接力矩安全。
- Sim2Real 更稳定。
- 容易加限幅。
- 真机异常时可以快速回到默认站立姿态。

### 4.3 推荐控制频率

建议：

- 低层发送频率：500 Hz 或接近官方低层控制要求。
- policy 推理频率：50 Hz。
- 每次 policy 输出保持 10 个低层周期。

示例：

```text
LowState 读取：500 Hz
LowCmd 发送：500 Hz
Policy inference：50 Hz
Action decimation：10
```

如果实际通信和系统调度达不到 500 Hz，先不要上真机跑自训练步态。

## 5. 训练任务设计

第一阶段只做平地速度跟踪。

### 5.1 观测 Observation

推荐观测：

```text
base angular velocity        3
projected gravity            3
command vx, vy, yaw_rate      3
joint position error         12
joint velocity               12
previous action              12
--------------------------------
total                        45
```

可选增强：

- gait phase 或 sin/cos phase。
- 最近几帧历史 observation。
- base linear velocity 只在仿真训练时用于 critic，不建议直接给 actor，除非真机也能可靠估计。
- terrain height samples 用于复杂地形阶段，不要第一阶段就加。

### 5.2 动作 Action

```text
action_dim = 12
action_scale = 0.25 到 0.5 rad
```

关节顺序必须和 Go2 真机低层接口完全一致。上线前单独写一个关节点动测试程序，每次只让一个关节小幅运动，确认：

- 左右腿没有反。
- 髋/大腿/小腿方向没有反。
- 默认站姿和仿真一致。
- 弧度单位一致。

### 5.3 奖励 Reward

初始奖励建议：

```text
速度跟踪：
  exp(-|vxy_cmd - vxy|^2 / sigma)
  exp(-|yaw_cmd - yaw_rate|^2 / sigma)

姿态稳定：
  惩罚 roll/pitch 偏离
  惩罚 z 方向角速度过大

能耗与平滑：
  惩罚 torque^2
  惩罚 action rate
  惩罚 joint acceleration

关节安全：
  惩罚接近关节限位
  惩罚关节速度过大

足端行为：
  奖励足端离地时间
  惩罚脚拖地
  惩罚不必要滑动

生存：
  未摔倒持续奖励
  摔倒终止 episode
```

常见坑：

- 只追速度会学出拖脚、抖腿、耗能很大的步态。
- 地面摩擦设置过高时，策略可能靠拖脚移动，真机上失败。
- 没有限制 action rate，真机会抖。
- 仿真默认 PD 太理想，真机电机延迟和带宽跟不上。

## 6. 域随机化 Domain Randomization

Sim2Real 的核心不是把仿真调到唯一正确，而是让策略见过足够多的合理误差。

建议逐步加入：

```text
机器人质量：          ±10%
机身质心偏移：        x/y/z 小范围随机
关节摩擦/阻尼：       随机
地面摩擦：            0.4 到 1.5
电机强度：            0.8 到 1.2
PD 增益：             ±20%
动作延迟：            0 到 2 个 policy step
观测噪声：            IMU、关节位置、关节速度
外力扰动：            随机推力
命令范围：            从小速度逐步扩大
```

不要一开始随机化过强。建议课程化：

```text
阶段 1：无随机化，学会站稳和小速度行走
阶段 2：加入摩擦、质量、观测噪声
阶段 3：加入延迟、电机强度、外力扰动
阶段 4：加入斜坡、台阶、离散地形
```

## 7. 训练流程

### 7.1 环境准备

GPU 主机建议：

- Ubuntu 22.04
- NVIDIA Driver 与 CUDA 版本匹配 Isaac Sim 要求
- Isaac Sim
- Isaac Lab
- PyTorch
- rsl_rl 或 skrl

ROS2 真机主机建议：

- Ubuntu 22.04
- ROS2 Humble
- Unitree ROS2 / SDK2
- CycloneDDS 配置正确
- 有线网或稳定 Wi-Fi，优先有线

### 7.2 先复现官方示例

先不要急着写自己的 reward。建议按顺序：

1. 跑通 Isaac Lab 自带 Go2 locomotion 示例。
2. 跑通官方或社区已有 Go2 平地 policy。
3. 导出 policy 并在 Isaac Sim 播放验证。
4. 在 Unitree MuJoCo 做 Sim2Sim。
5. 再改 observation/reward/action。

判断“训练成功”的最低标准：

- 平地 `vx=0.3 m/s` 稳定走 30 秒。
- 原地 `yaw_rate=0.5 rad/s` 稳定转 30 秒。
- `vx/vy/yaw` 命令切换时不会摔。
- episode 中平均关节速度和 torque 不异常。
- 脚没有明显长期拖地。

### 7.3 训练参数起点

PPO 常用起点：

```yaml
num_envs: 4096
policy_rate: 50
control_decimation: 10
episode_length_s: 20
learning_rate: 1.0e-3
entropy_coef: 0.01
clip_param: 0.2
num_learning_epochs: 5
num_mini_batches: 4
gamma: 0.99
lam: 0.95
```

这些不是固定答案，要结合 Isaac Lab 任务默认值和训练曲线调整。

## 8. 模型导出

训练完成后导出：

- TorchScript：部署到 Python/C++ libtorch 较方便。
- ONNX：跨语言部署方便，可用 ONNX Runtime。

导出时必须保存同一份元数据：

```yaml
policy_name: go2_flat_velocity_v1
obs_dim: 45
action_dim: 12
control_dt: 0.02
action_scale: 0.25
default_joint_pos: [...]
joint_order: [...]
obs_order: [...]
obs_normalization_mean: [...]
obs_normalization_std: [...]
clip_obs: 100.0
clip_action: 1.0
```

最容易出事故的不是神经网络本身，而是部署端 observation 顺序、归一化、关节顺序和仿真不一致。

## 9. Sim2Sim 验证

在上真机前，至少用两个仿真器验证：

```text
训练仿真：Isaac Lab
验证仿真：Unitree MuJoCo
```

验证项：

- policy runner 是否能以目标频率运行。
- LowCmd 消息是否正确。
- 关节方向和默认姿态是否正确。
- policy 输入输出是否和训练时一致。
- 超时停机是否有效。
- 摔倒检测是否有效。
- 命令从小到大逐步增加是否稳定。

建议先只发站立目标姿态，不加载 policy：

```text
default_joint_pos + PD -> 站稳
```

再加载 policy：

```text
vx=0.0, vy=0.0, yaw=0.0 -> 原地站稳
vx=0.1 -> 慢速前进
yaw=0.2 -> 慢速转向
```

## 10. 真机部署架构

建议写两个 ROS2 C++ 节点：

```text
go2_policy_runner
  subscribe:
    /lowstate
    /cmd_vel 或 /go2_rl_cmd
  publish:
    /lowcmd

go2_safety_monitor
  subscribe:
    /lowstate
    /go2_policy_status
  publish:
    /go2_emergency_stop
```

policy runner 逻辑：

```text
1. 启动后不立即控制
2. 等待 LowState 稳定
3. 检查机器人处于低层控制允许状态
4. 缓慢插值到默认站姿
5. policy 输出从 0 action 开始
6. 接收速度命令
7. 每 20 ms 推理一次
8. 每 2 ms 或实际低层周期发送 LowCmd
9. 任何异常进入 damping/stop
```

### 10.1 安全条件

触发停机条件：

```text
roll/pitch 超过阈值
base height 过低
关节位置超过软限位
关节速度超过阈值
电机温度过高
LowState 超时
policy 推理超时
速度命令超时
遥控器急停
用户 Ctrl+C
```

停机策略：

```text
首选：进入 damping 模式
其次：发送零力矩并让官方控制接管
不要：突然发送大幅默认姿态
```

## 11. 真机测试步骤

### 11.1 上电前检查

- 场地空旷，地面防滑。
- 机器人周围没有人和易碎物。
- 电量充足。
- 遥控器可随时接管。
- 电脑和机器人网络稳定。
- 能实时看到日志。

### 11.2 吊起测试

第一次必须吊起或让四足悬空：

```text
1. 只读 LowState，不发 LowCmd
2. 发 default_joint_pos，小 PD
3. 单关节点动
4. 全关节站姿插值
5. policy vx=0
6. policy 小速度命令
```

如果吊起时腿疯狂抖动，立即停止。优先查：

- 关节顺序。
- 关节方向。
- PD 增益。
- action scale。
- observation normalization。
- 控制频率。

### 11.3 落地测试

落地测试从最小命令开始：

```text
vx = 0.0, vy = 0.0, yaw = 0.0, 站 10 秒
vx = 0.05, 走 3 秒
vx = 0.10, 走 3 秒
yaw = 0.10, 转 3 秒
```

不要一开始就 `vx=0.5`。每次只改一个变量。

## 12. 里程碑

### M0：通信和高层动作验证

完成标准：

- `/api/sport/request` 有订阅者。
- `api_id=1016` 打招呼能执行。
- `/lowstate` 能稳定读取。
- DDS 网卡配置稳定。

### M1：低层站立控制

完成标准：

- 能发送 LowCmd。
- Go2 能从安全姿态插值到默认站姿。
- Ctrl+C 或急停能进入 damping。

### M2：仿真平地行走

完成标准：

- Isaac Lab 中平地速度跟踪成功。
- policy 可导出。
- replay 稳定。

### M3：Sim2Sim

完成标准：

- 同一 policy 在 Unitree MuJoCo 中可站立、慢走、转向。
- 控制频率和消息链路稳定。

### M4：吊起真机测试

完成标准：

- 不抖动。
- 单关节方向正确。
- policy 输出幅度合理。

### M5：落地慢速行走

完成标准：

- `vx <= 0.2 m/s` 稳定前进。
- 原地转向稳定。
- 遇到异常能安全停机。

### M6：扩展能力

完成标准：

- 提高速度范围。
- 加入横移。
- 加入斜坡、草地、小障碍。
- 接入导航层。

## 13. 当前你应该先做什么

你现在还在高层 API 和 ROS2 通信阶段，建议先完成：

```bash
ros2 topic info /api/sport/request -v
ros2 topic echo /lowstate --once
ros2 topic echo /lf/sportmodestate --once
```

如果 `/api/sport/request` 没有 subscriber，先修 DDS/网络，不要开始 RL。

接着做一个低层只读节点：

```text
订阅 /lowstate
打印 12 个关节位置、速度、IMU roll/pitch
确认频率稳定
```

然后做一个低层站立节点：

```text
读取当前关节角
3 到 5 秒插值到默认站姿
发送 PD 位置控制
Ctrl+C 进入 damping
```

只有这个稳定后，才进入 RL policy 部署。

## 14. 风险和注意事项

- Go2 真机低层控制有摔机风险，第一次不要落地跑。
- 不要在电量低、地面滑、人多或空间狭窄环境测试。
- 不要直接用别人 policy 上真机，除非确认机器人型号、关节顺序、控制频率、PD 参数、归一化完全一致。
- 不要把高层 `/api/sport/request` 和低层 `/lowcmd` 混着抢控制。
- policy 输出必须限幅，速度命令必须限幅。
- 所有部署代码必须有 watchdog。
- 每次测试都保留日志：命令、关节状态、IMU、policy action、异常原因。

## 15. 推荐资料

- Unitree ROS2：`https://github.com/unitreerobotics/unitree_ros2`
- Unitree MuJoCo：`https://github.com/unitreerobotics/unitree_mujoco`
- Unitree 官方 GitHub：`https://github.com/unitreerobotics`
- Isaac Lab 文档：`https://isaac-sim.github.io/IsaacLab/`
- Isaac Sim Go2 policy 示例：`https://docs.isaacsim.omniverse.nvidia.com/`

## 16. 最小可行路线

如果只追求最快看到成果：

```text
1. 修通 Go2 DDS 网络
2. 跑通 Unitree 官方高层 API 和低层 LowState
3. 安装 Isaac Lab
4. 跑通 Isaac Lab Go2 平地 locomotion 示例
5. 导出现成/复现 policy
6. 在 Unitree MuJoCo 中跑 policy
7. 写 go2_policy_runner
8. 吊起真机测试
9. 落地 0.05 m/s 慢走
10. 逐步扩展速度和地形
```

这条路线比“直接自己写 reward 然后上真机”安全得多，也更容易定位问题。
