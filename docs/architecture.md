# Ackermann Autonomy 架构

## 项目定位

`Ackermann Autonomy` 是面向 Ackermann 车辆的 ROS 2 自主导航单仓库。工程同时支持
Gazebo 仿真验证和实车集成，但两者只在明确的车辆接口处汇合，仿真真值不得进入实车
算法依赖链。

推荐的 GitHub 仓库名为 `ackermann-autonomy`。在实车驱动协议稳定、需要独立发布或由
不同团队维护之前，不拆分独立仓库。

## 包边界

| 包 | 职责 | 允许依赖 |
| --- | --- | --- |
| `ackermann_description` | 车体、传感器和 TF 几何描述 | ROS 2 描述工具 |
| `ackermann_autonomy` | 控制适配、导航辅助、语义与视觉算法、诊断和测试节点 | ROS 2 通用消息与算法库 |
| `ackermann_hardware` | Jetson SocketCAN、DS20270C、PWM、轮速反馈与原始里程计 | Linux 硬件接口与 ROS 2 通用消息 |
| `ackermann_bringup` | 实机控制边界和平台无关算法测试入口 | `ackermann_description`、`ackermann_autonomy`、`ackermann_hardware` |
| `ackermann_simulation` | Gazebo world、bridge、仿真参数、地图、RViz 和回归场景 | 上述共享包、Gazebo、Nav2 |

依赖方向必须保持单向：

```text
ackermann_description ─┐
ackermann_autonomy ────┼─> ackermann_bringup ─> ackermann_simulation
ackermann_hardware ────┘
```

`ackermann_autonomy` 不得依赖 Gazebo。实车驱动不得订阅 `/tf_ground_truth`，也不得绕过
`/cmd_vel_safe -> twist_to_ackermann -> /drive` 控制边界。

## 运行配置

### 仿真

仿真入口位于 `ackermann_simulation`，负责 Gazebo、bridge、仿真传感器、仿真定位和
自动回归。它可以使用 `/tf_ground_truth` 计算评估指标，但正式导航 TF 必须来自里程计、
AMCL 或 SLAM。

### 实机与台架

实机入口位于 `ackermann_bringup`。当前已经提供硬件无关的控制边界：

```text
Nav2 -> velocity_smoother -> Collision Monitor -> /cmd_vel_safe
     -> twist_to_ackermann -> /drive -> chassis driver
```

`ackermann_hardware` 已作为独立 ROS 2 Python 包加入本仓库，订阅 `/drive`，发布
`/wheel/odometry`、`/joint_states` 和 `/diagnostics`，并实现命令超时停车。它不发布
`odom -> base_footprint`，该 TF 仍由 `robot_localization` 独占。驱动包不得包含 Nav2、
Gazebo 或上层算法实现。

## 何时拆分仓库

只有满足以下至少一项时，再把硬件驱动拆成独立仓库：

1. 驱动需要独立版本、发布节奏或访问权限；
2. 同一驱动被多个非本项目车辆复用；
3. 厂商 SDK 的许可证或二进制分发规则要求隔离；
4. 驱动已有稳定接口，并能通过独立硬件在环测试。

算法、描述、仿真场景和集成测试仍建议保留在本仓库，以便一次提交完成接口变更与
Sim2Real 回归。

## 命名迁移

旧包名与新包名的对应关系：

| 旧包 | 新包 |
| --- | --- |
| `ackermann_line_following_description` | `ackermann_description` |
| `ackermann_line_following_controller` | `ackermann_autonomy` |
| `ackermann_line_following_bringup` | `ackermann_simulation` + `ackermann_bringup` |

迁移后必须清理旧的 `build/`、`install/` 和 `log/`，再重新执行 `colcon build`，避免
ament 索引中残留旧包名。
