# Ackermann Hardware

Jetson AGX Xavier 上的 ROS 2 Python 底盘驱动。它使用原生 SocketCAN 控制 DS20270C
双轴驱动器，使用 Linux PWM sysfs 控制转向舵机。

驱动订阅 `/drive` 的 `ackermann_msgs/msg/AckermannDriveStamped`，发布：

- `/wheel/odometry`：原始轮式里程计，不发布 TF；
- `/joint_states`：后轮角速度和前轮转角；
- `/diagnostics`：使能状态、反馈时效、负载与 I/O 错误。

`~/enable` 是 `std_srvs/srv/SetBool` 服务。默认电机保持失能。首次台架测试必须架空
车轮、断开或卸载舵机连杆，并准备独立硬件急停。

DS20270C 当前使用厂家双轴协议，要求 `F0.1.002 = 1`、`F1.1.002 = 1`。默认节点 ID
为 `1`，发送 ID 为 `0x601`，反馈 ID 为 `0x581`。此实现不兼容 CANopen 模式 `20`。

## 硬件连接

- J30 Pin 31 为 `CAN0_TX`，Pin 29 为 `CAN0_RX`；必须通过 3.3 V CAN 收发器转换为
  `CAN_H/CAN_L`，不能把 Jetson 引脚直接连接 CAN 总线；
- 默认 PWM 控制器提示为 `32c0000.pwm`、通道 `0`，对应参考载板 J30 Pin 18；
- 舵机使用独立电源，只与 Jetson 共 PWM 信号和 GND；
- 总线两端各保留一个 120 Ω 终端电阻。

首次运行前使用 Jetson-IO 配置 CAN0 与 PWM pinmux，并重启。驱动默认参数位于
`config/chassis.yaml`。其中轮径 `0.169 m`、轮距 `0.40 m`、轴距 `0.56 m` 和舵机
`150°` 中位都必须根据实车复测，不能直接作为机械标定结果。

## 控制与反馈

控制链保持为：

```text
Nav2 -> velocity_smoother -> Collision Monitor -> /cmd_vel_safe
     -> twist_to_ackermann -> /drive -> chassis_driver
```

`chassis_driver` 根据速度和转角计算左右后轮 RPM。DS20270C 返回的 RPM 会转换为原始
轮式里程计；节点不发布 TF，由后续 `robot_localization` 融合 IMU 后独占
`odom -> base_footprint`。

Python 节点只进行 20–50 Hz 的目标下发、反馈读取与 ROS 消息处理。电机速度闭环位于
DS20270C 内部，因此 Python 不会成为当前带宽下的性能瓶颈；它仍不是硬实时安全控制器。

```bash
ros2 run ackermann_hardware setup_can0.sh can0 500000
ros2 launch ackermann_bringup real_chassis.launch.py
ros2 service call /chassis_driver/enable std_srvs/srv/SetBool '{data: true}'
```

使能前必须架空车轮，并先用 `candump can0` 确认反馈。独立硬件急停、驱动心跳和人工
遥控抢占不能由 ROS watchdog 替代。
