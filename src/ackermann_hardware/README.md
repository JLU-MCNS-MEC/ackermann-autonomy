# Ackermann Hardware

Jetson AGX Xavier 上的 ROS 2 Python 底盘驱动。它使用原生 SocketCAN 控制 DS20270C
双轴驱动器，使用 Linux PWM sysfs 控制转向舵机。

## 运行环境

本包是 ROS 2 Jazzy 包，并使用 Python 3.10 以上的类型注解语法。Jetson AGX Xavier
能升级到的最高系统为 JetPack 5.1.7（Ubuntu 20.04），其对应的 ROS 2 发行版 Foxy 已停止
维护，因此本包目前不能直接运行在该 Jetson 上。真机运行需要桥接、容器或移植方案，
尚未确定；硬件层的接线、自检与 CAN 配置与本限制无关，已经可用。

驱动订阅 `/drive` 的 `ackermann_msgs/msg/AckermannDriveStamped`，发布：

- `/wheel/odometry`：原始轮式里程计，不发布 TF；
- `/joint_states`：后轮角速度和前轮转角；
- `/diagnostics`：使能状态、反馈时效、负载与 I/O 错误。

`~/enable` 是 `std_srvs/srv/SetBool` 服务。默认电机保持失能。首次台架测试必须架空
车轮、断开或卸载舵机连杆，并准备独立硬件急停。

DS20270C 当前使用厂家双轴协议，要求 `F0.1.002 = 1`、`F1.1.002 = 1`。默认节点 ID
为 `1`，发送 ID 为 `0x601`，反馈 ID 为 `0x581`。此实现不兼容 CANopen 模式 `20`。

## 硬件连接

- J30 Pin 31 为 `CAN0_DOUT`、Pin 29 为 `CAN0_DIN`；**必须外接 3.3 V CAN 收发器**
  （如 SN65HVD230）转换为 `CAN_H/CAN_L`，收发器的 `D` 接 Pin 31、`R` 接 Pin 29，
  不能把 Jetson 引脚直接连接 CAN 总线；
- 开发板**没有板载收发器**，收发器用 3.3 V 供电并与 Jetson 共地；5 V 收发器不能
  直接连接，排针信号不耐 5 V；
- 默认 PWM 控制器提示为 `32c0000.pwm`、通道 `0`，对应参考载板 J30 Pin 18；
- 舵机使用独立电源，只与 Jetson 共 PWM 信号和 GND；
- 总线两端各保留一个 120 Ω 终端电阻。

Jetson AGX Xavier 参考载板的设备树默认已启用 CAN0 与 PWM pinmux，实测无需 Jetson-IO
即可使用 `can0` 与 `32c0000.pwm`。只有更换载板、或引脚被复用导致接口不出现时，才需要
用 Jetson-IO 检查 pinmux。

完整 40 针映射、收发器接线图与台架排查见
[Jetson 硬件接线与台架准备](../../docs/jetson_hardware_setup.md)。

## 系统准备

`mttcan` 模块在 Jetson 上默认不加载，因此开机后没有 `can0` 接口。安装一次性服务后由
systemd 在开机时完成配置，之后 ROS launch 不需要 sudo：

```bash
ros2 run ackermann_hardware setup_can0.sh can0 500000
sudo "$(ros2 pkg prefix ackermann_hardware)/lib/ackermann_hardware/install_can_service.sh"
```

位速率在 `/etc/default/ackermann-can` 中修改，必须与 `config/chassis.yaml` 中的
`can_interface` 一致；安装脚本不会覆盖已经存在的配置文件，因此可以反复运行。

自检工具会检查平台、CAN 与 PWM 三部分，失败项直接给出修复命令：

```bash
ros2 run ackermann_hardware hardware_preflight
```

驱动默认参数位于 `config/chassis.yaml`。其中轮径 `0.169 m`、轮距 `0.40 m`、轴距
`0.56 m` 和舵机 `150°` 中位都必须根据实车复测，不能直接作为机械标定结果。

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
ros2 run ackermann_hardware hardware_preflight
ros2 run ackermann_hardware setup_can0.sh can0 500000
ros2 launch ackermann_bringup real_chassis.launch.py
ros2 service call /chassis_driver/enable std_srvs/srv/SetBool '{data: true}'
```

使能前必须架空车轮，并先用 `candump can0` 确认反馈。独立硬件急停、驱动心跳和人工
遥控抢占不能由 ROS watchdog 替代。
