# Jetson 硬件接线与台架准备

调研与实测日期：2026-10-13。实测平台：Jetson AGX Xavier 32GB，JetPack 4.4
（L4T R32.4.4，Ubuntu 18.04，内核 4.9.140-tegra）。

本文是底盘硬件接入的唯一完整参考：40 针映射、CAN 收发器接线、一次性系统配置与台架
排查。驱动接口与安全要求见
[`ackermann_hardware/README.md`](../src/ackermann_hardware/README.md)。

## 1. 运行环境限制

本工程建立在 ROS 2 Jazzy 上，包内使用了 Python 3.10 以上的类型注解语法。Jetson AGX
Xavier 能升级到的最高系统是 JetPack 5.1.7（Ubuntu 20.04），对应 ROS 2 发行版 Foxy
且已停止维护。因此底盘驱动目前不能直接运行在这台 Jetson 上。

本文描述的全部内容属于硬件层，与该限制无关：接线、CAN 接口启用、PWM 输出和自检工具
都已经在真机验证。真机运行方案（桥接、容器或移植）尚未确定。

## 2. 40 针接口映射

载板上丝印为 J30，对应开发板 40 针排针。功能名取自 `Jetson.GPIO` 的
`JETSON_XAVIER_PIN_DEFS` 定义表，与内核实际枚举的控制器一致。

| 引脚 | 功能 | 引脚 | 功能 |
| --- | --- | --- | --- |
| 1 | 3.3 V | 2 | 5 V |
| 3 | I2C1_SDA | 4 | 5 V |
| 5 | I2C1_SCL | 6 | GND |
| 7 | MCLK05 | 8 | UART1_TXD |
| 9 | GND | 10 | UART1_RXD |
| 11 | UART1_RTS | 12 | I2S2_CLK |
| 13 | PWM（`32f0000.pwm`） | 14 | GND |
| 15 | PWM（`3280000.pwm`） | 16 | CAN1_STB |
| 17 | 3.3 V | 18 | PWM（`32c0000.pwm`） |
| 19 | SPI1_MOSI | 20 | GND |
| 21 | SPI1_MISO | 22 | GPIO |
| 23 | SPI1_CLK | 24 | SPI1_CS0 |
| 25 | GND | 26 | SPI1_CS1 |
| 27 | I2C0_SDA | 28 | I2C0_SCL |
| 29 | CAN0_DIN | 30 | GND |
| 31 | CAN0_DOUT | 32 | CAN1_EN |
| 33 | CAN1_DOUT | 34 | GND |
| 35 | I2S2_FS | 36 | UART1_CTS |
| 37 | CAN1_DIN | 38 | I2S2_DIN |
| 39 | GND | 40 | I2S2_DOUT |

驱动实际使用的三个引脚：

| 用途 | 引脚 | 控制器 |
| --- | --- | --- |
| 转向舵机 PWM | 18 | `32c0000.pwm` 通道 0（`pwmchip2`） |
| 动力电机 CAN0 接收 | 29 | `mttcan@c310000` |
| 动力电机 CAN0 发送 | 31 | `mttcan@c310000` |

除 3.3 V 与 5 V 电源引脚外，排针上所有信号都是 3.3 V 逻辑，**不耐 5 V**，接入 5 V
信号会损坏 SoC。电机驱动器与收发器必须与本机共地。

## 3. CAN 收发器接线

开发板**没有板载 CAN 收发器**，设备树中也没有收发器节点。引脚 29 与 31 引出的是 SoC
CAN 控制器的 3.3 V TTL 逻辑电平（`CAN0_DIN` / `CAN0_DOUT`），不是差分 `CAN_H` /
`CAN_L`。必须外接 3.3 V 收发器，NVIDIA 推荐 SN65HVD230 或同类器件。

```text
SN65HVD230          Jetson 40 针
──────────          ────────────
3V3    ←──────────→ 引脚 1   (3.3 V)
GND    ←──────────→ 引脚 6   (GND，必须连接)
RXD    ←──────────→ 引脚 29  (CAN0_DIN)
TXD    ←──────────→ 引脚 31  (CAN0_DOUT)
CANH   ────────────→ 电机驱动器 CAN_H
CANL   ────────────→ 电机驱动器 CAN_L
```

只有 `CANH` / `CANL` 进入总线。总线两端各需要一个 120 Ω 终端电阻，合计两个。
收发器供电与逻辑都是 3.3 V；5 V 器件（例如 MCP2551）不能直接连接。

接口起来后 `ip -details link show can0` 显示 `can state ERROR-ACTIVE` 只说明控制器
就绪，**不代表总线已经连通**；总线连通性需要接上第二个 CAN 节点后实测确认。

## 4. 一次性系统配置

`mttcan` 模块在开机时不会自动加载，因此默认没有 `can0` 接口。安装一次服务由 systemd
在每次开机时完成配置，之后运行驱动不再需要 sudo：

```bash
cd /home/xqiao/Workspace/ackermann_ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select ackermann_hardware
source install/local_setup.zsh

sudo "$(ros2 pkg prefix ackermann_hardware)/lib/ackermann_hardware/install_can_service.sh"
```

安装脚本会生成 `/etc/systemd/system/ackermann-can.service`、保留
`/etc/default/ackermann-can` 中已有的配置，并设为开机自启。反复运行是安全的。

界速率与接口名在 `/etc/default/ackermann-can` 中修改，必须与
`config/chassis.yaml` 的 `can_interface` 一致。修改后重新运行安装脚本。

不使用服务时也可以手动临时配置，但每次开机都要重来：

```bash
ros2 run ackermann_hardware setup_can0.sh can0 500000
```

服务只负责配置接口，不负责关闭电机。`systemctl restart ackermann-can.service` 可用于
在接口被手动改乱后恢复。

## 5. 自检与验证

台架调试前先跑自检，它会检查平台、CAN 模块、CAN 接口和 PWM 四部分，失败项直接给出
修复命令：

```bash
ros2 run ackermann_hardware hardware_preflight
```

手动确认各部分：

```bash
ip -details link show can0                  # 期望 can state ERROR-ACTIVE
ls -l /sys/class/pwm                        # 期望存在 pwmchip2 -> 32c0000.pwm

# PWM 输出实测：50 Hz、1.5 ms 中位脉宽
cd /sys/class/pwm/pwmchip2
echo 0 > export && sleep 1
echo 20000000 > pwm0/period
echo 1500000 > pwm0/duty_cycle
echo 1 > pwm0/enable
cat pwm0/period pwm0/duty_cycle pwm0/enable
```

PWM 与 GPIO 权限：`export` 文件的属主是 `root:gpio`，登录用户在 `gpio` 组内，因此
PWM 操作不需要 sudo。`/dev/gpiochip*` 默认仅 root 可访问，但当前驱动不使用 GPIO，
只有将来引入 libgpiod 或 `Jetson.GPIO` 时才需要额外配置 udev 规则。

## 6. 台架调试顺序

1. 举升车轮，让驱动轮离地；断开或卸载舵机连杆。
2. 跑 `hardware_preflight`，确认 CAN 与 PWM 就绪。
3. 用 `candump can0` 确认 DS20270C 有反馈帧，核对 CAN ID 为 `0x581`。
4. 调用 `~/enable` 服务使能，先在空载下核对左右轮转向与转速方向。
5. 接回舵机连杆，用低速命令核对舵机中位与左右极限。
6. 落轮，按 [`sim2real_plan.md`](sim2real_plan.md) 的分阶段验收做低速几何与里程计测试。

全过程必须保持独立硬件急停可用。ROS watchdog 不能替代急停、驱动心跳和人工遥控抢占。

## 7. 验收状态

已经在本机实测通过：

- `mttcan` 模块存在，手动加载后 `can0` 与 `can1` 正常出现；
- `can0` 可配置到 500 kbps 并进入 `ERROR-ACTIVE`；
- PWM 控制器 `32c0000.pwm` 对应 `pwmchip2` 与引脚 18，实测可输出 50 Hz；
- PWM 与 GPIO 权限满足免 sudo 使用。

尚未验收，需要接上真实硬件：

- 总线实际连通性（需要第二个 CAN 节点）；
- 左右轮方向、舵机中位与极限；
- 负载反馈语义与实车里程计精度；
- 硬件急停、驱动心跳与遥控抢占。

不得把本文的接口验证结果当作整车闭环完成。

## 8. 故障排查

| 症状 | 可能原因 | 处理 |
| --- | --- | --- |
| 找不到 `can0` | `mttcan` 未加载 | 装服务或跑 `setup_can0.sh`；自检会指出 |
| 加载模块后仍无接口 | 设备树未启用该控制器 | 确认非 Jetson 平台；换载板时用 Jetson-IO 检查 |
| `ERROR-PASSIVE` 或 `BUS-OFF` | 收发器未接线、极性接反、缺少终端电阻 | 核对 Pin 29/31 与收发器 `R`/`D`，检查两端 120 Ω |
| `export` 写入失败 | 当前用户不在 `gpio` 组 | 加入 `gpio` 组后重新登录 |
| 自检报的 pwmchip 与引脚不符 | `controller_hint` 配错 | `32c0000.pwm` 对应引脚 18，`32f0000.pwm` 对应 13 |
| 服务重启后接口又变 down | 有人手动 `ip link set down` | `systemctl restart ackermann-can.service` |
| 舵机抖动或不回中 | 舵机未独立供电，或与 Jetson 未共地 | 舵机单独供电，只与 Jetson 共 PWM 信号与 GND |
