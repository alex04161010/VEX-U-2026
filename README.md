# VEX U 机器人程序

中国高校智能机器人创意大赛 VEX U 专项赛的机器人程序，VEX V5 平台，C++ / VEXcode。

## 目录

| 目录 | 说明 |
|---|---|
| `vex_blue/` | 蓝方机器人完整程序（VEXcode VSCode 工程），吸吐球机构 + 气动 + 双 PID 环 |
| `vex_red/` | 红方机器人完整程序，结构与 `vex_blue` 相同，色相阈值与气动接口不同 |
| `周钟文new/` | 早期版本（VEXcode Pro 工程，含 `.v5code`），4+4 底盘，基础吸球 / 抛球 |
| `周钟文push/` | “大车”版本，5+5 底盘，独立 `PID.cpp`，多套自动阶段路线 |
| `docs/` | PID 控制框图（SVG/PNG）与带逐行注释的 PID 源码 |

每个工程目录结构一致：`src/`（`main.cpp` + `robot-config.cpp`）、`include/`、`makefile`、`vex/`（构建规则）、`.vscode/`。

## 硬件配置

### vex_blue / vex_red

| 端口 | 设备 |
|---|---|
| 15 / 2 / 3 / 4 | 左底盘电机 ×4（`ratio6_1`，最外侧反向） |
| 5 / 6 / 10 / 8 | 右底盘电机 ×4（`ratio6_1`，内侧反向） |
| 7 / 11 / 12 / 13 / 14 / 1 / 16 | 吸吐球机构电机 ×7（`ratio18_1`） |
| 21 | `inertial` 陀螺仪 |
| 20 | `optical` 颜色传感器（`setLightPower(70)`，用 `hue()` 分色） |
| ThreeWire H | 气动：下铲 |
| ThreeWire B | 气动：上左（仅 `vex_red`） |
| ThreeWire C | 气动：上右 |

### 周钟文new

底盘 4+4（左 5/6/7/8，右 1/2/3/4，`ratio6_1`）；`inertial` 陀螺仪、`inertial` 右侧陀螺仪、`rotation` 抛球转角、`distance` ×3、`limit` 限位、`motor`（吸球 / 抛球 / 抬臂）。气动：`arm` / `door` / `wing`。

### 周钟文push

底盘 5+5（左 4/7/5/12/13，右 1/2/3/14/15，含 `ratio6_1` 与 `ratio18_1` 混排）；陀螺仪 PORT11、`tai1` / `tai2` 抬升、`intake` / `intake2` 吸取、`Throw` 抛球、`Arm`（`ratio36_1`）、`rotation`、`distance`。气动：`arm` / `door` / `wing` / `Fuck`（钩子）。

## 程序结构

**底盘控制**：`base_drive(left, right, max_rpm, brakeType)` 统一驱动左右各 4 个（或 5 个）电机，把百分比速度换算成 rpm 下发，非零才 `spin`，为零时按指定 `brakeType` 停车；同时把左右两侧的平均转速峰值打印到 Brain 屏幕。

**遥控**：`user_move()` 读 `Axis3` 前后、`Axis1` 转向，带 3 死区；`X` 键用 `Reverse` 变量一键反转前后方向（边沿触发，靠 `Xpressed` 防连按）。按键映射 R1 吸球 / R2 吐下层 / L1 吐上层 / L2 吐中层。

**PID（双环）**：`TPID` 结构体存 `Kp/Ki/Kd/Error/Integral/Derivative/LastError/Output`，`initialize_pid()` 参数化两套环：

- `PIDEncoder` 读轮子编码器管直行（`PIDForward`），`Kp=0.155, Ki=0.01, Kd=0.01`
- `PIDGyro` 读陀螺仪管转向（`PIDTurnTo`），`Kp=0.75, Ki=0.03, Kd=0`

共用 `UpdatePIDController(PIDController, Error, tolerance)`：进容差即置 `insidetarget`，出界时清零计时器（所以计时的实际是“连续稳定时长”）；误差超过 30 不做积分累加（积分分离），抑制超调；微分取误差变化量。`PIDTurnTo` 带角度归一到 ±180°。`leftPIDTurnTo` 是只用左轮转向的变体。

**温度监控**：`tem()` 读取并打印 8 个底盘电机温度，用 `brakeType` 与温度阈值做保护，防止连续跑自动阶段烧电机。

**自动阶段**：`autonomous()` 由 `PIDForward(编码器度数, 最高速, 容差)` 与 `PIDTurnTo(目标角, 最高速, 容差)` 串联成路线，中间用 `wait()` 稳车；吸球段按 `ColorSensor.hue()` 阈值判断来球颜色，识到对方颜色就反转部分吸球电机把球吐掉（`intake_move` 的 7 个参数分别控制 7 个电机方向）。`周钟文push` 的自动阶段拆成 `rightAutonomous` / `leftAutonomous_basic` / `leftAutonomous_defence` / `leftAutonomous_defence_with_throw` 多套，并用 `normalize_angle` / `constrain` / `pid_correction` 做浮点航向修正。

## 构建

用 VEX Robotics 官方 VSCode 扩展（`vexrobotics.vexcode`）打开工程目录，构建 / 下载到 V5 Brain；`makefile` 与 `vex/*.mk` 是扩展调用的命令行构建链，需要本地已安装 VEX V5 工具链与 SDK。

`build/`、`bin/`、`compile_commands.json` 为本地构建产物，未纳入版本管理。
