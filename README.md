# ESP32 CNC 板 A4988 X 轴测试项目

本项目用于验证 **ESP32 + CNC 板 + A4988 驱动器 + X 轴步进电机** 的控制链路是否正常。
程序已经实现自动往返、串口手动运动、速度调节、加速度调节、软停止和 HOME 回零，适合作为你后续做单轴控制、多轴扩展或接入更完整 CNC 逻辑之前的基础工程。

## 1. 项目用途

这个项目主要解决以下几个问题：

- 验证 ESP32 是否能稳定输出 X 轴的 `STEP` 和 `DIR` 脉冲
- 验证 A4988 驱动器在 1/16 细分下是否能正常带动电机
- 验证 CNC 板使能逻辑、方向逻辑和限位开关逻辑是否正确
- 提供一套可通过串口实时测试的控制框架，方便调试速度、加速度和回零

如果你当前目标是先把 X 轴单独跑顺，这个工程就是为这个阶段准备的。

## 2. 当前功能总览

当前固件已经具备以下能力：

- 自动模式：默认正转 3 圈，停顿，再反转 3 圈，循环执行
- 手动模式：支持 `+N` 和 `-N`，按圈数临时控制电机转动
- 串口调速：支持通过串口动态修改巡航速度
- 串口调加速度：支持通过串口动态修改加速度
- 软停止：运动过程中收到停止命令后，不会硬切断，而是按减速曲线停车
- HOME 回零：支持通过限位开关执行 X 轴回零
- AUTO 开关：支持随时开启或关闭自动往返模式
- 状态查询：支持通过串口查看当前参数和运行状态

## 3. 默认运行逻辑

上电后程序会完成初始化，然后进入默认自动模式。

默认动作流程如下：

1. 正转 3 圈
2. 停顿 1 秒
3. 反转 3 圈
4. 停顿 1 秒
5. 重复以上流程

如果在自动模式运行中收到手动命令，例如 `+1` 或 `-2`，控制器会先打断自动流程，等待 1 秒，再执行你输入的手动动作。

如果收到 `STOP`，则当前运动会按软停止方式减速停车，并关闭自动模式。

如果收到 `HOME`，则会先结束当前自动动作，再执行回零流程。

## 4. 项目结构与每个模块的功能

```text
.
├─ platformio.ini
├─ src/
│  └─ main.cpp
├─ lib/
│  ├─ StepperMotion/
│  │  └─ src/
│  │     ├─ StepperMotion.h
│  │     └─ StepperMotion.cpp
│  └─ XAxisController/
│     └─ src/
│        ├─ XAxisController.h
│        └─ XAxisController.cpp
├─ include/
└─ test/
```

### `src/main.cpp`

文件位置：[main.cpp](C:/Users/Admin/Documents/PlatformIO/Projects/test/src/main.cpp)

功能说明：

- 这是整个项目的入口文件
- 只负责创建控制器对象
- 在 `setup()` 中调用初始化
- 在 `loop()` 中持续调用控制器更新函数

也就是说，`main.cpp` 不直接处理运动细节，而是把工作交给控制器库。

### `lib/XAxisController`

文件位置：

- [XAxisController.h](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/XAxisController/src/XAxisController.h)
- [XAxisController.cpp](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/XAxisController/src/XAxisController.cpp)

功能说明：

- 负责 X 轴整体业务控制
- 负责串口命令解析
- 负责自动模式、手动模式、HOME 模式的状态切换
- 负责组织“什么时候动、动多少、为什么停”
- 负责把运动请求交给底层运动库 `StepperMotion`

你可以把它理解成“大脑”。

### `lib/StepperMotion`

文件位置：

- [StepperMotion.h](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/StepperMotion/src/StepperMotion.h)
- [StepperMotion.cpp](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/StepperMotion/src/StepperMotion.cpp)

功能说明：

- 负责真正输出 `STEP` 和 `DIR`
- 负责速度曲线计算
- 负责加速度和 jerk 限制
- 负责在收到中断时执行软停止
- 负责把“要走多少步”转换成实际脉冲序列

你可以把它理解成“电机运动执行器”。

## 5. 硬件配置

当前代码的硬件假设如下：

- 主控板：ESP32
- 扩展板：CNC 板
- 驱动器：A4988
- 电机：1.8° 两相步进电机
- 细分：1/16

根据以上条件：

- 电机整步每圈：`200`
- 微步倍数：`16`
- 每圈总脉冲：`3200`

也就是说：

- `+1` 表示正转 `3200` 个脉冲
- `-1` 表示反转 `3200` 个脉冲
- 默认自动 3 圈就是 `9600` 脉冲

## 6. 引脚定义

当前程序中的引脚定义如下：

- `EN = GPIO12`
- `X_STEP = GPIO26`
- `X_DIR = GPIO16`
- `X_HOME_SWITCH = GPIO13`

逻辑定义如下：

- `EN` 为低电平有效
- `HOME` 限位为低电平触发
- 正转和反转方向由 `FORWARD_DIR_LEVEL`、`REVERSE_DIR_LEVEL` 控制

如果你发现方向和机械预期相反，一般不需要改接线，直接修改：

- `FORWARD_DIR_LEVEL`
- `REVERSE_DIR_LEVEL`

如果限位触发逻辑相反，则检查：

- `HOME_SWITCH_ACTIVE_LEVEL`

## 7. 运动控制逻辑说明

当前程序不是简单的匀速脉冲输出，而是带有限制的平滑加减速控制。

### 控制器层做什么

`XAxisController` 负责这些事情：

- 维护自动往返模式
- 维护手动运动命令队列
- 在手动命令到来时打断自动模式
- 在等待期间继续处理串口输入
- 执行 HOME 的“快找 -> 退回 -> 慢靠”流程

### 运动库层做什么

`StepperMotion` 负责这些事情：

- 从起始速度起步
- 根据当前目标决定是加速、匀速还是减速
- 根据 jerk 限制平滑改变加速度
- 根据剩余步数判断何时提前进入刹车
- 在收到中断请求时执行软停止

这使得启动、刹车和命令打断都会比普通硬切换更平滑。

## 8. 串口命令说明

串口波特率固定为：`115200`

建议串口工具开启“发送换行”或“发送回车”，虽然当前程序已经支持空闲超时自动提交命令，但带换行会更稳定。

### 1. 手动正转

命令格式：

```text
+N
```

含义：

- 中断自动模式
- 等待 1 秒
- 正向转动 `N` 圈

示例：

```text
+1
```

表示正转 1 圈。

```text
+3
```

表示正转 3 圈。

### 2. 手动反转

命令格式：

```text
-N
```

含义：

- 中断自动模式
- 等待 1 秒
- 反向转动 `N` 圈

示例：

```text
-1
```

表示反转 1 圈。

```text
-4
```

表示反转 4 圈。

### 3. 设置巡航速度

命令格式：

```text
Sxxxx
```

含义：

- 设置巡航速度，单位是 `steps/s`

示例：

```text
S800
```

表示把巡航速度设置为 `800 steps/s`。

```text
S1500
```

表示把巡航速度设置为 `1500 steps/s`。

### 4. 设置加速度

命令格式：

```text
Axxxx
```

含义：

- 设置加速度，单位是 `steps/s^2`

示例：

```text
A1200
```

表示把加速度设置为 `1200 steps/s^2`。

```text
A3000
```

表示把加速度设置为 `3000 steps/s^2`。

### 5. 软停止

命令：

```text
STOP
```

作用：

- 让当前运动按减速曲线停车
- 关闭自动模式
- 不会像直接断使能那样硬停

适合在测试过程中安全停机。

### 6. 回零

命令：

```text
HOME
```

作用：

- 关闭自动模式
- 执行 X 轴回零
- 使用“快找 -> 退回 -> 慢靠”的方式提高回零精度
- 回零完成后将软件位置清零

### 7. 开启自动模式

命令：

```text
AUTO ON
```

作用：

- 开启默认自动往返
- 当前自动流程为正转 3 圈、反转 3 圈循环

### 8. 关闭自动模式

命令：

```text
AUTO OFF
```

作用：

- 关闭自动往返模式
- 不主动触发新的自动运动

### 9. 查询状态

命令：

```text
STATUS
```

作用：

- 打印当前自动模式状态
- 打印当前速度、加速度、jerk
- 打印当前位置估计值
- 打印 HOME 状态和限位触发状态

### 10. 查看帮助

命令：

```text
HELP
```

或者：

```text
?
```

作用：

- 打印当前可用命令列表

## 9. 串口调试示例

下面给出一组常见的串口输入示例，方便直接照着测试。

### 示例 1：让电机正转 1 圈

```text
+1
```

### 示例 2：让电机反转 1 圈

```text
-1
```

### 示例 3：先把速度改成 900，再正转 2 圈

```text
S900
+2
```

### 示例 4：把加速度调低一点，观察启动是否更柔和

```text
A1000
```

### 示例 5：运动过程中发停止命令

```text
STOP
```

### 示例 6：关闭自动模式，只保留手动测试

```text
AUTO OFF
```

### 示例 7：重新开启自动模式

```text
AUTO ON
```

### 示例 8：查看当前参数和状态

```text
STATUS
```

### 示例 9：执行回零

```text
HOME
```

## 10. 关键参数说明

以下参数主要位于 [XAxisController.cpp](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/XAxisController/src/XAxisController.cpp)。

- `DEFAULT_AUTO_TURNS`
  说明：默认自动模式每个方向运行的圈数
- `STEP_PULSE_HIGH_US`
  说明：STEP 脉冲高电平宽度
- `MANUAL_COMMAND_PAUSE_MS`
  说明：收到手动命令后，执行前等待时间
- `AUTO_PAUSE_AFTER_FORWARD_MS`
  说明：自动正转结束后的停顿时间
- `AUTO_PAUSE_AFTER_REVERSE_MS`
  说明：自动反转结束后的停顿时间
- `DEFAULT_CRUISE_RATE_SPS`
  说明：默认巡航速度
- `DEFAULT_ACCELERATION_SPS2`
  说明：默认加速度
- `DEFAULT_JERK_RATIO`
  说明：根据加速度推导 jerk 的比例
- `HOME_SEEK_RATE_SPS`
  说明：HOME 快找速度
- `HOME_LATCH_RATE_SPS`
  说明：HOME 慢靠速度
- `HOME_ACCELERATION_SPS2`
  说明：HOME 流程使用的加速度

以下参数主要位于 [StepperMotion.h](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/StepperMotion/src/StepperMotion.h) 和 [StepperMotion.cpp](C:/Users/Admin/Documents/PlatformIO/Projects/test/lib/StepperMotion/src/StepperMotion.cpp)。

- `startRateSps`
  说明：电机起步速度
- `cruiseRateSps`
  说明：巡航速度
- `accelerationSps2`
  说明：加速度
- `jerkSps3`
  说明：加加速度限制，用于让加速度变化更平滑
- `pulseHighUs`
  说明：STEP 高电平保持时间

## 11. 编译、烧录与串口监视

如果命令行已配置好 PlatformIO，可使用：

```bash
platformio run
platformio run -t upload
platformio device monitor -b 115200
```

如果系统里没有全局 `platformio` 命令，也可以使用：

```bash
C:\Users\Admin\.platformio\penv\Scripts\platformio.exe run
C:\Users\Admin\.platformio\penv\Scripts\platformio.exe run -t upload
C:\Users\Admin\.platformio\penv\Scripts\platformio.exe device monitor -b 115200
```

## 12. 常见问题

### 1. 电机锁轴但不转

可能原因：

- 使能接线不对
- `STEP` 没有正确输出
- 驱动器细分或电流设置不正确
- 速度设置过高

建议检查：

- `EN` 是否为低电平有效
- `STEP`、`DIR`、`GND` 是否共地
- A4988 电流是否已经按电机额定值调整
- 先发送 `S500` 降低速度测试

### 2. 电机抖动、啸叫或丢步

可能原因：

- 巡航速度太高
- 加速度太高
- 机械负载过大
- 供电能力不足

建议处理：

- 先尝试 `S600`
- 再尝试 `A800`
- 检查丝杆、联轴器和机械阻力
- 检查驱动器散热和供电

### 3. 方向反了

处理方法：

- 修改 `FORWARD_DIR_LEVEL`
- 修改 `REVERSE_DIR_LEVEL`

一般不需要改业务逻辑。

### 4. HOME 不正常

建议检查：

- 限位开关是否接在 `GPIO13`
- 限位是否为低电平触发
- 回零方向是否和机械方向一致
- 是否需要调整 `HOME_APPROACH_DIR_LEVEL`

### 5. 停止不够平滑

可以尝试：

- 适当降低 `Axxxx`
- 进一步减小 jerk 相关参数
- 降低默认巡航速度
