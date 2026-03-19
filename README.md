# ESP32 X/Z Axis Controller

当前项目已经根据实测结果修正了 `Z` 轴细分认知。

## 当前结论

- `Z` 轴驱动板是 `TMC2209`，插在 `CNC Shield` 的 `Z` 轴位
- `Z_STEP = GPIO17`
- `Z_DIR = GPIO14`
- `EN-GND` 跳线保持常使能
- 用户实测当前硬件组合下，`Z` 轴 `1 圈 ≈ 1 秒`
- 这与 `1/128` 细分不符，更接近 `1/16`

因此本项目现在以 **实测有效细分** 为准：

- `Z` 轴有效细分：`1/16`
- `Z` 轴每圈步数：`3200`

## 为什么不是继续按 1/128

你提供的驱动图片表显示：

- `MS1 = OFF`
- `MS2 = ON`
- `MS3 = ON`

按表理解应当是 `1/128`。

但实际测试中：

- 诊断固件按 `1/128` 计算时，理论上一圈应接近 `8 秒`
- 真实硬件却只有大约 `1 秒` 一圈

所以对这块板子插在当前 `CNC Shield` 上的实际效果，不能只看图片表，必须以实测为准。

## 构建模式

项目现在提供两种编译环境。

### 1. 正式控制器模式

环境名：

- `esp32`

作用：

- 使用 `X/Z` 双轴控制器
- 默认活动轴是 `Z`
- 默认 `AUTO` 为 `Z` 轴 `+3 / -3`
- `Z` 轴按有效 `1/16` 细分计算

### 2. Z 轴诊断模式

环境名：

- `esp32_diag`

作用：

- 只跑 `Z_STEP / Z_DIR`
- 用来验证方向切换、停机、圈时
- 默认目标为肉眼容易观察的低速

## 当前共享轴参数

共享参数定义在：

- [AxisProfile.h](c:\Users\Admin\Documents\PlatformIO\Projects\Koala_module_ESP32\include\AxisProfile.h)

当前关键值：

- `X` 轴细分：`1/16`
- `Z` 轴有效细分：`1/16`
- `X` 轴每圈步数：`3200`
- `Z` 轴每圈步数：`3200`

## 正式模式说明

正式模式入口：

- [main.cpp](c:\Users\Admin\Documents\PlatformIO\Projects\Koala_module_ESP32\src\main.cpp)
- [XAxisController.cpp](c:\Users\Admin\Documents\PlatformIO\Projects\Koala_module_ESP32\lib\XAxisController\src\XAxisController.cpp)

当前 `Z` 轴默认参数：

- 默认速度：`1000 steps/s`
- 默认加速度：`1800 steps/s^2`

常用串口命令：

```text
AXIS X
AXIS Z
AXIS?
+1
-1
Sxxxx
Axxxx
AUTO ON
AUTO OFF
STOP
STATUS
HELP
```

## 诊断模式说明

诊断模式入口：

- [main.cpp](c:\Users\Admin\Documents\PlatformIO\Projects\Koala_module_ESP32\src\main.cpp)

诊断模式命令：

```text
A
H
L
S
P
?
```

含义：

- `A`：打开/关闭自动往复
- `H`：`DIR=HIGH` 转 1 圈
- `L`：`DIR=LOW` 转 1 圈
- `S`：停止当前动作
- `P`：打印状态
- `?`：打印帮助

诊断模式默认目标：

- 按 `1/16` 计算
- 一圈大约 `7 秒`
- 便于肉眼确认反向是否正常

## 编译方式

编译正式控制器：

```bash
platformio run -e esp32
```

编译 Z 轴诊断模式：

```bash
platformio run -e esp32_diag
```

如果系统没有全局 `platformio`，也可以用：

```bash
C:\Users\Admin\.platformio\penv\Scripts\platformio.exe run -e esp32
C:\Users\Admin\.platformio\penv\Scripts\platformio.exe run -e esp32_diag
```

## 现在怎么用

如果你要恢复正式控制器：

- 选择 `env:esp32`

如果你要继续只测 `Z` 轴方向和圈时：

- 选择 `env:esp32_diag`

## 备注

- 当前 README 是按 **实测有效细分优先** 记录的
- 如果你后面逐个跳线重新测出新的有效细分，再改 [AxisProfile.h](c:\Users\Admin\Documents\PlatformIO\Projects\Koala_module_ESP32\include\AxisProfile.h) 即可同步到正式模式和诊断模式
