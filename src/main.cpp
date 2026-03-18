/*
 * 项目入口文件
 *
 * 职责：
 * 1. 创建 X 轴控制器实例
 * 2. 在 setup() 中完成一次初始化
 * 3. 在 loop() 中持续调用控制器更新逻辑
 *
 * 具体的串口命令解析、自动运行、手动控制、HOME 回零、
 * 加减速与软停止等逻辑，都已经下沉到 lib 目录中的控制器与运动库。
 */

#include <Arduino.h>

#include <XAxisController.h>

namespace {
// 控制器全局单例，负责整个 X 轴运行流程。
XAxisController controller;
}

void setup() {
  // 统一由控制器完成串口、引脚和运动参数初始化。
  controller.begin();
}

void loop() {
  // 每次循环都交给控制器处理状态机和运动更新。
  controller.update();
}
//mm

