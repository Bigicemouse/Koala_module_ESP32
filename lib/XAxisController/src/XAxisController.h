/*
 * XAxisController
 *
 * 这是上层业务控制器，负责把“串口命令”和“运动执行”连接起来。
 *
 * 主要职责：
 * 1. 初始化 X 轴相关引脚和运动参数
 * 2. 解析串口命令，如 +N / -N / Sxxxx / Axxxx / STOP / HOME
 * 3. 维护 AUTO、手动、HOME 三类运行状态
 * 4. 调用 StepperMotion 库执行具体的步进脉冲输出
 */

#pragma once

#include <Arduino.h>

#include <StepperMotion.h>

class XAxisController {
 public:
  // 初始化控制器、串口、引脚和默认运动参数。
  void begin();

  // 主循环更新入口，每次 loop() 都应调用一次。
  void update();

 private:
  // 当前运动所属场景，用于决定中断策略。
  enum class MotionOwner {
    None,
    Auto,
    Manual,
    Home,
  };

  // 底层运动执行器，真正负责输出脉冲和速度曲线。
  StepperMotion xAxisMotion_;

  // 串口接收缓冲区，支持逐字节拼装命令。
  char serialBuffer_[32] = {0};
  size_t serialBufferLength_ = 0;
  unsigned long lastSerialByteMs_ = 0;

  // 当前生效的运行参数。
  float cruiseRateSps_ = 1000.0f;
  float accelerationSps2_ = 1800.0f;

  // 上层状态位。
  int32_t pendingManualTurns_ = 0;
  bool pendingHomeRequest_ = false;
  bool stopRequested_ = false;
  bool autoModeEnabled_ = true;
  bool currentMoveStopsOnHomeSwitch_ = false;
  bool homeKnown_ = false;

  long currentPositionSteps_ = 0;
  MotionOwner currentMotionOwner_ = MotionOwner::None;

  // 由于底层运动库使用函数指针回调，这里保留当前活动实例用于桥接。
  static XAxisController* activeInstance_;

  static bool shouldInterruptBridge();

  // 运动执行期的中断判定逻辑。
  bool shouldInterruptCurrentMotion();
  bool isHomeSwitchTriggered() const;
  bool waitWithService(uint32_t durationMs, bool allowInterrupt);
  bool runHomeApproach(uint32_t steps, float speedSps, const __FlashStringHelper* label);
  bool runHomeBackoff(uint32_t steps, float speedSps, const __FlashStringHelper* label);

  void applyMotionProfile(float cruiseRate, float acceleration);
  void printStartupBanner() const;
  void printSerialHelp() const;
  void printStatus() const;

  // 串口处理链：接收字节 -> 组装命令 -> 解析命令。
  void pollSerial();
  void processBufferedSerialCommand();
  void handleSerialCommand(char* rawCommand);

  // 外部命令对应的状态切换。
  void queueManualMove(int32_t turns);
  void requestStop();
  void requestHome();
  void clearStopIfIdle();

  // 两类高层动作：手动运动和回零流程。
  void performManualMoveIfPending();
  void performHomeSequenceIfPending();

  // 运动结果回写与统一执行入口。
  void updatePositionByDirection(uint32_t stepsExecuted, bool dirLevel);
  StepperMoveReport executeMove(uint32_t steps,
                                bool dirLevel,
                                MotionOwner owner,
                                const __FlashStringHelper* label,
                                bool stopOnHomeSwitch);
};
