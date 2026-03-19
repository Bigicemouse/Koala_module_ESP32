/*
 * XAxisController
 *
 * 为了兼容既有工程名，对外仍保留 XAxisController 这个类名，
 * 但内部实际上已经扩展成 X / Z 双轴可切换控制器。
 */

#pragma once

#include <Arduino.h>

#include <StepperMotion.h>

class XAxisController {
 public:
  enum class AxisId : uint8_t {
    X = 0,
    Z = 1,
    Count,
  };

  // 每根轴的静态硬件配置。
  struct AxisConfig {
    const __FlashStringHelper* name;
    const __FlashStringHelper* driverName;
    const __FlashStringHelper* microstepLabel;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t homeSwitchPin;
    bool hasHomeSwitch;
    bool forwardDirLevel;
    bool reverseDirLevel;
    uint32_t stepsPerRev;
  };

  void begin();
  void update();

 private:
  // 当前运动由谁发起，便于在中断时判断处理策略。
  enum class MotionOwner {
    None,
    Auto,
    Manual,
    Home,
  };

  // 每根轴的运行时状态。
  struct AxisRuntime {
    StepperMotion motion;
    float cruiseRateSps = 1000.0f;
    float accelerationSps2 = 1800.0f;
    int32_t pendingManualTurns = 0;
    bool pendingHomeRequest = false;
    bool stopRequested = false;
    bool autoModeEnabled = true;
    bool currentMoveStopsOnHomeSwitch = false;
    bool homeKnown = false;
    long currentPositionSteps = 0;
    MotionOwner currentMotionOwner = MotionOwner::None;
  };

  // 串口命令缓冲区。支持无换行输入，也支持带换行输入。
  char serialBuffer_[32] = {0};
  size_t serialBufferLength_ = 0;
  unsigned long lastSerialByteMs_ = 0;

  // 轴切换采用“空闲后生效”的方式，避免强行打断底层状态。
  AxisId activeAxis_ = AxisId::Z;
  AxisId pendingAxis_ = AxisId::Z;
  AxisId executingAxis_ = AxisId::Z;
  bool axisSwitchPending_ = false;
  AxisRuntime axisRuntime_[static_cast<size_t>(AxisId::Count)];

  static XAxisController* activeInstance_;

  // 给 StepperMotion 的静态桥接回调。
  static bool shouldInterruptBridge();
  static constexpr size_t axisIndex(AxisId axis) {
    return static_cast<size_t>(axis);
  }

  // 轴状态访问辅助函数。
  AxisRuntime& runtimeFor(AxisId axis);
  const AxisRuntime& runtimeFor(AxisId axis) const;
  const AxisConfig& configFor(AxisId axis) const;
  AxisRuntime& activeRuntime();
  const AxisRuntime& activeRuntime() const;
  const AxisConfig& activeConfig() const;
  const __FlashStringHelper* axisName(AxisId axis) const;
  bool parseAxisCommand(const char* command, AxisId* axisOut) const;

  // 中断、限位与服务逻辑。
  bool shouldInterruptCurrentMotion();
  bool isHomeSwitchTriggered(AxisId axis) const;
  bool waitWithService(uint32_t durationMs, bool allowInterrupt);
  bool runHomeApproach(AxisId axis, uint32_t steps, float speedSps, const __FlashStringHelper* label);
  bool runHomeBackoff(AxisId axis, uint32_t steps, float speedSps, const __FlashStringHelper* label);
  void applyPendingAxisSwitchIfIdle();

  // 参数与状态输出。
  void applyMotionProfile(AxisId axis, float cruiseRate, float acceleration);
  void printStartupBanner() const;
  void printSerialHelp() const;
  void printStatus() const;

  // 串口输入处理。
  void pollSerial();
  void processBufferedSerialCommand();
  void handleSerialCommand(char* rawCommand);

  // 外部请求的动作入口。
  void queueManualMove(int32_t turns);
  void requestStop();
  void requestHome();
  void requestAxisSwitch(AxisId axis);
  void clearStopIfIdle();

  // 业务状态机。
  void performManualMoveIfPending();
  void performHomeSequenceIfPending();
  void performAutoCycleIfEnabled();

  // 位置估算与底层运动执行。
  void updatePositionByDirection(uint32_t stepsExecuted, bool dirLevel);
  StepperMoveReport executeMove(AxisId axis,
                                uint32_t steps,
                                bool dirLevel,
                                MotionOwner owner,
                                const __FlashStringHelper* label,
                                bool stopOnHomeSwitch);
};
