/*
 * StepperMotion
 *
 * 底层步进脉冲驱动层，只负责三件事：
 * 1. 根据速度、加速度、jerk 参数推进运动状态
 * 2. 输出 STEP / DIR 脉冲
 * 3. 在收到中断请求时按“软停”方式减速结束
 *
 * 它不关心串口命令、AUTO 模式、HOME 流程等上层业务。
 */

#pragma once

#include <Arduino.h>

enum class StepperMoveResult {
  Completed,
  SoftStopped,
};

// 一次运动结束后的结果摘要。
struct StepperMoveReport {
  StepperMoveResult result;
  uint32_t stepsExecuted;
};

using StepperInterruptCallback = bool (*)();

// 运动状态会在每一步之后推进一次。
struct StepperMotionState {
  float rateSps;
  float accelSps2;
  bool braking;
};

class StepperMotion {
 public:
  StepperMotion();

  // 初始化 STEP / DIR 引脚。
  void begin(uint8_t stepPin, uint8_t dirPin);

  // 设置步进脉冲宽度与运动学参数。
  void setPulseHighUs(uint16_t pulseHighUs);
  void setStartRateSps(float startRateSps);
  void setCruiseRateSps(float cruiseRateSps);
  void setAccelerationSps2(float accelerationSps2);
  void setJerkSps3(float jerkSps3);

  // 读取当前配置，便于上层打印状态和调试。
  uint16_t getPulseHighUs() const;
  float getStartRateSps() const;
  float getCruiseRateSps() const;
  float getAccelerationSps2() const;
  float getJerkSps3() const;

  // 执行固定步数运动，可选支持中断回调触发软停。
  StepperMoveReport moveSteps(uint32_t steps,
                              bool dirLevel,
                              StepperInterruptCallback interruptCallback = nullptr);

 private:
  // 硬件引脚与当前运动参数。
  uint8_t stepPin_ = 0;
  uint8_t dirPin_ = 0;
  uint16_t pulseHighUs_ = 10;
  float startRateSps_ = 200.0f;
  float cruiseRateSps_ = 1000.0f;
  float accelerationSps2_ = 1800.0f;
  float jerkSps3_ = 21600.0f;

  // 状态推进与刹车距离估算。
  bool shouldStartBraking(const StepperMotionState& state, uint32_t remainingSteps) const;
  float resolveTargetAcceleration(const StepperMotionState& state) const;
  void advanceState(StepperMotionState* state, float targetAccelSps2, float dtSeconds) const;
  uint32_t estimateBrakingDistanceSteps(const StepperMotionState& state) const;

  // 硬件脉冲输出辅助函数。
  uint32_t intervalFromRateUs(float rateSps) const;
  void pulseWithIntervalUs(uint32_t stepIntervalUs) const;
  void applyDirectionLevel(bool dirLevel) const;
};
