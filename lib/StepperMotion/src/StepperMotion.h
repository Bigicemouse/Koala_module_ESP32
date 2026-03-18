/*
 * StepperMotion
 *
 * 这是底层步进运动库，职责很单纯：
 * 1. 根据当前速度、加速度、jerk 参数生成步进节拍
 * 2. 输出 STEP / DIR 脉冲
 * 3. 在收到中断请求时按软停止逻辑减速停车
 *
 * 它不关心 AUTO、HOME、串口命令等上层业务，只专注于“怎么走”。
 */

#pragma once

#include <Arduino.h>

enum class StepperMoveResult {
  Completed,
  SoftStopped,
};

// 一次运动完成后的摘要结果。
struct StepperMoveReport {
  StepperMoveResult result;
  uint32_t stepsExecuted;
};

using StepperInterruptCallback = bool (*)();

// 运动内部状态：当前速度、当前加速度，以及是否进入刹车段。
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

  // 一组参数设置接口，对应步进脉冲宽度与运动学约束。
  void setPulseHighUs(uint16_t pulseHighUs);
  void setStartRateSps(float startRateSps);
  void setCruiseRateSps(float cruiseRateSps);
  void setAccelerationSps2(float accelerationSps2);
  void setJerkSps3(float jerkSps3);

  // 读取当前配置，便于上层状态输出和调试。
  uint16_t getPulseHighUs() const;
  float getStartRateSps() const;
  float getCruiseRateSps() const;
  float getAccelerationSps2() const;
  float getJerkSps3() const;

  // 执行一次固定步数的运动，可在过程中响应中断回调。
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

  // 速度规划与状态推进辅助函数。
  bool shouldStartBraking(const StepperMotionState& state, uint32_t remainingSteps) const;
  float resolveTargetAcceleration(const StepperMotionState& state) const;
  void advanceState(StepperMotionState* state, float targetAccelSps2, float dtSeconds) const;
  uint32_t estimateBrakingDistanceSteps(const StepperMotionState& state) const;

  // 硬件脉冲输出辅助函数。
  uint32_t intervalFromRateUs(float rateSps) const;
  void pulseWithIntervalUs(uint32_t stepIntervalUs) const;
};
