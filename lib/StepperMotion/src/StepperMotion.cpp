/*
 * StepperMotion.cpp
 *
 * 实现思路：
 * 1. 用 StepperMotionState 保存当前速度和加速度
 * 2. 每走一步，根据目标加速度和 jerk 限制推进状态
 * 3. 根据剩余步数提前判断是否应进入刹车段
 * 4. 用 STEP 脉冲周期把速度状态转换成实际输出
 *
 * 这样做的好处是：
 * - 启停更柔和
 * - STOP / 手动打断时可以平滑减速
 * - 上层控制器无需关心底层步进节拍细节
 */

#include "StepperMotion.h"

#include <math.h>

namespace {

// 以最大变化量逼近目标值，用来实现 jerk 限制下的加速度变化。
float moveToward(float currentValue, float targetValue, float maxDelta) {
  if (currentValue < targetValue) {
    return min(currentValue + maxDelta, targetValue);
  }
  if (currentValue > targetValue) {
    return max(currentValue - maxDelta, targetValue);
  }
  return currentValue;
}

}  // 匿名命名空间

StepperMotion::StepperMotion() = default;

void StepperMotion::begin(uint8_t stepPin, uint8_t dirPin) {
  // 初始化输出引脚，并确保 STEP 默认保持低电平，避免上电误触发。
  stepPin_ = stepPin;
  dirPin_ = dirPin;

  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  digitalWrite(stepPin_, LOW);
}

void StepperMotion::setPulseHighUs(uint16_t pulseHighUs) {
  // A4988 一类驱动需要一个最小高电平脉宽，这里顺手防御 0 值输入。
  pulseHighUs_ = pulseHighUs > 0 ? pulseHighUs : 1;
}

void StepperMotion::setStartRateSps(float startRateSps) {
  // 起始速度不允许低于 1，且不能高于巡航速度。
  startRateSps_ = startRateSps > 1.0f ? startRateSps : 1.0f;
  if (cruiseRateSps_ < startRateSps_) {
    cruiseRateSps_ = startRateSps_;
  }
}

void StepperMotion::setCruiseRateSps(float cruiseRateSps) {
  // 巡航速度至少要不小于起始速度，否则曲线会失去意义。
  cruiseRateSps_ = cruiseRateSps > startRateSps_ ? cruiseRateSps : startRateSps_;
}

void StepperMotion::setAccelerationSps2(float accelerationSps2) {
  // 加速度同样做基础防御，避免出现非法的 0 或负值。
  accelerationSps2_ = accelerationSps2 > 1.0f ? accelerationSps2 : 1.0f;
}

void StepperMotion::setJerkSps3(float jerkSps3) {
  // jerk 越小越柔和，越大越接近传统硬切换加速度。
  jerkSps3_ = jerkSps3 > 1.0f ? jerkSps3 : 1.0f;
}

uint16_t StepperMotion::getPulseHighUs() const {
  return pulseHighUs_;
}

float StepperMotion::getStartRateSps() const {
  return startRateSps_;
}

float StepperMotion::getCruiseRateSps() const {
  return cruiseRateSps_;
}

float StepperMotion::getAccelerationSps2() const {
  return accelerationSps2_;
}

float StepperMotion::getJerkSps3() const {
  return jerkSps3_;
}

bool StepperMotion::shouldStartBraking(const StepperMotionState& state,
                                       uint32_t remainingSteps) const {
  // 当剩余步数已经不够完成平滑刹车时，必须立刻进入减速段。
  if (remainingSteps <= 1) {
    return true;
  }

  return remainingSteps <= estimateBrakingDistanceSteps(state) + 1U;
}

float StepperMotion::resolveTargetAcceleration(const StepperMotionState& state) const {
  // 根据当前速度与刹车状态，决定“下一步想要靠近的目标加速度”。
  constexpr float kRateToleranceSps = 1.0f;
  constexpr float kAccelToleranceSps2 = 1.0f;

  if (state.braking) {
    return -accelerationSps2_;
  }

  if (state.rateSps < cruiseRateSps_ - kRateToleranceSps) {
    return accelerationSps2_;
  }

  if (state.rateSps > cruiseRateSps_ + kRateToleranceSps) {
    return -accelerationSps2_;
  }

  if (fabsf(state.accelSps2) <= kAccelToleranceSps2) {
    return 0.0f;
  }

  return 0.0f;
}

void StepperMotion::advanceState(StepperMotionState* state,
                                 float targetAccelSps2,
                                 float dtSeconds) const {
  // 先用 jerk 限制推进加速度，再用加速度推进速度。
  const float safeDtSeconds = dtSeconds > 0.0f ? dtSeconds : 0.000001f;
  const float maxAccelDelta = jerkSps3_ * safeDtSeconds;
  state->accelSps2 = moveToward(state->accelSps2, targetAccelSps2, maxAccelDelta);

  const float nextRate = state->rateSps + state->accelSps2 * safeDtSeconds;
  const float maxRate = state->braking ? state->rateSps : cruiseRateSps_;
  state->rateSps = min(max(nextRate, startRateSps_), maxRate);
}

uint32_t StepperMotion::estimateBrakingDistanceSteps(const StepperMotionState& state) const {
  // 这是一个保守估算值，用来尽量提前开始减速，
  // 避免实际运动中刹车点过晚。
  const float safeRate = max(state.rateSps, startRateSps_);
  const float baseDistance =
      max(0.0f, (safeRate * safeRate - startRateSps_ * startRateSps_) /
                    (2.0f * accelerationSps2_));

  const float accelReleaseTime = fabsf(state.accelSps2) / jerkSps3_;
  const float accelReleaseDistance =
      safeRate * accelReleaseTime +
      0.5f * fabsf(state.accelSps2) * accelReleaseTime * accelReleaseTime;

  const float jerkTransitionTime =
      (accelerationSps2_ + fabsf(state.accelSps2)) / jerkSps3_;
  const float jerkTransitionDistance =
      safeRate * jerkTransitionTime +
      0.5f * accelerationSps2_ * jerkTransitionTime * jerkTransitionTime;

  const float safetyMarginSteps = 2.0f;
  return max<uint32_t>(1,
                       static_cast<uint32_t>(ceilf(baseDistance + accelReleaseDistance +
                                                   jerkTransitionDistance +
                                                   safetyMarginSteps)));
}

uint32_t StepperMotion::intervalFromRateUs(float rateSps) const {
  // 把“速度（steps/s）”转换成“单步周期（us）”。
  const float safeRate = rateSps > 1.0f ? rateSps : 1.0f;
  const uint32_t rawIntervalUs = static_cast<uint32_t>(lroundf(1000000.0f / safeRate));
  const uint32_t minIntervalUs = static_cast<uint32_t>(pulseHighUs_) + 1U;
  return rawIntervalUs > minIntervalUs ? rawIntervalUs : minIntervalUs;
}

void StepperMotion::pulseWithIntervalUs(uint32_t stepIntervalUs) const {
  // 输出一个完整的 STEP 脉冲周期。
  const uint32_t lowTimeUs =
      stepIntervalUs > pulseHighUs_ ? stepIntervalUs - pulseHighUs_ : 1;

  digitalWrite(stepPin_, HIGH);
  delayMicroseconds(pulseHighUs_);
  digitalWrite(stepPin_, LOW);
  delayMicroseconds(lowTimeUs);
}

StepperMoveReport StepperMotion::moveSteps(uint32_t steps,
                                           bool dirLevel,
                                           StepperInterruptCallback interruptCallback) {
  // 单次运动流程：
  // 1. 初始化状态
  // 2. 循环输出脉冲
  // 3. 按剩余距离判断是否进入刹车
  // 4. 如果收到中断请求，则切换为软停止模式
  if (steps == 0) {
    return {StepperMoveResult::Completed, 0};
  }

  digitalWrite(dirPin_, dirLevel);

  StepperMotionState state = {
      // 从起始速度起步，初始加速度为 0，由后续状态推进逐渐拉起。
      startRateSps_,
      0.0f,
      false,
  };

  bool softStopped = false;
  uint32_t executedSteps = 0;

  while (executedSteps < steps) {
    const uint32_t remainingSteps = steps - executedSteps;

    if (!state.braking) {
      // 一旦收到中断请求，就切到软停止分支，后续不再恢复加速。
      if (interruptCallback != nullptr && interruptCallback()) {
        state.braking = true;
        softStopped = true;
      } else if (shouldStartBraking(state, remainingSteps)) {
        state.braking = true;
      }
    }

    const float currentRateSps = max(state.rateSps, startRateSps_);
    pulseWithIntervalUs(intervalFromRateUs(currentRateSps));
    ++executedSteps;

    const float dtSeconds = 1.0f / currentRateSps;
    const float targetAccelSps2 = resolveTargetAcceleration(state);
    advanceState(&state, targetAccelSps2, dtSeconds);
  }

  return {
      softStopped ? StepperMoveResult::SoftStopped : StepperMoveResult::Completed,
      executedSteps,
  };
}
