/*
 * StepperMotion.cpp
 *
 * 这一层把“目标运动学参数”转换成真正的 STEP 脉冲：
 * 1. 用状态对象保存当前速度与加速度
 * 2. 每走一步后按 jerk 限制推进状态
 * 3. 根据剩余步数决定何时进入减速段
 * 4. 如收到中断请求，则切换到软停模式
 */

#include "StepperMotion.h"

#include <math.h>

namespace {

// 切换方向后给驱动器一点建立时间，避免 DIR 还没稳定就开始 STEP。
constexpr uint16_t DIR_SETUP_US = 2000;

// 按最大变化量逼近目标值，用来实现 jerk 限制下的平滑过渡。
float moveToward(float currentValue, float targetValue, float maxDelta) {
  if (currentValue < targetValue) {
    return min(currentValue + maxDelta, targetValue);
  }
  if (currentValue > targetValue) {
    return max(currentValue - maxDelta, targetValue);
  }
  return currentValue;
}

}  // namespace

StepperMotion::StepperMotion() = default;

void StepperMotion::begin(uint8_t stepPin, uint8_t dirPin) {
  stepPin_ = stepPin;
  dirPin_ = dirPin;

  // 上电时先把 STEP 拉低，尽量避免误触发。
  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  digitalWrite(stepPin_, LOW);
  digitalWrite(dirPin_, LOW);
}

void StepperMotion::setPulseHighUs(uint16_t pulseHighUs) {
  // 脉冲高电平宽度至少保持 1us，避免出现非法 0 值。
  pulseHighUs_ = pulseHighUs > 0 ? pulseHighUs : 1;
}

void StepperMotion::setStartRateSps(float startRateSps) {
  // 起步速度不能低于 1 steps/s，也不能高于巡航速度。
  startRateSps_ = startRateSps > 1.0f ? startRateSps : 1.0f;
  if (cruiseRateSps_ < startRateSps_) {
    cruiseRateSps_ = startRateSps_;
  }
}

void StepperMotion::setCruiseRateSps(float cruiseRateSps) {
  // 巡航速度最少要不小于起步速度。
  cruiseRateSps_ = cruiseRateSps > startRateSps_ ? cruiseRateSps : startRateSps_;
}

void StepperMotion::setAccelerationSps2(float accelerationSps2) {
  accelerationSps2_ = accelerationSps2 > 1.0f ? accelerationSps2 : 1.0f;
}

void StepperMotion::setJerkSps3(float jerkSps3) {
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
  // 剩余步数已经不够完成平滑刹车时，就必须立刻进入减速段。
  if (remainingSteps <= 1) {
    return true;
  }

  return remainingSteps <= estimateBrakingDistanceSteps(state) + 1U;
}

float StepperMotion::resolveTargetAcceleration(const StepperMotionState& state) const {
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
  // 先限制加速度变化率，再用新的加速度更新速度。
  const float safeDtSeconds = dtSeconds > 0.0f ? dtSeconds : 0.000001f;
  const float maxAccelDelta = jerkSps3_ * safeDtSeconds;
  state->accelSps2 = moveToward(state->accelSps2, targetAccelSps2, maxAccelDelta);

  const float nextRate = state->rateSps + state->accelSps2 * safeDtSeconds;
  const float maxRate = state->braking ? state->rateSps : cruiseRateSps_;
  const float minRate = state->braking ? 0.0f : startRateSps_;
  state->rateSps = min(max(nextRate, minRate), maxRate);
}

uint32_t StepperMotion::estimateBrakingDistanceSteps(const StepperMotionState& state) const {
  // 这里做保守估算，宁可稍早刹车，也不要因为判断过晚而冲过目标点。
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
  // 把速度（steps/s）换算成单步周期（us）。
  const float safeRate = rateSps > 1.0f ? rateSps : 1.0f;
  const uint32_t rawIntervalUs = static_cast<uint32_t>(lroundf(1000000.0f / safeRate));
  const uint32_t minIntervalUs = static_cast<uint32_t>(pulseHighUs_) + 1U;
  return rawIntervalUs > minIntervalUs ? rawIntervalUs : minIntervalUs;
}

void StepperMotion::pulseWithIntervalUs(uint32_t stepIntervalUs) const {
  const uint32_t lowTimeUs =
      stepIntervalUs > pulseHighUs_ ? stepIntervalUs - pulseHighUs_ : 1U;

  digitalWrite(stepPin_, HIGH);
  delayMicroseconds(pulseHighUs_);
  digitalWrite(stepPin_, LOW);
  delayMicroseconds(lowTimeUs);
}

void StepperMotion::applyDirectionLevel(bool dirLevel) const {
  digitalWrite(dirPin_, dirLevel);
  delayMicroseconds(DIR_SETUP_US);
}

StepperMoveReport StepperMotion::moveSteps(uint32_t steps,
                                           bool dirLevel,
                                           StepperInterruptCallback interruptCallback) {
  if (steps == 0) {
    return {StepperMoveResult::Completed, 0};
  }

  applyDirectionLevel(dirLevel);

  StepperMotionState state = {
      startRateSps_,
      0.0f,
      false,
  };

  bool softStopped = false;
  uint32_t executedSteps = 0;

  while (executedSteps < steps) {
    if (softStopped && state.rateSps <= startRateSps_) {
      break;
    }

    const uint32_t remainingSteps = steps - executedSteps;

    if (!state.braking) {
      const bool interrupted = interruptCallback != nullptr && interruptCallback();
      if (interrupted) {
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
