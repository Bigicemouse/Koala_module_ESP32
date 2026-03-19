#pragma once

#include <Arduino.h>

namespace AxisProfile {

// 步进电机常见为 200 个整步一圈，X/Z 轴都以这个基准换算细分。
constexpr uint32_t kFullStepsPerRev = 200U;

// X 轴沿用当前项目的 1/16 细分配置。
constexpr uint32_t kXMicrosteps = 16U;

// Z 轴使用当前硬件组合的“实测有效细分”。
// 虽然 2209 模块图片表会让人联想到更高细分，
// 但在这套 CNC Shield + TMC2209 接法上，实测更接近 1/16。
constexpr uint32_t kZMicrosteps = 16U;

constexpr uint32_t kXStepsPerRev = kFullStepsPerRev * kXMicrosteps;
constexpr uint32_t kZStepsPerRev = kFullStepsPerRev * kZMicrosteps;

// 正式控制器默认以 X 轴为基准速度，Z 轴再根据每圈步数按比例继承。
constexpr float kXDefaultCruiseRateSps = 1000.0f;
constexpr float kXDefaultAccelerationSps2 = 1800.0f;

// 诊断模式故意把 Z 轴调慢，让用户可以直接用肉眼确认正反转。
constexpr float kZDiagnosticTurnSeconds = 7.0f;
constexpr uint16_t kDiagnosticStepHighUs = 10U;
constexpr uint32_t kDiagnosticStepPeriodUs =
    static_cast<uint32_t>((kZDiagnosticTurnSeconds * 1000000.0f) / kZStepsPerRev);
constexpr uint32_t kDiagnosticStepLowUs =
    kDiagnosticStepPeriodUs > kDiagnosticStepHighUs
        ? (kDiagnosticStepPeriodUs - kDiagnosticStepHighUs)
        : 1U;

// 切换方向后预留建立时间，避免驱动器来不及识别 DIR 电平。
constexpr uint16_t kDiagnosticDirSettleMs = 300U;

// 自动往复模式在每段动作之间留一点停顿，方便观察状态变化。
constexpr uint16_t kDiagnosticAutoPauseMs = 1200U;

}  // namespace AxisProfile
