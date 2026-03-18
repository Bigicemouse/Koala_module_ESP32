/*
 * XAxisController.cpp
 *
 * 本文件实现上层控制逻辑：
 * - 管理串口命令
 * - 管理 AUTO / 手动 / HOME 状态机
 * - 调用 StepperMotion 执行实际运动
 *
 * 可以把它理解为“业务层控制器”，而不是直接驱动电机的底层模块。
 */

#include "XAxisController.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

// ---------------- 硬件映射 ----------------
constexpr uint8_t EN_PIN = 12;
constexpr uint8_t X_STEP_PIN = 26;
constexpr uint8_t X_DIR_PIN = 16;
constexpr uint8_t X_HOME_SWITCH_PIN = 13;

constexpr bool ENABLE_ACTIVE_LEVEL = LOW;
constexpr bool HOME_SWITCH_ACTIVE_LEVEL = LOW;
constexpr bool FORWARD_DIR_LEVEL = HIGH;
constexpr bool REVERSE_DIR_LEVEL = LOW;
constexpr bool HOME_APPROACH_DIR_LEVEL = REVERSE_DIR_LEVEL;
constexpr bool HOME_BACKOFF_DIR_LEVEL = FORWARD_DIR_LEVEL;

// ---------------- 默认运动参数 ----------------
constexpr uint32_t STEPS_PER_REV = 3200;
constexpr uint8_t DEFAULT_AUTO_TURNS = 3;
constexpr uint32_t DEFAULT_AUTO_STEPS = STEPS_PER_REV * DEFAULT_AUTO_TURNS;

constexpr uint16_t STEP_PULSE_HIGH_US = 10;
constexpr uint16_t STARTUP_STABILIZE_MS = 1000;
constexpr uint16_t MANUAL_COMMAND_PAUSE_MS = 1000;
constexpr uint16_t AUTO_PAUSE_AFTER_FORWARD_MS = 1000;
constexpr uint16_t AUTO_PAUSE_AFTER_REVERSE_MS = 1000;
constexpr uint16_t IDLE_SERVICE_DELAY_MS = 10;
constexpr uint16_t SERIAL_COMMAND_IDLE_TIMEOUT_MS = 30;

// ---------------- 速度、加速度、Jerk 限幅 ----------------
constexpr float DEFAULT_CRUISE_RATE_SPS = 1000.0f;
constexpr float DEFAULT_ACCELERATION_SPS2 = 1800.0f;
constexpr float DEFAULT_JERK_RATIO = 12.0f;
constexpr float MIN_START_RATE_SPS = 200.0f;
constexpr float START_RATE_RATIO = 0.35f;
constexpr float MIN_CRUISE_RATE_SPS = 100.0f;
constexpr float MAX_CRUISE_RATE_SPS = 20000.0f;
constexpr float MIN_ACCELERATION_SPS2 = 100.0f;
constexpr float MAX_ACCELERATION_SPS2 = 50000.0f;
constexpr float MIN_JERK_SPS3 = 500.0f;
constexpr float MAX_JERK_SPS3 = 500000.0f;

// ---------------- HOME 相关参数 ----------------
constexpr float HOME_SEEK_RATE_SPS = 400.0f;
constexpr float HOME_LATCH_RATE_SPS = 200.0f;
constexpr float HOME_ACCELERATION_SPS2 = 800.0f;
constexpr uint32_t HOME_SEEK_MAX_STEPS = STEPS_PER_REV * 20;
constexpr uint32_t HOME_BACKOFF_STEPS = 200;
constexpr uint32_t HOME_LATCH_MAX_STEPS = 400;

// 通用浮点限幅工具，用于保护串口调参输入。
float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

// 根据巡航速度自动推导一个更柔和的起始速度。
float computeStartRateSps(float cruiseRate) {
  const float derivedRate = cruiseRate * START_RATE_RATIO;
  if (derivedRate < MIN_START_RATE_SPS) {
    return min(cruiseRate, MIN_START_RATE_SPS);
  }
  return min(cruiseRate, derivedRate);
}

// 去掉命令头尾空白，方便串口命令更宽容地匹配。
void trimCommandInPlace(char* command) {
  size_t start = 0;
  while (command[start] != '\0' &&
         isspace(static_cast<unsigned char>(command[start])) != 0) {
    ++start;
  }

  if (start > 0) {
    memmove(command, command + start, strlen(command + start) + 1);
  }

  size_t length = strlen(command);
  while (length > 0 &&
         isspace(static_cast<unsigned char>(command[length - 1])) != 0) {
    command[length - 1] = '\0';
    --length;
  }
}

// 串口命令统一转大写，避免大小写差异影响识别。
void upperCaseAsciiInPlace(char* command) {
  for (size_t i = 0; command[i] != '\0'; ++i) {
    command[i] = static_cast<char>(toupper(static_cast<unsigned char>(command[i])));
  }
}

// 允许 "STOP"、'STOP' 这类带包裹引号的串口输入。
void stripWrappingQuotesInPlace(char* command) {
  const size_t length = strlen(command);
  if (length < 2) {
    return;
  }

  const char first = command[0];
  const char last = command[length - 1];
  const bool wrappedWithDoubleQuotes = first == '"' && last == '"';
  const bool wrappedWithSingleQuotes = first == '\'' && last == '\'';

  if (!wrappedWithDoubleQuotes && !wrappedWithSingleQuotes) {
    return;
  }

  memmove(command, command + 1, length - 2);
  command[length - 2] = '\0';
}

// 解析 +N / -N 形式的圈数命令。
bool parseSignedTurns(const char* command, int32_t* turnsOut) {
  if (command[0] != '+' && command[0] != '-') {
    return false;
  }

  char* endPtr = nullptr;
  const long parsedValue = strtol(command, &endPtr, 10);
  if (endPtr == command || *endPtr != '\0' || parsedValue == 0) {
    return false;
  }

  if (parsedValue < -1000L || parsedValue > 1000L) {
    return false;
  }

  *turnsOut = static_cast<int32_t>(parsedValue);
  return true;
}

// 解析 Sxxxx / Axxxx 这类前缀命令，并做数值范围保护。
bool parsePositiveValueCommand(const char* command,
                               char prefix,
                               float minValue,
                               float maxValue,
                               float* valueOut) {
  if (command[0] != prefix) {
    return false;
  }

  char* endPtr = nullptr;
  const long parsedValue = strtol(command + 1, &endPtr, 10);
  if (endPtr == command + 1 || *endPtr != '\0' || parsedValue <= 0) {
    return false;
  }

  *valueOut = clampFloat(static_cast<float>(parsedValue), minValue, maxValue);
  return true;
}

}  // 匿名命名空间

XAxisController* XAxisController::activeInstance_ = nullptr;

void XAxisController::begin() {
  // 注册当前实例，便于底层回调反向访问控制器状态。
  activeInstance_ = this;

  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(X_HOME_SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(EN_PIN, ENABLE_ACTIVE_LEVEL);

  xAxisMotion_.begin(X_STEP_PIN, X_DIR_PIN);
  applyMotionProfile(DEFAULT_CRUISE_RATE_SPS, DEFAULT_ACCELERATION_SPS2);

  printStartupBanner();
  delay(STARTUP_STABILIZE_MS);
}

void XAxisController::update() {
  // 控制循环的优先级顺序：
  // 1. 先处理串口与停止状态
  // 2. 再处理 HOME
  // 3. 再处理手动命令
  // 4. 最后才进入自动往返
  pollSerial();
  clearStopIfIdle();

  performHomeSequenceIfPending();
  clearStopIfIdle();

  performManualMoveIfPending();
  clearStopIfIdle();

  if (!autoModeEnabled_) {
    delay(IDLE_SERVICE_DELAY_MS);
    return;
  }

  if (executeMove(DEFAULT_AUTO_STEPS,
                  FORWARD_DIR_LEVEL,
                  MotionOwner::Auto,
                  F("auto forward"),
                  false)
          .result == StepperMoveResult::SoftStopped) {
    return;
  }

  if (!waitWithService(AUTO_PAUSE_AFTER_FORWARD_MS, true)) {
    return;
  }

  if (executeMove(DEFAULT_AUTO_STEPS,
                  REVERSE_DIR_LEVEL,
                  MotionOwner::Auto,
                  F("auto reverse"),
                  false)
          .result == StepperMoveResult::SoftStopped) {
    return;
  }

  waitWithService(AUTO_PAUSE_AFTER_REVERSE_MS, true);
}

bool XAxisController::shouldInterruptBridge() {
  return activeInstance_ != nullptr && activeInstance_->shouldInterruptCurrentMotion();
}

bool XAxisController::shouldInterruptCurrentMotion() {
  // 运动执行过程中会频繁调用这里，
  // 用来判断当前动作是否需要被停止、打断或切换。
  pollSerial();

  if (currentMoveStopsOnHomeSwitch_ && isHomeSwitchTriggered()) {
    return true;
  }

  if (stopRequested_) {
    return true;
  }

  switch (currentMotionOwner_) {
    case MotionOwner::Auto:
      return !autoModeEnabled_ || pendingManualTurns_ != 0 || pendingHomeRequest_;
    case MotionOwner::Manual:
      return pendingHomeRequest_;
    case MotionOwner::Home:
      return false;
    case MotionOwner::None:
    default:
      return false;
  }
}

bool XAxisController::isHomeSwitchTriggered() const {
  return digitalRead(X_HOME_SWITCH_PIN) == HOME_SWITCH_ACTIVE_LEVEL;
}

bool XAxisController::waitWithService(uint32_t durationMs, bool allowInterrupt) {
  // 等待期间依旧持续处理串口，这样停顿阶段也能响应命令。
  const unsigned long startMs = millis();

  while (millis() - startMs < durationMs) {
    pollSerial();

    if (allowInterrupt &&
        (stopRequested_ || pendingHomeRequest_ || pendingManualTurns_ != 0 ||
         !autoModeEnabled_)) {
      return false;
    }

    delay(IDLE_SERVICE_DELAY_MS);
  }

  return true;
}

bool XAxisController::runHomeApproach(uint32_t steps,
                                      float speedSps,
                                      const __FlashStringHelper* label) {
  // HOME 探测阶段会临时切换为更保守的速度和加速度。
  const float savedCruiseRate = cruiseRateSps_;
  const float savedAcceleration = accelerationSps2_;

  applyMotionProfile(speedSps, HOME_ACCELERATION_SPS2);
  const StepperMoveReport report =
      executeMove(steps, HOME_APPROACH_DIR_LEVEL, MotionOwner::Home, label, true);
  applyMotionProfile(savedCruiseRate, savedAcceleration);

  if (report.result == StepperMoveResult::SoftStopped && isHomeSwitchTriggered()) {
    return true;
  }

  if (stopRequested_) {
    Serial.println(F("HOME sequence stopped by STOP command."));
  } else {
    Serial.println(F("HOME sequence failed to detect the limit switch."));
  }

  return false;
}

bool XAxisController::runHomeBackoff(uint32_t steps,
                                     float speedSps,
                                     const __FlashStringHelper* label) {
  // 回零释放阶段同样使用 HOME 专用运动参数，结束后恢复原配置。
  const float savedCruiseRate = cruiseRateSps_;
  const float savedAcceleration = accelerationSps2_;

  applyMotionProfile(speedSps, HOME_ACCELERATION_SPS2);
  const StepperMoveReport report =
      executeMove(steps, HOME_BACKOFF_DIR_LEVEL, MotionOwner::Home, label, false);
  applyMotionProfile(savedCruiseRate, savedAcceleration);

  return report.result == StepperMoveResult::Completed && !stopRequested_;
}

void XAxisController::applyMotionProfile(float cruiseRate, float acceleration) {
  // 这里统一管理控制器与底层运动库的参数同步，
  // jerk 则根据 acceleration 自动推导。
  cruiseRateSps_ = clampFloat(cruiseRate, MIN_CRUISE_RATE_SPS, MAX_CRUISE_RATE_SPS);
  accelerationSps2_ =
      clampFloat(acceleration, MIN_ACCELERATION_SPS2, MAX_ACCELERATION_SPS2);

  xAxisMotion_.setPulseHighUs(STEP_PULSE_HIGH_US);
  xAxisMotion_.setCruiseRateSps(cruiseRateSps_);
  xAxisMotion_.setStartRateSps(computeStartRateSps(cruiseRateSps_));
  xAxisMotion_.setAccelerationSps2(accelerationSps2_);
  xAxisMotion_.setJerkSps3(
      clampFloat(accelerationSps2_ * DEFAULT_JERK_RATIO, MIN_JERK_SPS3, MAX_JERK_SPS3));
}

void XAxisController::printStartupBanner() const {
  // 上电时输出一次当前配置，便于确认接线、默认模式和运动参数。
  Serial.println(F("=== ESP32 CNC Shield A4988 X-Axis Controller ==="));
  Serial.println(F("Default mode: AUTO 3 rev forward + 3 rev reverse"));
  Serial.println(F("Manual commands interrupt AUTO with soft stop"));
  Serial.print(F("Pins: EN="));
  Serial.print(EN_PIN);
  Serial.print(F(", STEP="));
  Serial.print(X_STEP_PIN);
  Serial.print(F(", DIR="));
  Serial.print(X_DIR_PIN);
  Serial.print(F(", HOME_SW="));
  Serial.println(X_HOME_SWITCH_PIN);
  Serial.print(F("Steps per revolution: "));
  Serial.println(STEPS_PER_REV);
  Serial.print(F("Cruise speed: "));
  Serial.print(cruiseRateSps_, 1);
  Serial.println(F(" steps/s"));
  Serial.print(F("Acceleration: "));
  Serial.print(accelerationSps2_, 1);
  Serial.println(F(" steps/s^2"));
  Serial.print(F("Jerk: "));
  Serial.print(xAxisMotion_.getJerkSps3(), 1);
  Serial.println(F(" steps/s^3"));
  Serial.print(F("Driver enabled level: "));
  Serial.println(ENABLE_ACTIVE_LEVEL == LOW ? F("LOW") : F("HIGH"));
  printSerialHelp();
}

void XAxisController::printSerialHelp() const {
  // 帮助文本集中放在这里，方便后续继续扩展串口协议。
  Serial.println(F("Serial commands:"));
  Serial.println(F("  +N      -> stop auto, wait 1s, move forward N revolutions"));
  Serial.println(F("  -N      -> stop auto, wait 1s, move reverse N revolutions"));
  Serial.println(F("  Sxxxx   -> set cruise speed in steps/second"));
  Serial.println(F("  Axxxx   -> set acceleration in steps/second^2"));
  Serial.println(F("  STOP    -> soft stop current motion and disable AUTO"));
  Serial.println(F("  HOME    -> run X-axis homing sequence and disable AUTO"));
  Serial.println(F("  AUTO ON -> enable auto 3-rev round trip"));
  Serial.println(F("  AUTO OFF-> disable auto mode"));
  Serial.println(F("  STATUS  -> print runtime status"));
  Serial.println(F("  HELP/?  -> print this help"));
}

void XAxisController::printStatus() const {
  // 运行时状态快照，主要用于串口调试和参数确认。
  Serial.println(F("=== Status ==="));
  Serial.print(F("AUTO mode: "));
  Serial.println(autoModeEnabled_ ? F("ON") : F("OFF"));
  Serial.print(F("Cruise speed: "));
  Serial.print(cruiseRateSps_, 1);
  Serial.println(F(" steps/s"));
  Serial.print(F("Acceleration: "));
  Serial.print(accelerationSps2_, 1);
  Serial.println(F(" steps/s^2"));
  Serial.print(F("Jerk: "));
  Serial.print(xAxisMotion_.getJerkSps3(), 1);
  Serial.println(F(" steps/s^3"));
  Serial.print(F("Position estimate: "));
  Serial.print(currentPositionSteps_);
  Serial.println(F(" steps"));
  Serial.print(F("Home known: "));
  Serial.println(homeKnown_ ? F("YES") : F("NO"));
  Serial.print(F("Home switch: "));
  Serial.println(isHomeSwitchTriggered() ? F("TRIGGERED") : F("RELEASED"));
  Serial.print(F("Pending manual turns: "));
  Serial.println(pendingManualTurns_);
  Serial.print(F("Pending HOME: "));
  Serial.println(pendingHomeRequest_ ? F("YES") : F("NO"));
}

void XAxisController::pollSerial() {
  // 串口按“逐字节接收 + 空闲超时提交命令”的方式工作，
  // 这样即使发送端不带换行，也能尽量识别命令。
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    if (received == '\r' || received == '\n') {
      processBufferedSerialCommand();
      continue;
    }

    if (serialBufferLength_ < sizeof(serialBuffer_) - 1) {
      serialBuffer_[serialBufferLength_++] = received;
      lastSerialByteMs_ = millis();
      continue;
    }

    serialBufferLength_ = 0;
    serialBuffer_[0] = '\0';
    Serial.println(F("Command too long, buffer cleared."));
  }

  if (serialBufferLength_ > 0 &&
      millis() - lastSerialByteMs_ >= SERIAL_COMMAND_IDLE_TIMEOUT_MS) {
    processBufferedSerialCommand();
  }
}

void XAxisController::processBufferedSerialCommand() {
  // 把已经收完整的一条命令交给解析器，并清空缓冲区准备下一条。
  if (serialBufferLength_ == 0) {
    return;
  }

  serialBuffer_[serialBufferLength_] = '\0';
  handleSerialCommand(serialBuffer_);
  serialBufferLength_ = 0;
  serialBuffer_[0] = '\0';
}

void XAxisController::handleSerialCommand(char* rawCommand) {
  // 命令进入业务解析前，先做去空格、去引号、统一大小写。
  trimCommandInPlace(rawCommand);
  stripWrappingQuotesInPlace(rawCommand);
  trimCommandInPlace(rawCommand);
  if (rawCommand[0] == '\0') {
    return;
  }

  int32_t turns = 0;
  if (parseSignedTurns(rawCommand, &turns)) {
    queueManualMove(turns);
    return;
  }

  upperCaseAsciiInPlace(rawCommand);

  float parsedValue = 0.0f;
  if (parsePositiveValueCommand(rawCommand,
                                'S',
                                MIN_CRUISE_RATE_SPS,
                                MAX_CRUISE_RATE_SPS,
                                &parsedValue)) {
    applyMotionProfile(parsedValue, accelerationSps2_);
    Serial.print(F("Cruise speed updated: "));
    Serial.print(cruiseRateSps_, 1);
    Serial.println(F(" steps/s"));
    return;
  }

  if (parsePositiveValueCommand(rawCommand,
                                'A',
                                MIN_ACCELERATION_SPS2,
                                MAX_ACCELERATION_SPS2,
                                &parsedValue)) {
    applyMotionProfile(cruiseRateSps_, parsedValue);
    Serial.print(F("Acceleration updated: "));
    Serial.print(accelerationSps2_, 1);
    Serial.println(F(" steps/s^2"));
    return;
  }

  if (strcmp(rawCommand, "STOP") == 0) {
    requestStop();
    return;
  }

  if (strcmp(rawCommand, "HOME") == 0) {
    requestHome();
    return;
  }

  if (strcmp(rawCommand, "AUTO ON") == 0) {
    autoModeEnabled_ = true;
    stopRequested_ = false;
    Serial.println(F("AUTO mode enabled."));
    return;
  }

  if (strcmp(rawCommand, "AUTO OFF") == 0) {
    autoModeEnabled_ = false;
    Serial.println(F("AUTO mode disabled."));
    return;
  }

  if (strcmp(rawCommand, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(rawCommand, "HELP") == 0 || strcmp(rawCommand, "?") == 0) {
    printSerialHelp();
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(rawCommand);
  printSerialHelp();
}

void XAxisController::queueManualMove(int32_t turns) {
  // 手动命令采用“排队请求”的方式，让主循环统一调度执行。
  pendingManualTurns_ = turns;
  Serial.print(F("Queued manual move: "));
  Serial.print(turns);
  Serial.println(F(" rev"));
}

void XAxisController::requestStop() {
  // STOP 的语义是：请求软停止，并清理会导致后续继续运动的待处理动作。
  stopRequested_ = true;
  pendingManualTurns_ = 0;
  pendingHomeRequest_ = false;
  autoModeEnabled_ = false;
  Serial.println(F("STOP requested: soft stopping current motion, AUTO OFF."));
}

void XAxisController::requestHome() {
  // HOME 会关闭自动模式，并在当前动作安全结束后进入回零流程。
  pendingHomeRequest_ = true;
  autoModeEnabled_ = false;
  Serial.println(F("HOME requested: AUTO OFF, homing will run after current motion stops."));
}

void XAxisController::clearStopIfIdle() {
  // 软停止标志只在主循环重新回到空闲态后清掉，避免重复触发。
  if (stopRequested_) {
    stopRequested_ = false;
    Serial.println(F("Soft stop completed. Controller is idle, AUTO OFF."));
  }
}

void XAxisController::performManualMoveIfPending() {
  // 手动动作会先等待 1 秒，给电机和用户一个明确的切换缓冲。
  if (pendingManualTurns_ == 0) {
    return;
  }

  Serial.println(F("Preparing manual move, waiting 1 second before execution."));
  waitWithService(MANUAL_COMMAND_PAUSE_MS, false);

  if (stopRequested_ || pendingHomeRequest_ || pendingManualTurns_ == 0) {
    return;
  }

  const int32_t turns = pendingManualTurns_;
  pendingManualTurns_ = 0;

  const bool dirLevel = turns > 0 ? FORWARD_DIR_LEVEL : REVERSE_DIR_LEVEL;
  const uint32_t steps = static_cast<uint32_t>(abs(turns)) * STEPS_PER_REV;
  const __FlashStringHelper* label =
      turns > 0 ? F("manual forward") : F("manual reverse");

  const StepperMoveReport report =
      executeMove(steps, dirLevel, MotionOwner::Manual, label, false);

  if (report.result == StepperMoveResult::SoftStopped) {
    Serial.println(F("Manual move was interrupted before the requested distance finished."));
  }
}

void XAxisController::performHomeSequenceIfPending() {
  // HOME 使用“快找 -> 退回 -> 慢靠”的二次靠近流程，提高可靠性。
  if (!pendingHomeRequest_) {
    return;
  }

  pendingHomeRequest_ = false;
  Serial.println(F("=== HOME sequence start ==="));

  if (isHomeSwitchTriggered()) {
    Serial.println(F("Home switch is already active, backing off first."));
    if (!runHomeBackoff(HOME_BACKOFF_STEPS, HOME_SEEK_RATE_SPS, F("home initial backoff"))) {
      return;
    }
  }

  if (!runHomeApproach(HOME_SEEK_MAX_STEPS, HOME_SEEK_RATE_SPS, F("home seek"))) {
    return;
  }

  if (!runHomeBackoff(HOME_BACKOFF_STEPS, HOME_SEEK_RATE_SPS, F("home release"))) {
    return;
  }

  if (!runHomeApproach(HOME_LATCH_MAX_STEPS, HOME_LATCH_RATE_SPS, F("home latch"))) {
    return;
  }

  currentPositionSteps_ = 0;
  homeKnown_ = true;
  Serial.println(F("HOME complete. Position reset to 0."));
}

void XAxisController::updatePositionByDirection(uint32_t stepsExecuted, bool dirLevel) {
  // 这里只维护一个“软件位置估计值”，用于串口状态显示和 HOME 后清零。
  const long signedSteps = static_cast<long>(stepsExecuted);
  if (dirLevel == FORWARD_DIR_LEVEL) {
    currentPositionSteps_ += signedSteps;
  } else {
    currentPositionSteps_ -= signedSteps;
  }
}

StepperMoveReport XAxisController::executeMove(uint32_t steps,
                                               bool dirLevel,
                                               MotionOwner owner,
                                               const __FlashStringHelper* label,
                                               bool stopOnHomeSwitch) {
  // 这里是统一运动入口：
  // 1. 设置当前动作归属
  // 2. 调用 StepperMotion 真正发脉冲
  // 3. 回写执行结果与位置估计
  currentMotionOwner_ = owner;
  currentMoveStopsOnHomeSwitch_ = stopOnHomeSwitch;

  Serial.print(F("Motion start: "));
  Serial.print(label);
  Serial.print(F(" | steps="));
  Serial.println(steps);

  const StepperMoveReport report =
      xAxisMotion_.moveSteps(steps, dirLevel, shouldInterruptBridge);

  currentMotionOwner_ = MotionOwner::None;
  currentMoveStopsOnHomeSwitch_ = false;
  updatePositionByDirection(report.stepsExecuted, dirLevel);

  Serial.print(F("Motion end: "));
  Serial.print(label);
  Serial.print(F(" | executed="));
  Serial.print(report.stepsExecuted);
  Serial.print(F(" | result="));
  Serial.println(report.result == StepperMoveResult::Completed ? F("completed")
                                                               : F("soft-stopped"));

  return report;
}
