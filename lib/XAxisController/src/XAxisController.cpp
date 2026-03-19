/*
 * XAxisController.cpp
 *
 * 当前实现负责：
 * 1. 管理 X / Z 双轴的静态配置与运行时状态
 * 2. 处理串口命令、AUTO 模式、手动转动、HOME 流程
 * 3. 把具体步进执行委托给 StepperMotion
 */

#include "XAxisController.h"

#include <AxisProfile.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t EN_PIN = 12;
constexpr bool ENABLE_ACTIVE_LEVEL = LOW;
constexpr bool HOME_SWITCH_ACTIVE_LEVEL = LOW;

// 当前项目采用“EN-GND 硬件常使能”方案，避免 GPIO12 影响 ESP32 启动。
constexpr bool USE_SHARED_ENABLE_PIN = false;

constexpr uint8_t X_STEP_PIN = 26;
constexpr uint8_t X_DIR_PIN = 16;
constexpr uint8_t X_HOME_SWITCH_PIN = 13;

// 这套 ESP32 CNC Shield 上，Z 轴按当前实测接线使用 GPIO17 / GPIO14。
constexpr uint8_t Z_STEP_PIN = 17;
constexpr uint8_t Z_DIR_PIN = 14;
constexpr uint8_t UNUSED_HOME_PIN = 0;

constexpr bool FORWARD_DIR_LEVEL = HIGH;
constexpr bool REVERSE_DIR_LEVEL = LOW;

constexpr uint32_t X_STEPS_PER_REV = AxisProfile::kXStepsPerRev;
constexpr uint32_t Z_STEPS_PER_REV = AxisProfile::kZStepsPerRev;

constexpr uint8_t DEFAULT_AUTO_TURNS = 3;
constexpr uint16_t STEP_PULSE_HIGH_US = 10;
constexpr uint16_t STARTUP_STABILIZE_MS = 1000;
constexpr uint16_t MANUAL_COMMAND_PAUSE_MS = 1000;
constexpr uint16_t AUTO_PAUSE_AFTER_FORWARD_MS = 1000;
constexpr uint16_t AUTO_PAUSE_AFTER_REVERSE_MS = 1000;
constexpr uint16_t IDLE_SERVICE_DELAY_MS = 10;
constexpr uint16_t SERIAL_COMMAND_IDLE_TIMEOUT_MS = 30;

constexpr float X_DEFAULT_CRUISE_RATE_SPS = AxisProfile::kXDefaultCruiseRateSps;
constexpr float X_DEFAULT_ACCELERATION_SPS2 = AxisProfile::kXDefaultAccelerationSps2;
constexpr float DEFAULT_JERK_RATIO = 12.0f;
constexpr float MIN_START_RATE_SPS = 200.0f;
constexpr float START_RATE_RATIO = 0.35f;
constexpr float MIN_CRUISE_RATE_SPS = 100.0f;
constexpr float MAX_CRUISE_RATE_SPS = 20000.0f;
constexpr float MIN_ACCELERATION_SPS2 = 100.0f;
constexpr float MAX_ACCELERATION_SPS2 = 50000.0f;
constexpr float MIN_JERK_SPS3 = 500.0f;
constexpr float MAX_JERK_SPS3 = 500000.0f;

constexpr float HOME_SEEK_RATE_SPS = 400.0f;
constexpr float HOME_LATCH_RATE_SPS = 200.0f;
constexpr float HOME_ACCELERATION_SPS2 = 800.0f;
constexpr uint32_t HOME_SEEK_MAX_TURNS = 20;
constexpr uint32_t HOME_BACKOFF_STEPS = 200;
constexpr uint32_t HOME_LATCH_MAX_STEPS = 400;

constexpr XAxisController::AxisConfig kAxisConfig[] = {
    {F("X"), F("A4988"), F("1/16"), X_STEP_PIN, X_DIR_PIN, X_HOME_SWITCH_PIN, true,
     FORWARD_DIR_LEVEL, REVERSE_DIR_LEVEL, X_STEPS_PER_REV},
    {F("Z"), F("TMC2209"), F("1/16 (effective)"), Z_STEP_PIN, Z_DIR_PIN, UNUSED_HOME_PIN, false,
     FORWARD_DIR_LEVEL, REVERSE_DIR_LEVEL, Z_STEPS_PER_REV},
};

static_assert(sizeof(kAxisConfig) / sizeof(kAxisConfig[0]) ==
                  static_cast<size_t>(XAxisController::AxisId::Count),
              "Axis config count mismatch");

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float computeStartRateSps(float cruiseRate) {
  const float derivedRate = cruiseRate * START_RATE_RATIO;
  if (derivedRate < MIN_START_RATE_SPS) {
    return min(cruiseRate, MIN_START_RATE_SPS);
  }
  return min(cruiseRate, derivedRate);
}

// Z 轴默认速度按照每圈脉冲数同比换算，保证换轴后的“圈速感受”更接近。
float defaultCruiseRateSpsForAxis(XAxisController::AxisId axis) {
  switch (axis) {
    case XAxisController::AxisId::X:
      return X_DEFAULT_CRUISE_RATE_SPS;
    case XAxisController::AxisId::Z:
      return X_DEFAULT_CRUISE_RATE_SPS *
             (static_cast<float>(Z_STEPS_PER_REV) / static_cast<float>(X_STEPS_PER_REV));
    case XAxisController::AxisId::Count:
    default:
      return X_DEFAULT_CRUISE_RATE_SPS;
  }
}

float defaultAccelerationSps2ForAxis(XAxisController::AxisId axis) {
  switch (axis) {
    case XAxisController::AxisId::X:
      return X_DEFAULT_ACCELERATION_SPS2;
    case XAxisController::AxisId::Z:
      return X_DEFAULT_ACCELERATION_SPS2 *
             (static_cast<float>(Z_STEPS_PER_REV) / static_cast<float>(X_STEPS_PER_REV));
    case XAxisController::AxisId::Count:
    default:
      return X_DEFAULT_ACCELERATION_SPS2;
  }
}

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

void upperCaseAsciiInPlace(char* command) {
  for (size_t i = 0; command[i] != '\0'; ++i) {
    command[i] = static_cast<char>(toupper(static_cast<unsigned char>(command[i])));
  }
}

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

// 这些命令输入完整后就可以立即提交，不必等换行。
bool shouldSubmitImmediately(const char* command) {
  return strcmp(command, "STOP") == 0 || strcmp(command, "HOME") == 0 ||
         strcmp(command, "STATUS") == 0 || strcmp(command, "HELP") == 0 ||
         strcmp(command, "?") == 0 || strcmp(command, "AUTO ON") == 0 ||
         strcmp(command, "AUTO OFF") == 0 || strcmp(command, "AXIS X") == 0 ||
         strcmp(command, "AXIS Z") == 0 || strcmp(command, "AXIS?") == 0;
}

}  // namespace

XAxisController* XAxisController::activeInstance_ = nullptr;

XAxisController::AxisRuntime& XAxisController::runtimeFor(AxisId axis) {
  return axisRuntime_[axisIndex(axis)];
}

const XAxisController::AxisRuntime& XAxisController::runtimeFor(AxisId axis) const {
  return axisRuntime_[axisIndex(axis)];
}

const XAxisController::AxisConfig& XAxisController::configFor(AxisId axis) const {
  return kAxisConfig[axisIndex(axis)];
}

XAxisController::AxisRuntime& XAxisController::activeRuntime() {
  return runtimeFor(activeAxis_);
}

const XAxisController::AxisRuntime& XAxisController::activeRuntime() const {
  return runtimeFor(activeAxis_);
}

const XAxisController::AxisConfig& XAxisController::activeConfig() const {
  return configFor(activeAxis_);
}

const __FlashStringHelper* XAxisController::axisName(AxisId axis) const {
  return configFor(axis).name;
}

bool XAxisController::parseAxisCommand(const char* command, AxisId* axisOut) const {
  if (strcmp(command, "X") == 0) {
    *axisOut = AxisId::X;
    return true;
  }

  if (strcmp(command, "Z") == 0) {
    *axisOut = AxisId::Z;
    return true;
  }

  return false;
}

void XAxisController::begin() {
  activeInstance_ = this;

  Serial.begin(115200);

  if (USE_SHARED_ENABLE_PIN) {
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, ENABLE_ACTIVE_LEVEL);
  }

  for (size_t i = 0; i < axisIndex(AxisId::Count); ++i) {
    const AxisId axis = static_cast<AxisId>(i);
    const AxisConfig& config = configFor(axis);

    runtimeFor(axis).motion.begin(config.stepPin, config.dirPin);
    applyMotionProfile(axis,
                       defaultCruiseRateSpsForAxis(axis),
                       defaultAccelerationSps2ForAxis(axis));
    runtimeFor(axis).autoModeEnabled = (axis == AxisId::Z);

    if (config.hasHomeSwitch) {
      pinMode(config.homeSwitchPin, INPUT_PULLUP);
    }
  }

  printStartupBanner();
  delay(STARTUP_STABILIZE_MS);
}

void XAxisController::update() {
  pollSerial();
  clearStopIfIdle();
  applyPendingAxisSwitchIfIdle();

  performHomeSequenceIfPending();
  clearStopIfIdle();
  applyPendingAxisSwitchIfIdle();

  performManualMoveIfPending();
  clearStopIfIdle();
  applyPendingAxisSwitchIfIdle();

  performAutoCycleIfEnabled();
}

bool XAxisController::shouldInterruptBridge() {
  return activeInstance_ != nullptr && activeInstance_->shouldInterruptCurrentMotion();
}

bool XAxisController::shouldInterruptCurrentMotion() {
  pollSerial();

  const AxisRuntime& runtime = runtimeFor(executingAxis_);

  if (runtime.currentMoveStopsOnHomeSwitch && isHomeSwitchTriggered(executingAxis_)) {
    return true;
  }

  if (runtime.stopRequested || axisSwitchPending_) {
    return true;
  }

  switch (runtime.currentMotionOwner) {
    case MotionOwner::Auto:
      return !runtime.autoModeEnabled || runtime.pendingManualTurns != 0 ||
             runtime.pendingHomeRequest;
    case MotionOwner::Manual:
      return runtime.pendingHomeRequest;
    case MotionOwner::Home:
      return false;
    case MotionOwner::None:
    default:
      return false;
  }
}

bool XAxisController::isHomeSwitchTriggered(AxisId axis) const {
  const AxisConfig& config = configFor(axis);
  if (!config.hasHomeSwitch) {
    return false;
  }

  return digitalRead(config.homeSwitchPin) == HOME_SWITCH_ACTIVE_LEVEL;
}

bool XAxisController::waitWithService(uint32_t durationMs, bool allowInterrupt) {
  const unsigned long startMs = millis();

  while (millis() - startMs < durationMs) {
    pollSerial();
    applyPendingAxisSwitchIfIdle();

    const AxisRuntime& runtime = activeRuntime();
    if (allowInterrupt &&
        (runtime.stopRequested || runtime.pendingHomeRequest || runtime.pendingManualTurns != 0 ||
         !runtime.autoModeEnabled || axisSwitchPending_)) {
      return false;
    }

    delay(IDLE_SERVICE_DELAY_MS);
  }

  return true;
}

bool XAxisController::runHomeApproach(AxisId axis,
                                      uint32_t steps,
                                      float speedSps,
                                      const __FlashStringHelper* label) {
  AxisRuntime& runtime = runtimeFor(axis);
  const float savedCruiseRate = runtime.cruiseRateSps;
  const float savedAcceleration = runtime.accelerationSps2;

  applyMotionProfile(axis, speedSps, HOME_ACCELERATION_SPS2);
  const StepperMoveReport report =
      executeMove(axis, steps, configFor(axis).reverseDirLevel, MotionOwner::Home, label, true);
  applyMotionProfile(axis, savedCruiseRate, savedAcceleration);

  if (report.result == StepperMoveResult::SoftStopped && isHomeSwitchTriggered(axis)) {
    return true;
  }

  if (runtime.stopRequested) {
    Serial.println(F("HOME sequence stopped by STOP command."));
  } else {
    Serial.println(F("HOME sequence failed to detect the limit switch."));
  }

  return false;
}

bool XAxisController::runHomeBackoff(AxisId axis,
                                     uint32_t steps,
                                     float speedSps,
                                     const __FlashStringHelper* label) {
  AxisRuntime& runtime = runtimeFor(axis);
  const float savedCruiseRate = runtime.cruiseRateSps;
  const float savedAcceleration = runtime.accelerationSps2;

  applyMotionProfile(axis, speedSps, HOME_ACCELERATION_SPS2);
  const StepperMoveReport report =
      executeMove(axis, steps, configFor(axis).forwardDirLevel, MotionOwner::Home, label, false);
  applyMotionProfile(axis, savedCruiseRate, savedAcceleration);

  return report.result == StepperMoveResult::Completed && !runtime.stopRequested;
}

void XAxisController::applyPendingAxisSwitchIfIdle() {
  if (!axisSwitchPending_) {
    return;
  }

  const AxisRuntime& runtime = runtimeFor(executingAxis_);
  if (runtime.currentMotionOwner != MotionOwner::None) {
    return;
  }

  activeAxis_ = pendingAxis_;
  axisSwitchPending_ = false;
  Serial.print(F("Active axis switched to "));
  Serial.println(axisName(activeAxis_));
}

void XAxisController::applyMotionProfile(AxisId axis, float cruiseRate, float acceleration) {
  AxisRuntime& runtime = runtimeFor(axis);

  runtime.cruiseRateSps = clampFloat(cruiseRate, MIN_CRUISE_RATE_SPS, MAX_CRUISE_RATE_SPS);
  runtime.accelerationSps2 =
      clampFloat(acceleration, MIN_ACCELERATION_SPS2, MAX_ACCELERATION_SPS2);

  runtime.motion.setPulseHighUs(STEP_PULSE_HIGH_US);
  runtime.motion.setCruiseRateSps(runtime.cruiseRateSps);
  runtime.motion.setStartRateSps(computeStartRateSps(runtime.cruiseRateSps));
  runtime.motion.setAccelerationSps2(runtime.accelerationSps2);
  runtime.motion.setJerkSps3(clampFloat(runtime.accelerationSps2 * DEFAULT_JERK_RATIO,
                                        MIN_JERK_SPS3,
                                        MAX_JERK_SPS3));
}

void XAxisController::printStartupBanner() const {
  Serial.println(F("=== ESP32 X/Z Axis Controller ==="));
  Serial.println(F("X axis: A4988 1/16 microstep"));
  Serial.println(F("Z axis: TMC2209 1/16 effective microstep"));
  Serial.print(F("Shared EN pin: "));
  if (USE_SHARED_ENABLE_PIN) {
    Serial.println(EN_PIN);
  } else {
    Serial.println(F("GPIO12 ignored by firmware"));
    Serial.println(F("Recommended wiring: short EN to GND on CNC Shield"));
  }

  for (size_t i = 0; i < axisIndex(AxisId::Count); ++i) {
    const AxisId axis = static_cast<AxisId>(i);
    const AxisConfig& config = configFor(axis);
    const AxisRuntime& runtime = runtimeFor(axis);

    Serial.print(F("Axis "));
    Serial.print(config.name);
    Serial.print(F(": DRIVER="));
    Serial.print(config.driverName);
    Serial.print(F(", MICROSTEP="));
    Serial.print(config.microstepLabel);
    Serial.print(F(", STEP="));
    Serial.print(config.stepPin);
    Serial.print(F(", DIR="));
    Serial.print(config.dirPin);
    Serial.print(F(", STEPS/REV="));
    Serial.println(config.stepsPerRev);
    Serial.print(F("  Cruise="));
    Serial.print(runtime.cruiseRateSps, 1);
    Serial.print(F(" steps/s, Accel="));
    Serial.print(runtime.accelerationSps2, 1);
    Serial.println(F(" steps/s^2"));
  }

  Serial.print(F("Default active axis: "));
  Serial.println(axisName(activeAxis_));
  printSerialHelp();
}

void XAxisController::printSerialHelp() const {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  AXIS X   -> switch active axis to X"));
  Serial.println(F("  AXIS Z   -> switch active axis to Z"));
  Serial.println(F("  AXIS?    -> print current active axis"));
  Serial.println(F("  +N       -> move active axis forward N turns"));
  Serial.println(F("  -N       -> move active axis reverse N turns"));
  Serial.println(F("  Sxxxx    -> set active axis cruise speed in steps/second"));
  Serial.println(F("  Axxxx    -> set active axis acceleration in steps/second^2"));
  Serial.println(F("  STOP     -> soft stop current active axis and disable AUTO"));
  Serial.println(F("  HOME     -> run HOME on active axis if limit switch exists"));
  Serial.println(F("  AUTO ON  -> enable active axis auto round trip"));
  Serial.println(F("  AUTO OFF -> disable active axis auto mode"));
  Serial.println(F("  STATUS   -> print runtime status"));
  Serial.println(F("  HELP/?   -> print this help"));
}

void XAxisController::printStatus() const {
  Serial.println(F("=== Status ==="));
  Serial.print(F("Active axis: "));
  Serial.println(axisName(activeAxis_));

  for (size_t i = 0; i < axisIndex(AxisId::Count); ++i) {
    const AxisId axis = static_cast<AxisId>(i);
    const AxisConfig& config = configFor(axis);
    const AxisRuntime& runtime = runtimeFor(axis);

    Serial.print(F("[Axis "));
    Serial.print(config.name);
    Serial.println(F("]"));
    Serial.print(F("  Auto mode: "));
    Serial.println(runtime.autoModeEnabled ? F("ON") : F("OFF"));
    Serial.print(F("  Cruise speed: "));
    Serial.print(runtime.cruiseRateSps, 1);
    Serial.println(F(" steps/s"));
    Serial.print(F("  Acceleration: "));
    Serial.print(runtime.accelerationSps2, 1);
    Serial.println(F(" steps/s^2"));
    Serial.print(F("  Jerk: "));
    Serial.print(runtime.motion.getJerkSps3(), 1);
    Serial.println(F(" steps/s^3"));
    Serial.print(F("  Position estimate: "));
    Serial.print(runtime.currentPositionSteps);
    Serial.println(F(" steps"));
    Serial.print(F("  Steps per revolution: "));
    Serial.println(config.stepsPerRev);
    Serial.print(F("  Home known: "));
    Serial.println(runtime.homeKnown ? F("YES") : F("NO"));
    Serial.print(F("  Home switch: "));
    if (config.hasHomeSwitch) {
      Serial.println(isHomeSwitchTriggered(axis) ? F("TRIGGERED") : F("RELEASED"));
    } else {
      Serial.println(F("NOT CONFIGURED"));
    }
    Serial.print(F("  Pending manual turns: "));
    Serial.println(runtime.pendingManualTurns);
    Serial.print(F("  Pending HOME: "));
    Serial.println(runtime.pendingHomeRequest ? F("YES") : F("NO"));
  }

  if (axisSwitchPending_) {
    Serial.print(F("Pending axis switch: "));
    Serial.println(axisName(pendingAxis_));
  }
}

void XAxisController::pollSerial() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    if (received == '\r' || received == '\n') {
      processBufferedSerialCommand();
      continue;
    }

    if (serialBufferLength_ < sizeof(serialBuffer_) - 1) {
      serialBuffer_[serialBufferLength_++] = received;
      serialBuffer_[serialBufferLength_] = '\0';
      lastSerialByteMs_ = millis();

      char previewBuffer[sizeof(serialBuffer_)] = {0};
      memcpy(previewBuffer, serialBuffer_, serialBufferLength_ + 1);
      trimCommandInPlace(previewBuffer);
      stripWrappingQuotesInPlace(previewBuffer);
      trimCommandInPlace(previewBuffer);
      upperCaseAsciiInPlace(previewBuffer);
      if (shouldSubmitImmediately(previewBuffer)) {
        processBufferedSerialCommand();
      }
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
  if (serialBufferLength_ == 0) {
    return;
  }

  serialBuffer_[serialBufferLength_] = '\0';
  handleSerialCommand(serialBuffer_);
  serialBufferLength_ = 0;
  serialBuffer_[0] = '\0';
}

void XAxisController::handleSerialCommand(char* rawCommand) {
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

  if (strcmp(rawCommand, "AXIS?") == 0) {
    Serial.print(F("Active axis: "));
    Serial.println(axisName(activeAxis_));
    return;
  }

  if (strncmp(rawCommand, "AXIS ", 5) == 0) {
    AxisId requestedAxis = activeAxis_;
    if (parseAxisCommand(rawCommand + 5, &requestedAxis)) {
      requestAxisSwitch(requestedAxis);
      return;
    }
  }

  float parsedValue = 0.0f;
  if (parsePositiveValueCommand(rawCommand,
                                'S',
                                MIN_CRUISE_RATE_SPS,
                                MAX_CRUISE_RATE_SPS,
                                &parsedValue)) {
    applyMotionProfile(activeAxis_, parsedValue, activeRuntime().accelerationSps2);
    Serial.print(F("Axis "));
    Serial.print(axisName(activeAxis_));
    Serial.print(F(" cruise speed updated: "));
    Serial.print(activeRuntime().cruiseRateSps, 1);
    Serial.println(F(" steps/s"));
    return;
  }

  if (parsePositiveValueCommand(rawCommand,
                                'A',
                                MIN_ACCELERATION_SPS2,
                                MAX_ACCELERATION_SPS2,
                                &parsedValue)) {
    applyMotionProfile(activeAxis_, activeRuntime().cruiseRateSps, parsedValue);
    Serial.print(F("Axis "));
    Serial.print(axisName(activeAxis_));
    Serial.print(F(" acceleration updated: "));
    Serial.print(activeRuntime().accelerationSps2, 1);
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
    activeRuntime().autoModeEnabled = true;
    activeRuntime().stopRequested = false;
    Serial.print(F("Axis "));
    Serial.print(axisName(activeAxis_));
    Serial.println(F(" AUTO mode enabled."));
    return;
  }

  if (strcmp(rawCommand, "AUTO OFF") == 0) {
    activeRuntime().autoModeEnabled = false;
    Serial.print(F("Axis "));
    Serial.print(axisName(activeAxis_));
    Serial.println(F(" AUTO mode disabled."));
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
  AxisRuntime& runtime = activeRuntime();
  runtime.pendingManualTurns = turns;
  runtime.autoModeEnabled = false;

  Serial.print(F("Axis "));
  Serial.print(axisName(activeAxis_));
  Serial.print(F(" queued manual move: "));
  Serial.print(turns);
  Serial.println(F(" rev"));
}

void XAxisController::requestStop() {
  AxisRuntime& runtime = activeRuntime();
  runtime.stopRequested = true;
  runtime.pendingManualTurns = 0;
  runtime.pendingHomeRequest = false;
  runtime.autoModeEnabled = false;

  Serial.print(F("Axis "));
  Serial.print(axisName(activeAxis_));
  Serial.println(F(" STOP requested: soft stopping current motion, AUTO OFF."));
}

void XAxisController::requestHome() {
  const AxisConfig& config = activeConfig();
  AxisRuntime& runtime = activeRuntime();

  if (!config.hasHomeSwitch) {
    Serial.print(F("Axis "));
    Serial.print(config.name);
    Serial.println(F(" HOME is unavailable because no limit switch pin is configured."));
    return;
  }

  runtime.pendingHomeRequest = true;
  runtime.autoModeEnabled = false;
  Serial.print(F("Axis "));
  Serial.print(config.name);
  Serial.println(F(" HOME requested: AUTO OFF, homing will run after current motion stops."));
}

void XAxisController::requestAxisSwitch(AxisId axis) {
  if (!axisSwitchPending_ && activeAxis_ == axis) {
    Serial.print(F("Active axis is already "));
    Serial.println(axisName(axis));
    return;
  }

  pendingAxis_ = axis;
  axisSwitchPending_ = true;
  activeRuntime().autoModeEnabled = false;

  Serial.print(F("Axis switch requested: "));
  Serial.print(axisName(activeAxis_));
  Serial.print(F(" -> "));
  Serial.println(axisName(axis));
}

void XAxisController::clearStopIfIdle() {
  for (size_t i = 0; i < axisIndex(AxisId::Count); ++i) {
    AxisRuntime& runtime = runtimeFor(static_cast<AxisId>(i));
    if (!runtime.stopRequested || runtime.currentMotionOwner != MotionOwner::None) {
      continue;
    }

    runtime.stopRequested = false;
    Serial.print(F("Axis "));
    Serial.print(axisName(static_cast<AxisId>(i)));
    Serial.println(F(" soft stop completed. Controller is idle, AUTO OFF."));
  }
}

void XAxisController::performManualMoveIfPending() {
  AxisRuntime& runtime = activeRuntime();
  const AxisConfig& config = activeConfig();
  if (runtime.pendingManualTurns == 0) {
    return;
  }

  Serial.print(F("Axis "));
  Serial.print(config.name);
  Serial.println(F(" preparing manual move, waiting 1 second before execution."));
  waitWithService(MANUAL_COMMAND_PAUSE_MS, false);

  if (runtime.stopRequested || runtime.pendingHomeRequest || runtime.pendingManualTurns == 0 ||
      axisSwitchPending_) {
    return;
  }

  const int32_t turns = runtime.pendingManualTurns;
  runtime.pendingManualTurns = 0;

  const bool dirLevel = turns > 0 ? config.forwardDirLevel : config.reverseDirLevel;
  const uint32_t steps = static_cast<uint32_t>(abs(turns)) * config.stepsPerRev;
  const __FlashStringHelper* label =
      turns > 0 ? F("manual forward") : F("manual reverse");

  const StepperMoveReport report =
      executeMove(activeAxis_, steps, dirLevel, MotionOwner::Manual, label, false);

  if (report.result == StepperMoveResult::SoftStopped) {
    Serial.println(F("Manual move was interrupted before the requested distance finished."));
  }
}

void XAxisController::performHomeSequenceIfPending() {
  AxisRuntime& runtime = activeRuntime();
  const AxisConfig& config = activeConfig();
  if (!runtime.pendingHomeRequest) {
    return;
  }

  runtime.pendingHomeRequest = false;
  Serial.print(F("=== HOME sequence start on axis "));
  Serial.print(config.name);
  Serial.println(F(" ==="));

  const uint32_t homeSeekMaxSteps = config.stepsPerRev * HOME_SEEK_MAX_TURNS;
  if (isHomeSwitchTriggered(activeAxis_)) {
    Serial.println(F("Home switch is already active, backing off first."));
    if (!runHomeBackoff(activeAxis_, HOME_BACKOFF_STEPS, HOME_SEEK_RATE_SPS,
                        F("home initial backoff"))) {
      return;
    }
  }

  if (!runHomeApproach(activeAxis_, homeSeekMaxSteps, HOME_SEEK_RATE_SPS, F("home seek"))) {
    return;
  }

  if (!runHomeBackoff(activeAxis_, HOME_BACKOFF_STEPS, HOME_SEEK_RATE_SPS, F("home release"))) {
    return;
  }

  if (!runHomeApproach(activeAxis_, HOME_LATCH_MAX_STEPS, HOME_LATCH_RATE_SPS, F("home latch"))) {
    return;
  }

  runtime.currentPositionSteps = 0;
  runtime.homeKnown = true;
  Serial.print(F("Axis "));
  Serial.print(config.name);
  Serial.println(F(" HOME complete. Position reset to 0."));
}

void XAxisController::performAutoCycleIfEnabled() {
  AxisRuntime& runtime = activeRuntime();
  const AxisConfig& config = activeConfig();
  if (!runtime.autoModeEnabled) {
    delay(IDLE_SERVICE_DELAY_MS);
    return;
  }

  const uint32_t autoSteps = config.stepsPerRev * DEFAULT_AUTO_TURNS;
  if (executeMove(activeAxis_,
                  autoSteps,
                  config.forwardDirLevel,
                  MotionOwner::Auto,
                  F("auto forward"),
                  false)
          .result == StepperMoveResult::SoftStopped) {
    return;
  }

  if (!waitWithService(AUTO_PAUSE_AFTER_FORWARD_MS, true)) {
    return;
  }

  if (executeMove(activeAxis_,
                  autoSteps,
                  config.reverseDirLevel,
                  MotionOwner::Auto,
                  F("auto reverse"),
                  false)
          .result == StepperMoveResult::SoftStopped) {
    return;
  }

  waitWithService(AUTO_PAUSE_AFTER_REVERSE_MS, true);
}

void XAxisController::updatePositionByDirection(uint32_t stepsExecuted, bool dirLevel) {
  AxisRuntime& runtime = runtimeFor(executingAxis_);
  const AxisConfig& config = configFor(executingAxis_);
  const long signedSteps = static_cast<long>(stepsExecuted);

  if (dirLevel == config.forwardDirLevel) {
    runtime.currentPositionSteps += signedSteps;
  } else {
    runtime.currentPositionSteps -= signedSteps;
  }
}

StepperMoveReport XAxisController::executeMove(AxisId axis,
                                               uint32_t steps,
                                               bool dirLevel,
                                               MotionOwner owner,
                                               const __FlashStringHelper* label,
                                               bool stopOnHomeSwitch) {
  AxisRuntime& runtime = runtimeFor(axis);
  const AxisConfig& config = configFor(axis);

  executingAxis_ = axis;
  runtime.currentMotionOwner = owner;
  runtime.currentMoveStopsOnHomeSwitch = stopOnHomeSwitch;

  Serial.print(F("Axis "));
  Serial.print(config.name);
  Serial.print(F(" motion start: "));
  Serial.print(label);
  Serial.print(F(" | steps="));
  Serial.print(steps);
  Serial.print(F(" | dir="));
  Serial.println(dirLevel == HIGH ? F("HIGH") : F("LOW"));

  const StepperMoveReport report =
      runtime.motion.moveSteps(steps, dirLevel, shouldInterruptBridge);

  runtime.currentMotionOwner = MotionOwner::None;
  runtime.currentMoveStopsOnHomeSwitch = false;
  updatePositionByDirection(report.stepsExecuted, dirLevel);

  Serial.print(F("Axis "));
  Serial.print(config.name);
  Serial.print(F(" motion end: "));
  Serial.print(label);
  Serial.print(F(" | executed="));
  Serial.print(report.stepsExecuted);
  Serial.print(F(" | result="));
  Serial.println(report.result == StepperMoveResult::Completed ? F("completed")
                                                               : F("soft-stopped"));

  return report;
}
