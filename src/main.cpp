#include <Arduino.h>

#include <AxisProfile.h>

#ifndef Z_AXIS_DIAGNOSTIC_MODE
#define Z_AXIS_DIAGNOSTIC_MODE 0
#endif

#if Z_AXIS_DIAGNOSTIC_MODE

namespace diag {

constexpr uint8_t kZStepPin = 17;
constexpr uint8_t kZDirPin = 14;
constexpr uint32_t kSerialBaud = 115200;

enum class PendingMove : uint8_t {
  None,
  DirHighTurn,
  DirLowTurn,
};

struct DiagnosticState {
  bool autoModeEnabled = true;
  bool stopRequested = false;
  bool lastDirLevel = LOW;
  PendingMove pendingMove = PendingMove::None;
};

DiagnosticState state;

void printHelp() {
  Serial.println(F("=== Z Axis DIR Diagnostic ==="));
  Serial.println(F("单字符命令（无需换行）:"));
  Serial.println(F("  A -> 切换自动往复"));
  Serial.println(F("  H -> DIR=HIGH 转 1 圈"));
  Serial.println(F("  L -> DIR=LOW 转 1 圈"));
  Serial.println(F("  S -> 停止当前动作并关闭自动模式"));
  Serial.println(F("  P -> 打印当前状态"));
  Serial.println(F("  ? -> 打印帮助"));
  Serial.println(F("自动模式: HIGH 1 圈 -> 暂停 -> LOW 1 圈 -> 暂停"));
  Serial.print(F("Z 轴实测有效细分: 1/"));
  Serial.println(AxisProfile::kZMicrosteps);
}

const __FlashStringHelper* pendingMoveLabel(PendingMove move) {
  switch (move) {
    case PendingMove::DirHighTurn:
      return F("DIR=HIGH 1 turn");
    case PendingMove::DirLowTurn:
      return F("DIR=LOW 1 turn");
    case PendingMove::None:
    default:
      return F("NONE");
  }
}

void printStatus() {
  Serial.println(F("=== Status ==="));
  Serial.print(F("Auto mode: "));
  Serial.println(state.autoModeEnabled ? F("ON") : F("OFF"));
  Serial.print(F("Pending move: "));
  Serial.println(pendingMoveLabel(state.pendingMove));
  Serial.print(F("Last DIR level: "));
  Serial.println(state.lastDirLevel == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("Pulses per revolution: "));
  Serial.println(AxisProfile::kZStepsPerRev);
  Serial.print(F("Step high/low us: "));
  Serial.print(AxisProfile::kDiagnosticStepHighUs);
  Serial.print(F(" / "));
  Serial.println(AxisProfile::kDiagnosticStepLowUs);
  Serial.print(F("Target 1-turn time: "));
  Serial.print(AxisProfile::kZDiagnosticTurnSeconds, 1);
  Serial.println(F(" s"));
}

void disableAutoAndClearQueue() {
  state.autoModeEnabled = false;
  state.pendingMove = PendingMove::None;
}

void queueMove(PendingMove move) {
  disableAutoAndClearQueue();
  state.stopRequested = false;
  state.pendingMove = move;
}

void handleCommand(char command) {
  switch (command) {
    case 'A':
      state.autoModeEnabled = !state.autoModeEnabled;
      state.stopRequested = false;
      state.pendingMove = PendingMove::None;
      Serial.print(F("Auto mode "));
      Serial.println(state.autoModeEnabled ? F("ENABLED") : F("DISABLED"));
      break;
    case 'H':
      queueMove(PendingMove::DirHighTurn);
      Serial.println(F("Queued 1 turn with DIR=HIGH"));
      break;
    case 'L':
      queueMove(PendingMove::DirLowTurn);
      Serial.println(F("Queued 1 turn with DIR=LOW"));
      break;
    case 'S':
      disableAutoAndClearQueue();
      state.stopRequested = true;
      Serial.println(F("STOP requested"));
      break;
    case 'P':
      printStatus();
      break;
    case '?':
      printHelp();
      break;
    default:
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(toupper(Serial.read()));
    if (received == '\r' || received == '\n' || received == ' ') {
      continue;
    }
    handleCommand(received);
  }
}

bool waitWithService(uint32_t durationMs) {
  const unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    pollSerial();
    if (state.stopRequested) {
      return false;
    }
    delay(5);
  }
  return true;
}

bool runOneTurn(bool dirLevel) {
  state.stopRequested = false;
  state.lastDirLevel = dirLevel;

  Serial.print(F("Move start: DIR="));
  Serial.print(dirLevel == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(", steps="));
  Serial.println(AxisProfile::kZStepsPerRev);

  digitalWrite(kZDirPin, dirLevel);
  delay(AxisProfile::kDiagnosticDirSettleMs);

  uint32_t executedSteps = 0;
  while (executedSteps < AxisProfile::kZStepsPerRev) {
    pollSerial();
    if (state.stopRequested) {
      Serial.print(F("Move stopped early at steps="));
      Serial.println(executedSteps);
      state.stopRequested = false;
      return false;
    }

    digitalWrite(kZStepPin, HIGH);
    delayMicroseconds(AxisProfile::kDiagnosticStepHighUs);
    digitalWrite(kZStepPin, LOW);
    delayMicroseconds(AxisProfile::kDiagnosticStepLowUs);
    ++executedSteps;
  }

  Serial.print(F("Move complete: DIR="));
  Serial.println(dirLevel == HIGH ? F("HIGH") : F("LOW"));
  return true;
}

bool runPendingMoveIfAny() {
  if (state.pendingMove == PendingMove::None) {
    return false;
  }

  const PendingMove move = state.pendingMove;
  state.pendingMove = PendingMove::None;

  if (move == PendingMove::DirHighTurn) {
    runOneTurn(HIGH);
  } else {
    runOneTurn(LOW);
  }
  return true;
}

void runAutoCycleIfEnabled() {
  if (!state.autoModeEnabled) {
    delay(5);
    return;
  }

  if (!runOneTurn(HIGH)) {
    return;
  }
  if (!waitWithService(AxisProfile::kDiagnosticAutoPauseMs)) {
    return;
  }
  if (!runOneTurn(LOW)) {
    return;
  }
  waitWithService(AxisProfile::kDiagnosticAutoPauseMs);
}

void initializePins() {
  pinMode(kZStepPin, OUTPUT);
  pinMode(kZDirPin, OUTPUT);
  digitalWrite(kZStepPin, LOW);
  digitalWrite(kZDirPin, LOW);
}

}  // namespace diag

void setup() {
  Serial.begin(diag::kSerialBaud);
  diag::initializePins();

  delay(300);
  diag::printHelp();
  diag::printStatus();
}

void loop() {
  diag::pollSerial();

  if (diag::runPendingMoveIfAny()) {
    return;
  }

  diag::runAutoCycleIfEnabled();
}

#else

#include <XAxisController.h>

namespace {
// 正式模式下把串口协议、状态机和轴管理都交给控制器对象。
XAxisController controller;
}

void setup() {
  controller.begin();
}

void loop() {
  controller.update();
}

#endif
