#include "GamepadManager.h"

#include <Bluepad32.h>

#include "Config.h"

namespace {

GamepadPtr sGamepads[BP32_MAX_GAMEPADS];

float applyDeadzone(float value, float deadzone) {
  if (fabsf(value) < deadzone) return 0.0f;
  return value;
}

float normalizeAxis(int32_t raw) {
  float v = raw / 512.0f;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

void onConnectedGamepad(GamepadPtr gp) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (sGamepads[i] == nullptr) {
      Serial.printf("Gamepad connected, slot=%d, model=%s\n", i, gp->getModelName().c_str());
      sGamepads[i] = gp;
      return;
    }
  }
  Serial.println("Gamepad connected, but no free slot");
}

void onDisconnectedGamepad(GamepadPtr gp) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (sGamepads[i] == gp) {
      Serial.printf("Gamepad disconnected, slot=%d\n", i);
      sGamepads[i] = nullptr;
      return;
    }
  }
}

}  // namespace

void GamepadManager::begin() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) sGamepads[i] = nullptr;

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void GamepadManager::update() {
  BP32.update();

  GamepadPtr ctl = nullptr;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (sGamepads[i] != nullptr && sGamepads[i]->isConnected()) {
      ctl = sGamepads[i];
      break;
    }
  }

  if (ctl == nullptr) {
    _state.connected = false;
    _state.throttle = 0.0f;
    _state.pitch = 0.0f;
    _state.roll = 0.0f;
    _state.yawRate = 0.0f;
    _state.armButtonsHeld = false;
    _state.disarmButtonPressed = false;
    return;
  }

  _state.connected = true;
  _state.lastUpdateMs = millis();


  _state.throttle = ctl->throttle() / 1023.0f;  // 0..1

  _state.pitch = applyDeadzone(normalizeAxis(-ctl->axisY()), STICK_DEADZONE);
  _state.roll = applyDeadzone(normalizeAxis(ctl->axisX()), STICK_DEADZONE);
  _state.yawRate = applyDeadzone(normalizeAxis(ctl->axisRX()), STICK_DEADZONE);

  _state.armButtonsHeld = ctl->l1() && ctl->r1();
  _state.disarmButtonPressed = ctl->b();
}
