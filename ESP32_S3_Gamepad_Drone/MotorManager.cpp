#include "MotorManager.h"

#include <Arduino.h>

#include "Config.h"

namespace {
const int kMotorPins[4] = {PIN_MOTOR_FL, PIN_MOTOR_RL, PIN_MOTOR_RR, PIN_MOTOR_FR};
// esp32-bluepad32 core бандлить старіший arduino-esp32 (channel-based LEDC
// API: ledcSetup/ledcAttachPin), не новий pin-based ledcAttach() з core 3.x.
const int kMotorChannels[4] = {0, 1, 2, 3};
}  // namespace

void MotorManager::begin() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(kMotorChannels[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(kMotorPins[i], kMotorChannels[i]);
    ledcWrite(kMotorChannels[i], 0);
  }
}

void MotorManager::setMotors(const MotorOutputs& outputs) {
  // outputs.* вже в сирих duty-одиницях (0..MOTOR_PWM_MAX) - StabilizationController
  // сам робить мікшер+кламп, тут лише запис у LEDC.
  float values[4] = {outputs.fl, outputs.rl, outputs.rr, outputs.fr};
  for (int i = 0; i < 4; i++) {
    float v = constrain(values[i], 0.0f, (float)MOTOR_PWM_MAX);
    ledcWrite(kMotorChannels[i], (uint32_t)v);
  }
}

void MotorManager::disableAll() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(kMotorChannels[i], 0);
  }
}
