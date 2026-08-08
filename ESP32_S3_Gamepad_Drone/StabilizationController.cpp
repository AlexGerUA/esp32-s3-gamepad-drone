#include "StabilizationController.h"

#include <Arduino.h>

#include "Config.h"

StabilizationController::StabilizationController()
    : _rollAnglePid(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_INTEGRATION_LIMIT,
                    ANGLE_PID_OUTPUT_LIMIT),
      _pitchAnglePid(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_INTEGRATION_LIMIT,
                     ANGLE_PID_OUTPUT_LIMIT),
      _yawAnglePid(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_INTEGRATION_LIMIT,
                   ANGLE_PID_OUTPUT_LIMIT),
      _rollRatePid(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
                   PID_ROLL_RATE_INTEGRATION_LIMIT, RATE_PID_OUTPUT_LIMIT),
      _pitchRatePid(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
                    PID_PITCH_RATE_INTEGRATION_LIMIT, RATE_PID_OUTPUT_LIMIT),
      _yawRatePid(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
                  PID_YAW_RATE_INTEGRATION_LIMIT, RATE_PID_OUTPUT_LIMIT) {}

float StabilizationController::capAngle(float angleDeg) {
  float result = angleDeg;
  while (result > 180.0f) result -= 360.0f;
  while (result < -180.0f) result += 360.0f;
  return result;
}

void StabilizationController::reset(float currentYawDeg) {
  _rollAnglePid.reset();
  _pitchAnglePid.reset();
  _yawAnglePid.reset();
  _rollRatePid.reset();
  _pitchRatePid.reset();
  _yawRatePid.reset();
  _yawTargetDeg = currentYawDeg;
}

MotorOutputs StabilizationController::compute(float throttleDuty, float pitchCmd, float rollCmd,
                                               float yawRateCmd, float measuredPitchDeg,
                                               float measuredRollDeg, float measuredYawDeg,
                                               float measuredGyroXDegS, float measuredGyroYDegS,
                                               float measuredGyroZDegS, float dtSeconds) {
  // controller_pid.c: "if (control->thrust == 0) { ... reset ... }"
  if (throttleDuty <= 0.0f) {
    reset(measuredYawDeg);
    return MotorOutputs{};
  }

  // Yaw - не прямий rate-control: стік інтегрується в ціль (як modeVelocity
  // у ESP-Drone), ціль тримається, коли стік у нулі.
  _yawTargetDeg = capAngle(_yawTargetDeg + yawRateCmd * MAX_YAW_RATE_DEG_S * dtSeconds);

  // pitchCmd інвертовано ТУТ (тільки для user-команди, не для self-stabilization):
  // стік "вперед" (pitchCmd>0) → бажаний нахил ніс униз (setpoint<0) → дрон
  // летить вперед. Self-stabilization працює з поточним знаком pitchOutput.
  float pitchSetpointDeg = -pitchCmd * MAX_TILT_ANGLE_DEG;
  float rollSetpointDeg = rollCmd * MAX_TILT_ANGLE_DEG;

  // --- Зовнішній (angle) контур: кут -> бажана кутова швидкість ---
  float rateDesiredRoll = _rollAnglePid.compute(rollSetpointDeg, measuredRollDeg, dtSeconds);
  float rateDesiredPitch = _pitchAnglePid.compute(pitchSetpointDeg, measuredPitchDeg, dtSeconds);

  float yawError = capAngle(_yawTargetDeg - measuredYawDeg);
  float rateDesiredYaw = _yawAnglePid.computeFromError(yawError, dtSeconds);

  // --- Внутрішній (rate) контур: gyro-швидкість -> сирий duty-вихід ---
  // ROLL ІНВЕРТОВАНИЙ для нашого монтажу MPU (діагноз 2026-07-28):
  // без інверсії дрон перевертався ліворуч при зльоті - позитивний
  // зворотний зв'язок. Pitch НЕ інвертувати - self-stabilization pitch
  // працює правильно з поточним знаком (спроба інвертувати 2026-07-29
  // призвела до миттєвого перекидання; інверсія user-команди pitch
  // окремо, у setpoint нижче).
  float rollOutput = -_rollRatePid.compute(rateDesiredRoll, measuredGyroXDegS, dtSeconds);
  float pitchOutput = _pitchRatePid.compute(rateDesiredPitch, -measuredGyroYDegS, dtSeconds);
  float yawOutput = -_yawRatePid.compute(rateDesiredYaw, measuredGyroZDegS, dtSeconds);
  // control->yaw = -control->yaw в оригіналі - інверсія застосована вище.

  // --- X-мікшер (power_distribution_stock.c, QUAD_FORMATION_X) ---
  float r = rollOutput / 2.0f;
  float p = pitchOutput / 2.0f;

  MotorOutputs out;
  out.fl = throttleDuty - r + p + yawOutput;
  out.rl = throttleDuty - r - p - yawOutput;
  out.rr = throttleDuty + r - p + yawOutput;
  out.fr = throttleDuty + r + p - yawOutput;

  float* motors[4] = {&out.fl, &out.rl, &out.rr, &out.fr};
  for (int i = 0; i < 4; i++) {
    if (*motors[i] < IDLE_THRUST) *motors[i] = IDLE_THRUST;
    *motors[i] = constrain(*motors[i], 0.0f, (float)MOTOR_PWM_MAX);
  }

  return out;
}
