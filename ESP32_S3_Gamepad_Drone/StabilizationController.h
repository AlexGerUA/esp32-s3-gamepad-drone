#ifndef STABILIZATION_CONTROLLER_H
#define STABILIZATION_CONTROLLER_H

// Generic PID, портовано з ESP-Drone modules/src/pid.c (без D-lowpass
// фільтра - в оригіналі він теж вимкнений за замовчуванням,
// ATTITUDE_LPF_ENABLE/ATTITUDE_RATE_LPF_ENABLE = false).
class PidController {
 public:
  PidController(float kp, float ki, float kd, float iLimit, float outputLimit)
      : _kp(kp), _ki(ki), _kd(kd), _iLimit(iLimit), _outputLimit(outputLimit) {}

  // Стандартний шлях: похибка рахується як setpoint - measurement.
  float compute(float setpoint, float measurement, float dtSeconds) {
    _error = setpoint - measurement;
    return updateFromError(dtSeconds);
  }

  // Шлях для yaw (capAngle-похибка рахується зовні, як pidSetError +
  // pidUpdate(updateError=false) у ESP-Drone controller_pid.c).
  float computeFromError(float error, float dtSeconds) {
    _error = error;
    return updateFromError(dtSeconds);
  }

  void reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
  }

 private:
  float updateFromError(float dtSeconds) {
    float outP = _kp * _error;

    float derivative = (dtSeconds > 0.0f) ? (_error - _prevError) / dtSeconds : 0.0f;
    float outD = _kd * derivative;
    _prevError = _error;

    _integral += _error * dtSeconds;
    if (_iLimit != 0.0f) {
      if (_integral > _iLimit) _integral = _iLimit;
      if (_integral < -_iLimit) _integral = -_iLimit;
    }
    float outI = _ki * _integral;

    float output = outP + outI + outD;
    if (_outputLimit != 0.0f) {
      if (output > _outputLimit) output = _outputLimit;
      if (output < -_outputLimit) output = -_outputLimit;
    }
    return output;
  }

  float _kp, _ki, _kd;
  float _iLimit;
  float _outputLimit;
  float _error = 0.0f;
  float _prevError = 0.0f;
  float _integral = 0.0f;
};

struct MotorOutputs {
  // Сирі PWM-duty одиниці, 0..MOTOR_PWM_MAX (не нормалізовані 0..1) - та сама
  // шкала підходу, що й thrust/control у ESP-Drone (там 0..65535).
  float fl = 0.0f;
  float rl = 0.0f;
  float rr = 0.0f;
  float fr = 0.0f;
};

// Каскад angle -> rate PID (6 інстанцій) + X-мікшер, портовано зі структури
// ESP-Drone: attitude_pid_controller.c (каскад), controller_pid.c (оркестрація,
// yaw через інтеграцію в target з capAngle, zero-thrust reset), і
// power_distribution_stock.c (мікшер, QUAD_FORMATION_X).
class StabilizationController {
 public:
  StabilizationController();

  // throttleDuty - 0..MOTOR_PWM_MAX; pitchCmd/rollCmd -1..1 (кут), yawRateCmd
  // -1..1 (швидкість обертання ЦІЛІ по yaw, не прямий rate-control);
  // measured* - з ImuManager (град. / град/с, БЕЗ інверсії - інверсія тут,
  // як у controller_pid.c: attitudeControllerCorrectRatePID(gyroX, -gyroY, gyroZ, ...)).
  MotorOutputs compute(float throttleDuty, float pitchCmd, float rollCmd, float yawRateCmd,
                        float measuredPitchDeg, float measuredRollDeg, float measuredYawDeg,
                        float measuredGyroXDegS, float measuredGyroYDegS, float measuredGyroZDegS,
                        float dtSeconds);

  // Скидає всі 6 PID і ресинхронізує yaw-ціль на поточний виміряний yaw
  // (викликати при ARM, і автоматично викликається всередині compute() при
  // throttleDuty == 0 - як controller_pid.c: if (control->thrust == 0) {...}).
  void reset(float currentYawDeg);

 private:
  PidController _rollAnglePid;
  PidController _pitchAnglePid;
  PidController _yawAnglePid;

  PidController _rollRatePid;
  PidController _pitchRatePid;
  PidController _yawRatePid;

  float _yawTargetDeg = 0.0f;

  static float capAngle(float angleDeg);
};

#endif  // STABILIZATION_CONTROLLER_H
