// ESP32-S3 SuperMini + Bluepad32 (Xbox One S) + MPU6050 + 4x SI2300 MOSFET
// брашед-дрон. Логіка стабілізації (Mahony AHRS + каскад angle/rate PID +
// X-мікшер) портована зі структури офіційного espressif/esp-drone.

#include "Config.h"
#include "GamepadManager.h"
#include "ImuManager.h"
#include "StabilizationController.h"
#include "MotorManager.h"
#include "StatusIndicator.h"

GamepadManager gamepad;
ImuManager imu;
StabilizationController stabilization;
MotorManager motors;
StatusIndicator statusLed;

bool imuOk = false;
bool armed = false;
bool armHoldActive = false;
unsigned long armHoldStartMs = 0;

unsigned long lastLoopMicros = 0;
unsigned long lastDebugPrintMs = 0;

// Три режими виводу залежно від стану:
//   armed              -> детальна телеметрія кожні DEBUG_PRINT_INTERVAL_MS
//   disarmed + gamepad -> TEST MODE: розкладка стіків з підписами (250мс)
//   disarmed, no gp    -> heartbeat раз на STATUS_HEARTBEAT_INTERVAL_MS
const char* pitchLabel(float v) {
  if (v > 0.15f) return "ВПЕРЕД";
  if (v < -0.15f) return "НАЗАД";
  return "-";
}
const char* rollLabel(float v) {
  if (v > 0.15f) return "ВПРАВО";
  if (v < -0.15f) return "ВЛІВО";
  return "-";
}
const char* yawLabel(float v) {
  if (v > 0.15f) return "ПОВОРОТ ВПРАВО";
  if (v < -0.15f) return "ПОВОРОТ ВЛІВО";
  return "-";
}

void printStatus(const GamepadManager::State& gs, bool gamepadOk, const MotorOutputs& outputs) {
  unsigned long interval;
  if (armed) interval = DEBUG_PRINT_INTERVAL_MS;
  else if (gamepadOk) interval = TEST_MODE_INTERVAL_MS;
  else interval = STATUS_HEARTBEAT_INTERVAL_MS;

  if (millis() - lastDebugPrintMs < interval) return;
  lastDebugPrintMs = millis();

  if (armed) {
    Serial.printf(
        "armed=1 gp=%d imu=%d | thr=%.2f pitchCmd=%.2f rollCmd=%.2f yawCmd=%.2f | "
        "pitch=%.1f roll=%.1f yaw=%.1f | M: FL=%.0f RL=%.0f FR=%.0f RR=%.0f\n",
        gamepadOk, imuOk, gs.throttle, gs.pitch, gs.roll, gs.yawRate, imu.pitchDeg(),
        imu.rollDeg(), imu.yawDeg(), outputs.fl, outputs.rl, outputs.fr, outputs.rr);
  } else if (gamepadOk) {
    Serial.printf(
        "[TEST] RT=%3d%% (%s) | L-Y=%+.2f (%-6s) L-X=%+.2f (%-6s) | R-X=%+.2f (%-14s) | "
        "L1+R1=%s B=%s | IMU: pitch=%+.1f° roll=%+.1f° yaw=%+.1f° | ARM: %s\n",
        (int)(gs.throttle * 100), gs.throttle > ARM_THROTTLE_DEADBAND ? "газ>0" : "газ 0%",
        gs.pitch, pitchLabel(gs.pitch),
        gs.roll, rollLabel(gs.roll),
        gs.yawRate, yawLabel(gs.yawRate),
        gs.armButtonsHeld ? "ТАК" : "нi",
        gs.disarmButtonPressed ? "ТАК" : "нi",
        imu.pitchDeg(), imu.rollDeg(), imu.yawDeg(),
        (gs.armButtonsHeld && imuOk && gs.throttle < ARM_THROTTLE_DEADBAND) ? "готовий (тримай L1+R1)" : "заблоковано");
  } else {
    Serial.printf("[стан] imu=%s gamepad=немає\n", imuOk ? "OK" : "FAIL");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32_S3_Gamepad_Drone: старт");

  statusLed.begin();
  gamepad.begin();

  imuOk = imu.begin();
  if (!imuOk) {
    Serial.println("УВАГА: MPU6050 не ініціалізовано - arm заблоковано");
  }

  motors.begin();
  motors.disableAll();

  lastLoopMicros = micros();
}

void updateArmState(const GamepadManager::State& gs, bool gamepadOk) {
  bool wasArmed = armed;

  if (!gamepadOk) {
    armed = false;
    armHoldActive = false;
  } else if (armed && gs.disarmButtonPressed) {
    armed = false;
    armHoldActive = false;
    Serial.println("DISARM (кнопка)");
  } else if (!armed) {
    bool canArm = gs.armButtonsHeld && imuOk && gs.throttle < ARM_THROTTLE_DEADBAND;
    if (canArm) {
      if (!armHoldActive) {
        armHoldActive = true;
        armHoldStartMs = millis();
      } else if (millis() - armHoldStartMs >= ARM_HOLD_MS) {
        armed = true;
        armHoldActive = false;
        stabilization.reset(imu.yawDeg());
        Serial.println("ARMED");
      }
    } else {
      armHoldActive = false;
    }
  }

  // Автокалібрування gyro-bias + level trim одразу після кожного DISARM
  // (незалежно від причини - кнопка чи failsafe). Мотори вже примусово 0
  // (disableAll() нижче в loop()), тому ~2с блокуючої калібровки тут
  // безпечна. Компенсує температурний/часовий дрейф gyro-bias і трохи
  // усереднює похибку монтажу MPU6050 між польотами, без потреби
  // перезавантажувати плату. Дрон МАЄ лежати рівно й нерухомо в цей момент.
  if (wasArmed && !armed && imuOk) {
    motors.disableAll();
    Serial.println("DISARM -> автокалібрування IMU (дрон рівно, не рухати; чекає спокою)");
    imu.recalibrate();  // сама друкує деталі
  }
}

void loop() {
  unsigned long nowMicros = micros();
  unsigned long elapsedMicros = nowMicros - lastLoopMicros;
  if (elapsedMicros < CONTROL_LOOP_PERIOD_US) return;

  float dtSeconds = elapsedMicros / 1000000.0f;
  lastLoopMicros = nowMicros;

  gamepad.update();
  const GamepadManager::State& gs = gamepad.state();

  bool gamepadOk = gs.connected && (millis() - gs.lastUpdateMs < GAMEPAD_TIMEOUT_MS);

  if (imuOk) {
    imu.update(dtSeconds);
  }

  updateArmState(gs, gamepadOk);
  statusLed.update(imuOk, gamepadOk, armed, millis());

  MotorOutputs outputs;
  if (armed) {
    float throttleDuty = gs.throttle * MOTOR_PWM_MAX;
    outputs = stabilization.compute(throttleDuty, gs.pitch, gs.roll, gs.yawRate, imu.pitchDeg(),
                                     imu.rollDeg(), imu.yawDeg(), imu.gyroXDegPerSec(),
                                     imu.gyroYDegPerSec(), imu.gyroZDegPerSec(), dtSeconds);
    motors.setMotors(outputs);
  } else {
    motors.disableAll();
  }

  printStatus(gs, gamepadOk, outputs);
}
