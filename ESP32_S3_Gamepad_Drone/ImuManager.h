#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Adafruit_MPU6050.h>

#include "Config.h"

// MPU6050 через Adafruit_MPU6050 + Mahony AHRS (quaternion-фільтр), портовано
// з ESP-Drone (components/core/crazyflie/modules/src/sensfusion6.c,
// MAHONY_QUATERNION_IMU гілка). Калібрування gyro-bias відбувається один раз
// у begin() - дрон МАЄ стояти нерухомо на рівній поверхні в цей момент.
//
// Знаки осей pitch/roll/yaw - конвенція ESP-Drone (sensfusion6.c відмічає
// "Pitch seems to be inverted" в оригіналі, це навмисно узгоджено з тим, як
// StabilizationController застосовує -gyroY, як в controller_pid.c). Якщо
// на стенді без гвинтів реакція виявиться дзеркальною - інвертувати відповідний
// знак саме в StabilizationController, а не тут (щоб не розсинхронізувати
// фільтр і мікшер).
class ImuManager {
 public:
  bool begin();

  // dtSeconds - час з попереднього виклику update()
  void update(float dtSeconds);

  // Скориговані accel-based level trim'ом (компенсує похибку монтажу
  // MPU6050 на рамі - див. captureLevelTrim()).
  float pitchDeg() const { return _pitchDeg - _pitchTrimDeg; }
  float rollDeg() const { return _rollDeg - _rollTrimDeg; }
  float yawDeg() const { return _yawDeg; }  // yaw не тримиться - немає "абсолютного рівня"

  // Bias-скориговані, БЕЗ інверсії осей (інверсія - у StabilizationController).
  float gyroXDegPerSec() const { return _gyroXDps; }
  float gyroYDegPerSec() const { return _gyroYDps; }
  float gyroZDegPerSec() const { return _gyroZDps; }

  // true, якщо хоч раз побачили апаратне INT-переривання (data-ready) з
  // MPU6050 на PIN_MPU_INT - підтверджує, що дріт INT дійсно підключений і
  // чіп сигналізує. НЕ впливає на роботу update() (той завжди полінгом,
  // безпечніше під час bench-тестів) - лише діагностика.
  bool interruptDetected() const { return _interruptSeen; }

  // false, якщо кілька I2C-читань поспіль (>=I2C_FAILURE_DISARM_THRESHOLD)
  // провалились під час update() - ознака зависання/відірваного контакту
  // I2C-шини. Перевіряти в .ino і форсувати disarm, якщо false.
  bool isHealthy() const { return _consecutiveI2cFailures < I2C_FAILURE_DISARM_THRESHOLD; }

  // Повторне калібрування gyro-bias + level trim. БЛОКУЮЧЕ (до ~5с з wait-
  // for-still): спочатку чекає, поки рух не спаде нижче порогу
  // CALIB_STILL_GYRO_THRESHOLD_DPS (макс CALIB_STILL_MAX_WAIT_MS), потім
  // семплює. Якщо variance перевищує CALIB_MAX_GYRO_VARIANCE_DPS, вважає
  // що дрон рухали під час калібрування і НЕ переписує старий bias/trim
  // (краще старий надійний, ніж новий сміттєвий). Друкує в Serial результат.
  void recalibrate();

 private:
  Adafruit_MPU6050 _mpu;

  float _gyroBiasX = 0.0f;
  float _gyroBiasY = 0.0f;
  float _gyroBiasZ = 0.0f;

  // Quaternion сенсорного фрейму відносно опорного (Mahony AHRS).
  float _qw = 1.0f;
  float _qx = 0.0f;
  float _qy = 0.0f;
  float _qz = 0.0f;

  float _integralFBx = 0.0f;
  float _integralFBy = 0.0f;
  float _integralFBz = 0.0f;

  float _pitchDeg = 0.0f;
  float _rollDeg = 0.0f;
  float _yawDeg = 0.0f;

  // Level trim (accel-based) - компенсує похибку монтажу MPU6050 на рамі.
  float _pitchTrimDeg = 0.0f;
  float _rollTrimDeg = 0.0f;

  float _gyroXDps = 0.0f;
  float _gyroYDps = 0.0f;
  float _gyroZDps = 0.0f;

  // true, якщо Adafruit_MPU6050::begin() відмовив через WHO_AM_I-перевірку
  // (дешеві GY-521 клони часто мають MPU6500/інший чіп під тим самим
  // корпусом і повертають не 0x68) - у цьому режимі читаємо регістри
  // напряму через Wire, в обхід Adafruit-бібліотеки.
  bool _rawMode = false;

  bool _interruptSeen = false;
  void configureInterrupt();

  int _consecutiveI2cFailures = 0;

  // Повертає false якщо variance-check не пройдено (дрон рухали під час
  // калібрування) - у цьому випадку _gyroBias* НЕ переписані.
  bool calibrateGyro();
  // Чекає, поки |gyro| < CALIB_STILL_GYRO_THRESHOLD_DPS протягом >200мс, або
  // поки не пройде CALIB_STILL_MAX_WAIT_MS. Повертає true якщо дочекались.
  bool waitForStill();
  // gx/gy/gz - град/с; ax/ay/az - будь-яка узгоджена одиниця (нормалізується всередині)
  void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  // Читання зразка + Mahony + Euler -> _pitchDeg/_rollDeg/_yawDeg (сирі,
  // без трим-корекції). Спільна логіка для update() і captureLevelTrim().
  // Повертає false при помилці I2C-читання (нічого не оновлює в такому разі).
  bool runFusionStep(float dtSeconds);
  // Скидає quaternion, дає фільтру ~1с збіжитись, фіксує _pitchTrimDeg/
  // _rollTrimDeg з поточного нахилу. Дрон МАЄ лежати рівно й нерухомо.
  // Повертає false якщо під час збіжності був суттєвий рух (variance
  // перевищив поріг) - _pitchTrimDeg/_rollTrimDeg НЕ переписані в такому разі.
  bool captureLevelTrim();

  // Один зразок accel(м/с²)+gyro(рад/с) - або через Adafruit, або raw Wire,
  // залежно від _rawMode. Повертає false при помилці I2C-читання.
  bool readSample(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);

  // Raw-режим: пряма робота з регістрами MPU6050/сумісного клона.
  bool rawProbe(uint8_t& outWhoAmI);
  void rawWakeAndConfigure();
  bool rawRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
  static void scanI2CBus();
};

#endif  // IMU_MANAGER_H
