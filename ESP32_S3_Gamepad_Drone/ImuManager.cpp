#include "ImuManager.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Config.h"

namespace {
// Портовано з ESP-Drone sensfusion6.c (MAHONY_QUATERNION_IMU гілка).
const float kTwoKp = MAHONY_TWO_KP;
const float kTwoKi = MAHONY_TWO_KI;

// Regmap MPU6050/MPU6500 (сумісний у обох, клон-чіпи теж підтримують ці
// регістри - відрізняється лише WHO_AM_I).
const uint8_t kRegWhoAmI = 0x75;
const uint8_t kRegPwrMgmt1 = 0x6B;
const uint8_t kRegGyroConfig = 0x1B;
const uint8_t kRegAccelConfig = 0x1C;
const uint8_t kRegAccelXoutH = 0x3B;

const uint8_t kGyroConfig500Dps = 0x08;   // FS_SEL=1  (має відповідати MPU6050_RANGE_500_DEG)
const uint8_t kAccelConfig8G = 0x10;      // AFS_SEL=2 (має відповідати MPU6050_RANGE_8_G)
const float kAccelSensitivityLsbPerG = 4096.0f;
const float kGyroSensitivityLsbPerDps = 65.5f;
const float kGravityMs2 = 9.80665f;

const uint8_t kRegIntPinCfg = 0x37;
const uint8_t kRegIntEnable = 0x38;

const uint8_t kRegConfig = 0x1A;     // DLPF (digital low-pass filter)
const uint8_t kDlpfCfg21Hz = 0x04;   // ~21Гц accel+gyro, як MPU6050_BAND_21_HZ в Adafruit-режимі

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ISR має бути вільною функцією (не член класу) + IRAM_ATTR на ESP32.
// Лише виставляє прапорець - жодної I2C-роботи всередині переривання.
volatile bool sMpuDataReady = false;

void IRAM_ATTR onMpuDataReady() { sMpuDataReady = true; }
}  // namespace

bool ImuManager::rawProbe(uint8_t& outWhoAmI) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(kRegWhoAmI);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)1, (uint8_t) true) != 1) return false;
  if (!Wire.available()) return false;
  outWhoAmI = Wire.read();
  return true;
}

void ImuManager::rawWakeAndConfigure() {
  writeReg(kRegPwrMgmt1, 0x00);  // вийти зі sleep-режиму (за замовчуванням після POR)
  delay(50);
  writeReg(kRegGyroConfig, kGyroConfig500Dps);
  writeReg(kRegAccelConfig, kAccelConfig8G);
  // На відміну від Adafruit-режиму (де setFilterBandwidth виставляє це),
  // raw-режим досі не мав DLPF взагалі - за замовчуванням чіп працює
  // широкосмугово (DLPF_CFG=0, ~256Гц), тобто без фільтрації вібрації від
  // гвинтів. На реальному польоті з гвинтами це шумний вхід для Mahony.
  writeReg(kRegConfig, kDlpfCfg21Hz);
}

bool ImuManager::rawRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(kRegAccelXoutH);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)14, (uint8_t) true) != 14) return false;

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t rawAx = (buf[0] << 8) | buf[1];
  int16_t rawAy = (buf[2] << 8) | buf[3];
  int16_t rawAz = (buf[4] << 8) | buf[5];
  // buf[6..7] - температура, не використовується
  int16_t rawGx = (buf[8] << 8) | buf[9];
  int16_t rawGy = (buf[10] << 8) | buf[11];
  int16_t rawGz = (buf[12] << 8) | buf[13];

  ax = (rawAx / kAccelSensitivityLsbPerG) * kGravityMs2;
  ay = (rawAy / kAccelSensitivityLsbPerG) * kGravityMs2;
  az = (rawAz / kAccelSensitivityLsbPerG) * kGravityMs2;
  gx = (rawGx / kGyroSensitivityLsbPerDps) * DEG_TO_RAD;
  gy = (rawGy / kGyroSensitivityLsbPerDps) * DEG_TO_RAD;
  gz = (rawGz / kGyroSensitivityLsbPerDps) * DEG_TO_RAD;
  return true;
}

void ImuManager::scanI2CBus() {
  Serial.println("Сканування I2C шини...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  пристрій на адресі 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  жодного I2C пристрою не знайдено - перевір VCC/GND/SDA/SCL");
  }
}

bool ImuManager::readSample(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  if (_rawMode) return rawRead(ax, ay, az, gx, gy, gz);

  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  return true;
}

bool ImuManager::begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);  // інакше зависла I2C-шина блокує loop() назавжди
  delay(50);

  if (_mpu.begin(MPU6050_I2C_ADDR, &Wire)) {
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    _rawMode = false;
    Serial.println("MPU6050 OK (Adafruit)");
  } else {
    // Adafruit_MPU6050::begin() відмовляє, якщо WHO_AM_I != 0x68 - типово
    // для дешевих GY-521 клонів (MPU6500 та ін. під тим самим корпусом).
    // Пробуємо напряму: якщо чіп хоч якось відповідає на I2C - працюємо з
    // ним через сирі регістри в обхід бібліотеки.
    uint8_t whoAmI = 0;
    if (rawProbe(whoAmI)) {
      Serial.printf(
          "MPU6050: Adafruit-бібліотека відмовила (WHO_AM_I=0x%02X, ймовірно клон-чіп) - "
          "переходжу на прямий доступ через Wire\n",
          whoAmI);
      rawWakeAndConfigure();
      _rawMode = true;
    } else {
      Serial.println("MPU6050 не знайдено на I2C (немає ACK на адресі 0x68)");
      scanI2CBus();
      return false;
    }
  }

  configureInterrupt();
  delay(20);  // за замовчуванням чіп сампрлить на 1кГц - 20мс достатньо для хоч 1 переривання
  if (sMpuDataReady) {
    _interruptSeen = true;
    sMpuDataReady = false;
    Serial.printf("MPU6050 INT (GP%d) OK - data-ready сигнал надходить\n", PIN_MPU_INT);
  } else {
    Serial.printf(
        "УВАГА: MPU6050 INT (GP%d) не отримано - перевір підключення INT-піна модуля. "
        "Не критично: update() все одно працює полінгом, без INT просто немає цієї діагностики.\n",
        PIN_MPU_INT);
  }

  Serial.println("MPU6050 OK, калібрування gyro + level trim (не рухати, дрон рівно)...");
  recalibrate();

  return true;
}

void ImuManager::configureInterrupt() {
  writeReg(kRegIntPinCfg, 0x00);  // active-high, push-pull, 50мкс імпульс (без latch)
  writeReg(kRegIntEnable, 0x01);  // DATA_RDY_EN

  pinMode(PIN_MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), onMpuDataReady, RISING);
}

bool ImuManager::waitForStill() {
  // Використовуємо VARIANCE (розкид) у ковзному вікні, а не magnitude.
  // Причина (діагноз 2026-07-29): raw-magnitude містить bias чіпа (~2°/с
  // навіть у спокої на нашому клоні), тому magnitude > 1.5°/с завжди й
  // waitForStill ніколи не спрацьовував. Variance ж не залежить від bias -
  // вона нульова, поки дрон реально не рухається, і росте при обертанні.
  // Це "чи РУХАЄТЬСЯ дрон зараз?" замість "чи gyro-відлік малий?".
  const int kWindow = 20;  // 20 семплів * ~10мс = ~200мс вікно
  float bufX[kWindow] = {0}, bufY[kWindow] = {0}, bufZ[kWindow] = {0};
  int idx = 0, count = 0;
  unsigned long start = millis();
  while (millis() - start < CALIB_STILL_MAX_WAIT_MS) {
    float ax, ay, az, gx, gy, gz;
    if (readSample(ax, ay, az, gx, gy, gz)) {
      bufX[idx] = gx; bufY[idx] = gy; bufZ[idx] = gz;
      idx = (idx + 1) % kWindow;
      if (count < kWindow) count++;

      if (count == kWindow) {
        // std-dev по кожній осі у вікні
        double sX = 0, sY = 0, sZ = 0, sqX = 0, sqY = 0, sqZ = 0;
        for (int i = 0; i < kWindow; i++) {
          sX += bufX[i]; sY += bufY[i]; sZ += bufZ[i];
          sqX += bufX[i] * bufX[i]; sqY += bufY[i] * bufY[i]; sqZ += bufZ[i] * bufZ[i];
        }
        double varX = sqX / kWindow - (sX / kWindow) * (sX / kWindow);
        double varY = sqY / kWindow - (sY / kWindow) * (sY / kWindow);
        double varZ = sqZ / kWindow - (sZ / kWindow) * (sZ / kWindow);
        // Максимальне std-dev (у рад/с) переводимо в град/с
        double maxStdDps = sqrt(max(max(varX, varY), varZ)) * RAD_TO_DEG;
        if (maxStdDps < CALIB_STILL_GYRO_THRESHOLD_DPS) return true;
      }
    }
    delay(10);
  }
  return false;  // не дочекались
}

bool ImuManager::calibrateGyro() {
  double sumX = 0, sumY = 0, sumZ = 0;
  double sumSqX = 0, sumSqY = 0, sumSqZ = 0;
  int okSamples = 0;

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readSample(ax, ay, az, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      sumSqX += gx * gx;
      sumSqY += gy * gy;
      sumSqZ += gz * gz;
      okSamples++;
    }
    delay(2);
  }

  if (okSamples < GYRO_CALIBRATION_SAMPLES / 2) {
    Serial.println("  gyro-калібрування: замало валідних I2C-семплів (шина?)");
    return false;
  }

  double meanX = sumX / okSamples;
  double meanY = sumY / okSamples;
  double meanZ = sumZ / okSamples;
  // std-dev у рад/с, переводимо в град/с для порівняння з порогом.
  double varX = sumSqX / okSamples - meanX * meanX;
  double varY = sumSqY / okSamples - meanY * meanY;
  double varZ = sumSqZ / okSamples - meanZ * meanZ;
  double maxVarDps = sqrt(max(max(varX, varY), varZ)) * RAD_TO_DEG;

  if (maxVarDps > CALIB_MAX_GYRO_VARIANCE_DPS) {
    Serial.printf("  gyro-калібрування: variance=%.2f°/с > поріг %.2f°/с - рух виявлено, старий bias НЕ переписано\n",
                  maxVarDps, (double)CALIB_MAX_GYRO_VARIANCE_DPS);
    return false;
  }

  _gyroBiasX = meanX;
  _gyroBiasY = meanY;
  _gyroBiasZ = meanZ;
  Serial.printf("  gyro-bias: X=%+.3f Y=%+.3f Z=%+.3f rad/s (variance %.2f°/с OK)\n",
                _gyroBiasX, _gyroBiasY, _gyroBiasZ, maxVarDps);
  return true;
}

// Madgwick's implementation of Mayhony's AHRS algorithm, портовано з
// ESP-Drone sensfusion6.c 1:1 (тільки перейменовано на member-змінні).
void ImuManager::mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                               float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * DEG_TO_RAD;
  gy = gy * DEG_TO_RAD;
  gz = gz * DEG_TO_RAD;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = _qx * _qz - _qw * _qy;
    halfvy = _qw * _qx + _qy * _qz;
    halfvz = _qw * _qw - 0.5f + _qz * _qz;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (kTwoKi > 0.0f) {
      _integralFBx += kTwoKi * halfex * dt;
      _integralFBy += kTwoKi * halfey * dt;
      _integralFBz += kTwoKi * halfez * dt;
      gx += _integralFBx;
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f;
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    gx += kTwoKp * halfex;
    gy += kTwoKp * halfey;
    gz += kTwoKp * halfez;
  }

  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = _qw;
  qb = _qx;
  qc = _qy;
  _qw += (-qb * gx - qc * gy - _qz * gz);
  _qx += (qa * gx + qc * gz - _qz * gy);
  _qy += (qa * gy - qb * gz + _qz * gx);
  _qz += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0f / sqrtf(_qw * _qw + _qx * _qx + _qy * _qy + _qz * _qz);
  _qw *= recipNorm;
  _qx *= recipNorm;
  _qy *= recipNorm;
  _qz *= recipNorm;
}

bool ImuManager::runFusionStep(float dtSeconds) {
  float ax, ay, az, gx, gy, gz;
  if (!readSample(ax, ay, az, gx, gy, gz)) return false;

  _gyroXDps = (gx - _gyroBiasX) * RAD_TO_DEG;
  _gyroYDps = (gy - _gyroBiasY) * RAD_TO_DEG;
  _gyroZDps = (gz - _gyroBiasZ) * RAD_TO_DEG;

  mahonyUpdate(_gyroXDps, _gyroYDps, _gyroZDps, ax, ay, az, dtSeconds);

  // sensfusion6GetEulerRPY: кут через оцінений напрямок гравітації з quaternion.
  float gravX = 2.0f * (_qx * _qz - _qw * _qy);
  float gravY = 2.0f * (_qw * _qx + _qy * _qz);
  float gravZ = _qw * _qw - _qx * _qx - _qy * _qy + _qz * _qz;

  if (gravX > 1.0f) gravX = 1.0f;
  if (gravX < -1.0f) gravX = -1.0f;

  _yawDeg = atan2f(2.0f * (_qw * _qz + _qx * _qy), _qw * _qw + _qx * _qx - _qy * _qy - _qz * _qz) *
            RAD_TO_DEG;
  _pitchDeg = asinf(gravX) * RAD_TO_DEG;  // "Pitch seems to be inverted" - коментар оригіналу
  _rollDeg = atan2f(gravY, gravZ) * RAD_TO_DEG;
  return true;
}

void ImuManager::update(float dtSeconds) {
  if (sMpuDataReady) {
    _interruptSeen = true;
    sMpuDataReady = false;
  }

  if (!runFusionStep(dtSeconds)) {
    _consecutiveI2cFailures++;
    if (_consecutiveI2cFailures == I2C_FAILURE_DISARM_THRESHOLD) {
      Serial.printf("УВАГА: MPU6050 I2C не відповідає %d разів поспіль - можлива проблема шини\n",
                    _consecutiveI2cFailures);
    }
    return;  // лишаємо попередні _pitchDeg/_rollDeg/_yawDeg, не оновлюємо "сміттям"
  }
  _consecutiveI2cFailures = 0;
}

bool ImuManager::captureLevelTrim() {
  _qw = 1.0f;
  _qx = 0.0f;
  _qy = 0.0f;
  _qz = 0.0f;
  _integralFBx = 0.0f;
  _integralFBy = 0.0f;
  _integralFBz = 0.0f;

  const float dtStep = 0.005f;  // 5мс крок
  double sumSq = 0;
  int okSamples = 0;
  for (int i = 0; i < 200; i++) {  // ~1с на збіжність фільтра
    if (runFusionStep(dtStep)) {
      float magDps = sqrtf(_gyroXDps * _gyroXDps + _gyroYDps * _gyroYDps + _gyroZDps * _gyroZDps);
      sumSq += magDps * magDps;
      okSamples++;
    }
    delay(5);
  }
  if (okSamples < 100) {
    Serial.println("  level-trim: замало валідних I2C-семплів (шина?)");
    return false;
  }
  double rmsDps = sqrt(sumSq / okSamples);
  if (rmsDps > CALIB_MAX_GYRO_VARIANCE_DPS * 2.0f) {  // level trim менш строгий, вдвічі більший поріг
    Serial.printf("  level-trim: rms=%.2f°/с - рух виявлено, старий trim НЕ переписано\n",
                  rmsDps);
    return false;
  }

  _pitchTrimDeg = _pitchDeg;
  _rollTrimDeg = _rollDeg;
  Serial.printf("  level-trim: pitch0=%+.2f° roll0=%+.2f° (rms %.2f°/с OK)\n",
                _pitchTrimDeg, _rollTrimDeg, rmsDps);
  return true;
}

void ImuManager::recalibrate() {
  Serial.println("  калібрування IMU...");
  if (!waitForStill()) {
    Serial.printf("  УВАГА: не вдалось дочекатись спокою за %dмс - продовжую все одно\n",
                  CALIB_STILL_MAX_WAIT_MS);
  }
  bool gyroOk = calibrateGyro();
  bool trimOk = captureLevelTrim();
  if (gyroOk && trimOk) {
    Serial.println("  калібрування завершено (обидві фази OK)");
  } else {
    Serial.println("  калібрування завершено ЧАСТКОВО - деякі значення лишились попередні");
  }
}
