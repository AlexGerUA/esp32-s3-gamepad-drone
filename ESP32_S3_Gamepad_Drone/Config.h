#ifndef CONFIG_H
#define CONFIG_H


// GP48 на цій платі НАЗАВЖДИ зайнятий вбудованим адресним RGB LED (WS2818) —

#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define MPU6050_I2C_ADDR 0x68


#define I2C_TIMEOUT_MS 50

#define I2C_FAILURE_DISARM_THRESHOLD 2

#define PIN_MPU_INT 10


#define PIN_MOTOR_FL 1   // ліва сторона плати
#define PIN_MOTOR_RL 2   // ліва сторона плати
#define PIN_MOTOR_FR 12  // права сторона плати
#define PIN_MOTOR_RR 13  // права сторона плати

#define MOTOR_PWM_FREQ_HZ 16000  // вище чутного діапазону писку коростких моторів
#define MOTOR_PWM_RESOLUTION_BITS 10
#define MOTOR_PWM_MAX ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1)


#define PIN_STATUS_LED 48
#define STATUS_LED_COUNT 1
#define STATUS_LED_BRIGHTNESS 40  // 0-255, притлумлено щоб не сліпило


#define DEBUG_PRINT_INTERVAL_MS 200
#define TEST_MODE_INTERVAL_MS 250
#define STATUS_HEARTBEAT_INTERVAL_MS 60000  // 1 хв, коли зовсім тихо

#define CONTROL_LOOP_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_HZ)


#define GAMEPAD_TIMEOUT_MS 300
#define ARM_HOLD_MS 500            // скільки тримати L1+R1 щоб armed
#define ARM_THROTTLE_DEADBAND 0.05f  // throttle має бути ~0 щоб дозволити arm

// ===== Стіки =====
#define STICK_DEADZONE 0.05f


#define CALIB_STILL_MAX_WAIT_MS 3000      // максимум чекаємо на спокій
#define CALIB_STILL_GYRO_THRESHOLD_DPS 1.5f  // рух > 1.5°/с = не спокій
#define CALIB_MAX_GYRO_VARIANCE_DPS 0.5f   // якщо стандартне відхилення >, вважаємо, що рухали


#define PID_ROLL_KP 3.0f   // ESP-Drone pid.h: 5.9
#define PID_ROLL_KI 0.0f   // ESP-Drone pid.h: 2.9
#define PID_ROLL_KD 0.0f
#define PID_ROLL_INTEGRATION_LIMIT 20.0f  // град/с

#define PID_PITCH_KP 3.0f  // ESP-Drone pid.h: 5.9
#define PID_PITCH_KI 0.0f  // ESP-Drone pid.h: 2.9
#define PID_PITCH_KD 0.0f
#define PID_PITCH_INTEGRATION_LIMIT 20.0f  // град/с

#define PID_YAW_KP 2.0f    // ESP-Drone pid.h: 6.0
#define PID_YAW_KI 0.0f    // ESP-Drone pid.h: 1.0
#define PID_YAW_KD 0.0f    // ESP-Drone pid.h: 0.35
#define PID_YAW_INTEGRATION_LIMIT 30.0f  // град/с

#define ANGLE_PID_OUTPUT_LIMIT 300.0f  // град/с, стеля для всіх 3 angle-PID


#define PID_ROLL_RATE_KP 3.0f    // ESP-Drone pid.h: 250 (їхня шкала 0-65535 vs наша 0-1023)
#define PID_ROLL_RATE_KI 0.0f    // ESP-Drone pid.h: 500
#define PID_ROLL_RATE_KD 0.02f   // ESP-Drone pid.h: 2.5
#define PID_ROLL_RATE_INTEGRATION_LIMIT 50.0f

#define PID_PITCH_RATE_KP 3.0f   // ESP-Drone pid.h: 250
#define PID_PITCH_RATE_KI 0.0f   // ESP-Drone pid.h: 500
#define PID_PITCH_RATE_KD 0.02f  // ESP-Drone pid.h: 2.5
#define PID_PITCH_RATE_INTEGRATION_LIMIT 50.0f

#define PID_YAW_RATE_KP 2.0f     // ESP-Drone pid.h: 120
#define PID_YAW_RATE_KI 0.0f     // ESP-Drone pid.h: 16.7
#define PID_YAW_RATE_KD 0.0f
#define PID_YAW_RATE_INTEGRATION_LIMIT 50.0f

#define RATE_PID_OUTPUT_LIMIT (MOTOR_PWM_MAX * 0.5f)  // сирі duty-одиниці


#define IDLE_THRUST 0

#define MAX_TILT_ANGLE_DEG 25.0f
#define MAX_YAW_RATE_DEG_S 180.0f

// ===== Mahony AHRS (sensfusion6.c з ESP-Drone) =====
#define MAHONY_TWO_KP (2.0f * 0.4f)    // ESP-Drone sensfusion6.c: TWO_KP_DEF
#define MAHONY_TWO_KI (2.0f * 0.001f)  // ESP-Drone sensfusion6.c: TWO_KI_DEF
#define GYRO_CALIBRATION_SAMPLES 500   // ~1-2с на старті, дрон має стояти нерухомо

#endif  // CONFIG_H
