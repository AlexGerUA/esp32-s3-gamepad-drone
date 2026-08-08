#ifndef CONFIG_H
#define CONFIG_H

// ===== Плата =====
// ESP32-S3 "Super Mini" (генерик, не Seeed XIAO). FQBN для збірки:
//   esp32-bluepad32:esp32:esp32s3
// GP48 на цій платі НАЗАВЖДИ зайнятий вбудованим адресним RGB LED (WS2818) —
// не використовувати під щось інше.

// ===== I2C (MPU6050) =====
// Дефолтні Wire-піни ESP32-S3 Arduino core.
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define MPU6050_I2C_ADDR 0x68

// Без таймауту зависла I2C-шина (напр. відірваний контакт SDA/SCL посеред
// транзакції) блокує Wire-виклик НАЗАВЖДИ - весь loop() стає, мотори
// застигають на останньому значенні. Це реально сталось на стенді
// (2026-07-27/28, ручний тест з нахилами) - плата повністю зависала,
// допомагало лише живлення вимкнути/увімкнути. I2C_TIMEOUT_MS змушує
// Wire-виклики повертати помилку замість вічного очікування.
#define I2C_TIMEOUT_MS 50
// Скільки підряд невдалих I2C-читань під час update() терпимо, перш ніж
// вважати IMU "нездоровим" і форсувати disarm (як при втраті геймпада).
// При 250Гц циклі 2 пропущені зразки = 8мс - миттєво (як просив користувач).
#define I2C_FAILURE_DISARM_THRESHOLD 2

// Data-ready переривання MPU6050 (INT пін модуля). GP10 - вільний, поруч з
// I2C-кластером (GP8/9). Використовується лише як діагностика при старті
// (підтвердити, що дріт підключений і чіп реально сигналізує) - основний
// цикл лишається на полінгу (безпечніше під час bench-тестів: жодного
// ризику зависання, якщо переривання раптом не прийде).
#define PIN_MPU_INT 10

// ===== Мотори (LEDC PWM -> gate SI2300 MOSFET) =====
// GP1-GP7 (ліва сторона плати) і GP8-GP13 (права сторона плати) - пара
// рядів пінів по обидва боки чипа, ближче до центру плати (не GP14-48, ті
// в іншому куті, ближче до battery-конектора). 2 мотори зліва, 2 справа -
// дроти до лівих/правих моторів рами йдуть кожен зі свого боку.
// GP8/GP9 зайняті під I2C (MPU6050), тому праві мотори - GP12/GP13.
// GP1/GP2 лишають GP3-GP7 (ADC1) вільними під майбутній BatteryMonitor.
// Заміни піни під свою розводку, якщо відрізняється.
#define PIN_MOTOR_FL 1   // ліва сторона плати
#define PIN_MOTOR_RL 2   // ліва сторона плати
#define PIN_MOTOR_FR 12  // права сторона плати
#define PIN_MOTOR_RR 13  // права сторона плати

#define MOTOR_PWM_FREQ_HZ 16000  // вище чутного діапазону писку коростких моторів
#define MOTOR_PWM_RESOLUTION_BITS 10
#define MOTOR_PWM_MAX ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1)

// ===== Батарея (поки не реалізовано) =====
// GP4 (ADC) лишається вільним на майбутнє.

// ===== Статус-індикатор (вбудований WS2818 RGB на платі Super Mini) =====
#define PIN_STATUS_LED 48
#define STATUS_LED_COUNT 1
#define STATUS_LED_BRIGHTNESS 40  // 0-255, притлумлено щоб не сліпило

// ===== Діагностика (Serial, 115200 бод) =====
// Поки armed - детальна телеметрія (throttle/кути/мотори) з цим інтервалом,
// потрібна для bench-тюнінгу.
// Поки disarmed + gamepad connected - TEST MODE: подетальна розкладка з
// підписами (що який стік/кнопка робить) - для калібрування розкладки.
// Поки disarmed + no gamepad - раз на STATUS_HEARTBEAT короткий heartbeat.
#define DEBUG_PRINT_INTERVAL_MS 200
#define TEST_MODE_INTERVAL_MS 250
#define STATUS_HEARTBEAT_INTERVAL_MS 60000  // 1 хв, коли зовсім тихо

// ===== Контрольний цикл =====
#define CONTROL_LOOP_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_HZ)

// ===== Безпека / arm-disarm =====
// 100мс було занадто агресивно: Bluepad32/BT-з'єднання на практиці дає
// джитер >100мс навіть при нормальному сигналі, failsafe спрацьовував
// кожні кілька секунд посеред штатного керування. 300мс - робочий баланс
// між "падає швидко при реальному розриві" і "не спрацьовує на джитері".
#define GAMEPAD_TIMEOUT_MS 300
#define ARM_HOLD_MS 500            // скільки тримати L1+R1 щоб armed
#define ARM_THROTTLE_DEADBAND 0.05f  // throttle має бути ~0 щоб дозволити arm

// ===== Стіки =====
#define STICK_DEADZONE 0.05f

// ===== Калібрування IMU: wait-for-still + variance check =====
// Спостереження (2026-07-29): фреш-boot калібрування дає кращий результат,
// ніж після DISARM, бо після польоту повітряні потоки й вібрація рами ще
// затухають. Тому перед калібруванням чекаємо, поки рух реально не спаде.
#define CALIB_STILL_MAX_WAIT_MS 3000      // максимум чекаємо на спокій
#define CALIB_STILL_GYRO_THRESHOLD_DPS 1.5f  // рух > 1.5°/с = не спокій
#define CALIB_MAX_GYRO_VARIANCE_DPS 0.5f   // якщо стандартне відхилення >, вважаємо, що рухали

// ===== PID-гейни: каскад angle -> rate, за структурою ESP-Drone =====
// (attitude_pid_controller.c: зовнішній angle-PID видає бажану кутову
// швидкість (град/с), внутрішній rate-PID порівнює її з виміряною gyro-
// швидкістю і видає сирий duty-вихід на мотор). Числа тут - ВЛАСНІ, свідомо
// консервативні (НЕ скопійовані з ESP-Drone pid.h) - той код калібрований
// під безщітковий Crazyflie-подібний дрон з ESC у шкалі 0-65535, тут для
// довідки в коментарях. Обов'язково тюнити на стенді без гвинтів.
// !!! Потребують ручного тюнінгу на стенді без гвинтів перед реальним польотом.

// --- Angle loop (вхід: похибка кута, град; вихід: бажана швидкість, град/с) ---
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

// --- Rate loop (вхід: похибка швидкості, град/с; вихід: сирий duty на мотор) ---
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

// Мінімальний "холостий" duty, поки armed і throttle > 0 (у ESP-Drone
// utримує гвинти безщіткового ESC на мінімальних обертах для відгуку;
// на щіткових моторах через MOSFET не потрібен - 0 = вимкнено).
#define IDLE_THRUST 0

// Стік pitch/roll (-1..1) масштабується у цільовий кут (як setpoint->attitude
// у ESP-Drone); yaw стік інтегрується у ціль по куту з максимальною
// швидкістю обертання нижче (як їхній modeVelocity для yaw) - тримає heading
// коли стік відпущено, замість повзучого дрейфу прямого rate-control.
#define MAX_TILT_ANGLE_DEG 25.0f
#define MAX_YAW_RATE_DEG_S 180.0f

// ===== Mahony AHRS (sensfusion6.c з ESP-Drone) =====
#define MAHONY_TWO_KP (2.0f * 0.4f)    // ESP-Drone sensfusion6.c: TWO_KP_DEF
#define MAHONY_TWO_KI (2.0f * 0.001f)  // ESP-Drone sensfusion6.c: TWO_KI_DEF
#define GYRO_CALIBRATION_SAMPLES 500   // ~1-2с на старті, дрон має стояти нерухомо

#endif  // CONFIG_H
