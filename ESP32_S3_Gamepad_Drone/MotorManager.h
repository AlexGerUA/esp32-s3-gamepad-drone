#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "StabilizationController.h"

// LEDC PWM на 4 канали -> gate SI2300 MOSFET (кожен з flyback-діодом і
// pulldown резистором на платі, поза кодом).
class MotorManager {
 public:
  void begin();

  // outputs.* очікуються в діапазоні 0..1
  void setMotors(const MotorOutputs& outputs);

  // Миттєвий примусовий стоп (failsafe/disarm).
  void disableAll();
};

#endif  // MOTOR_MANAGER_H
