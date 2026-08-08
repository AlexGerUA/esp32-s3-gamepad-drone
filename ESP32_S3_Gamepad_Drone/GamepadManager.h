#ifndef GAMEPAD_MANAGER_H
#define GAMEPAD_MANAGER_H

#include <Arduino.h>


class GamepadManager {
 public:
  struct State {
    bool connected = false;
    unsigned long lastUpdateMs = 0;

    float throttle = 0.0f;  // 0..1
    float pitch = 0.0f;     // -1..1
    float roll = 0.0f;      // -1..1
    float yawRate = 0.0f;   // -1..1

    bool armButtonsHeld = false;   // L1 && R1 наразі затиснуті
    bool disarmButtonPressed = false;  // B наразі затиснута
  };

  void begin();
  void update();

  const State& state() const { return _state; }

 private:
  State _state;
};

#endif  // GAMEPAD_MANAGER_H
