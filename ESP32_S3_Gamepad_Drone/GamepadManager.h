#ifndef GAMEPAD_MANAGER_H
#define GAMEPAD_MANAGER_H

#include <Arduino.h>

// Обгортка над Bluepad32 для Xbox One S контролера.
// Мапінг - FPS-стиль (2026-07-29):
//   RT (правий тригер)         -> throttle, 0..1 (не самоцентрується)
//   Лівий стік Y (вгору)       -> pitch (вперед), -1..1
//   Лівий стік X (вправо)      -> roll (вправо),  -1..1
//   Правий стік X              -> yawRate, -1..1
//   Правий стік Y              -> НЕ використовується
//   L1 + R1 утримати            -> запит на arm (перевірка throttle/таймінгу - в .ino)
//   B                           -> миттєвий запит на disarm
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
