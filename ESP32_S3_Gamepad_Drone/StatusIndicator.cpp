#include "StatusIndicator.h"

void StatusIndicator::begin() {
  _pixel.begin();
  _pixel.setBrightness(STATUS_LED_BRIGHTNESS);
  _pixel.show();
}

void StatusIndicator::update(bool imuOk, bool gamepadConnected, bool armed, unsigned long nowMs) {
  uint32_t color;

  if (!imuOk) {
    color = _pixel.Color(255, 0, 0);  // MPU6050 не ініціалізувався - arm неможливий
  } else if (armed) {
    color = _pixel.Color(0, 255, 0);
  } else if (!gamepadConnected) {
    bool blinkOn = (nowMs / 300) % 2 == 0;
    color = blinkOn ? _pixel.Color(0, 0, 255) : _pixel.Color(0, 0, 0);
  } else {
    color = _pixel.Color(0, 0, 255);  // disarmed, геймпад підключений
  }

  _pixel.setPixelColor(0, color);
  _pixel.show();
}
