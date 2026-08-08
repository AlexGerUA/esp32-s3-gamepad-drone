#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <Adafruit_NeoPixel.h>

#include "Config.h"

// Керує вбудованим адресним RGB LED (WS2818) на GP48 плати Super Mini.
// Не з ESP-Drone - окрема утиліта для видимого стану без serial-монітора.
class StatusIndicator {
 public:
  void begin();

  // Викликати кожен тік control loop.
  void update(bool imuOk, bool gamepadConnected, bool armed, unsigned long nowMs);

 private:
  Adafruit_NeoPixel _pixel{STATUS_LED_COUNT, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800};
};

#endif  // STATUS_INDICATOR_H
