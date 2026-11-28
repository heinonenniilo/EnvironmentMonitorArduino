#pragma once

#include "configs/pins.h"

#ifdef USE_DISPLAY
  #include <stddef.h>

  class MotionSensor;

  void initDisplay();
  void clearDisplay();
  void showCommunicationError(int errorCount);
  void showReboot(bool hasCommunicationErrors, int errorCount);
  void printSensorReading(int sensorId, float temperature, float humidity, float light, int motion);
  void showMeasurementFooter(size_t sensorCount, int measureCount, int measureLimit,
                             unsigned int sentMessageCount, MotionSensor* motionSensor = nullptr);
#endif
