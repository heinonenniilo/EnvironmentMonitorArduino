#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include "Sensor.h"
#include <Arduino.h>
#include <initializer_list>

#define OUTPUT_MAX_DELAY_MS 1200000
#define OUTPUT_MIN_DELAY_MS 10000
#define MOTION_CONTROL_DEFAULT_DELAY_MS 30000

enum MotionControlStatus {
    AlwaysOff = 0,
    AlwaysOn = 1,
    MotionControl = 2
};

class MotionSensor : public Sensor {
private:
  std::vector<uint8_t> inPins;
  std::vector<uint8_t> outPins;
  bool motionDetectedTelemetry = 0; // Aggregated value, for telemetry
  bool motionDetectedDisplay = 0; // Aggregated for display
  bool multiTriggerMode = 1;
  // Status
  bool lastMotionStatus = 0; 
  unsigned long lastMotionOnMillis = 0;
  // Control
  MotionControlStatus motionControlStatus = MotionControl;
  unsigned long motionControlDelaysMs = MOTION_CONTROL_DEFAULT_DELAY_MS;

  bool readCurrentMotion();
  void setOutputs(bool mode);
public:
    MotionSensor(int sensorId, std::initializer_list<uint8_t> inPinsList, std::initializer_list<uint8_t> outPinsList, bool multiTriggerMode);

    void begin() override;
    int readMotion(bool aggregate) override;
    void resetAverages() override;

    float getOutputDelayLeft();

    void checkOutputs(); 
    bool setMotionControlStatus(MotionControlStatus status); // Returns true if changed
    bool setMotionControlDelay(unsigned long delayInMs); // retursn true if changed
};

#endif