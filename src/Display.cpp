#include "Display.h"

#ifdef USE_DISPLAY
#include <Wire.h>
#include <vector>
#include "SerialLogger.h"
#include "sensors/MotionSensor.h"
#include "sensors/Sensor.h"

#ifdef SH1106
  #include <Adafruit_SH110X.h>
#else
  #include <Adafruit_SSD1306.h>
#endif

namespace {
constexpr int SCREEN_WIDTH = 128;
constexpr int DISPLAY_TEXT_SIZE = 1;
constexpr uint8_t DISPLAY_ADDRESS = 0x3C;
#ifdef SH1106
constexpr size_t DISPLAY_ROW_COUNT = 8;
Adafruit_SH1106G display(SCREEN_WIDTH, 64, &Wire, -1);
#else
constexpr size_t DISPLAY_ROW_COUNT = 4;
Adafruit_SSD1306 display(SCREEN_WIDTH, 32, &Wire, -1);
#endif

void printCommunicationErrorLines(int errorCount)
{
  display.println("COMMUNICATION ERROR");
  display.println("Count: " + String(errorCount));
}
}

void initDisplay()
{
  #ifdef SH1106
    display.begin(DISPLAY_ADDRESS, true);
  #else
    if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;);
    }
  #endif

  Logger.Info("DISPLAY INITED");
  delay(2000);
  Logger.Info("DISPLAY INITED. WILL PRINT");
  clearDisplay();
  display.setTextSize(DISPLAY_TEXT_SIZE);
  #ifdef SH1106
    display.setTextColor(SH110X_WHITE);
  #else
    display.setTextColor(SSD1306_WHITE);
  #endif
  display.println("App initing...");
  display.display();
  delay(2000);
}

void clearDisplay()
{
  display.clearDisplay();
  display.setCursor(0, 0);
}

void showCommunicationError(int errorCount)
{
  clearDisplay();
  printCommunicationErrorLines(errorCount);
  display.display();
}

void showReboot(bool hasCommunicationErrors, int errorCount)
{
  clearDisplay();
  display.println("REBOOTING");
  if (hasCommunicationErrors) {
    printCommunicationErrorLines(errorCount);
  }
  display.display();
}

void printSensorReading(int sensorId, float temperature, float humidity, float light, int motion)
{
  String message = String(sensorId) + ": ";
  if (temperature == Sensor::ERROR_FAILED_READING) {
    message += "ERR C";
  } else if (temperature != Sensor::ERROR_UNSUPPORTED) {
    message += String(temperature) + " C";
  }

  if (humidity == Sensor::ERROR_FAILED_READING) {
    message += ", ERR %";
  } else if (humidity != Sensor::ERROR_UNSUPPORTED) {
    message += ", " + String(humidity) + " %";
  }

  if (light == Sensor::ERROR_FAILED_READING) {
    message += "ERR lx";
  } else if (light != Sensor::ERROR_UNSUPPORTED) {
    message += String(light) + " lx";
  }

  if (motion >= 0) {
    message += motion ? "ON" : "OFF";
  }

  display.println(message);
}

void showMeasurementFooter(size_t sensorCount, int measureCount, int measureLimit,
                           unsigned int sentMessageCount, MotionSensor* motionSensor)
{
  size_t rowCount = sensorCount;
  if (motionSensor != nullptr) {
    float delayInSeconds = motionSensor->getOutputDelayLeft();
    if (delayInSeconds > 0) {
      display.println("ON DELAY (s): " + String(delayInSeconds));
      rowCount++;
    }
  }

  if (rowCount < DISPLAY_ROW_COUNT) {
    display.println("Measure count: " + String(measureCount) + "/" + String(measureLimit));
    rowCount++;
  }

  if (rowCount < DISPLAY_ROW_COUNT) {
    display.println("Sent messages: " + String(sentMessageCount));
  }

  display.display();
}
#endif
