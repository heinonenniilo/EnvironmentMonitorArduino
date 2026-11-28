#include <Arduino.h>
#include <ArduinoJson.h>
#include <atomic>
#include <cstdlib>
#include <string.h>
#include <time.h>
#include <vector>

#include "Display.h"
#include "transports/TransportConfig.h"

#define CONFIG_FREERTOS_NUMBER_OF_CORES 1
#include "esp_task_wdt.h"
#include "esp_system.h"
#include <WiFi.h>

#include "SerialLogger.h"
#include "sensors/BH1750FVISensor.h"
#include "sensors/DS18B20Sensor.h"
#include "sensors/DHT22Sensor.h"
#include "sensors/MotionSensor.h"
#include "sensors/TMP36Sensor.h"

#ifdef RUUVI_MAC
  #include "sensors/RuuviTagScanner.h"
#endif

#include "transports/HttpApi.h"
#include "transports/IoTHub.h"
#include "transports/MqttBroker.h"
#include "transports/TransportEvents.h"

#define DEBUG 0
#define RUUVI_SCAN_TIME 20000 // RUUVI SCAN TIME IN MS

// MEASURE_LOOP_COUNT * MEASURE_LIMIT*LOOP_WAIT = Send Interval in ms
#define LOOP_WAIT 100
#define MEASURE_LOOP_COUNT  40
#define FIRST_MESSAGE_MILLIS 5000
#define SUCCESS_LIMIT 3
#define MEASURE_LIMIT 30

#define MEASURE_START_LOOP_LIMIT 6
#define MOTION_DETECTION_SHUTDOWN_DELAY_MS 45000 // Set to 0 if controlled by sensor

// https://iotassistant.io/esp32/fixing-error-hardware-wdt-arduino-esp32/
#define WDT_TIMEOUT 15000 // 15 s

// Utility macros and defines
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define UNIX_TIME_NOV_13_2017 1510592825

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

enum class MeasurementTypes : int {
  Undefined = 0,
  Temperature = 1,
  Humidity = 2,
  Light = 3,
  Motion = 4,
  Pressure = 5
};

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;

static String telemetry_payload = "{}"; // JSON payload for telemetry data

// Status variables
static std::atomic<int> communicationErrorCount{0};
int successCount = 0;
unsigned int sentMessageCount = 0;
int inited = 0;
int measureCount = 0;
int lastRuuviMeasureCount = 0;
bool hasSentMessage = false;
bool hasInitedRuuvi = false;
static unsigned long loopCount = 0;
static unsigned long lastLoopCount = 0;
int lastMotionStatus = 0;
int deviceStatus = 0; // Current status

MotionControlStatus motionControlStatus = MotionControl;
// Sensors
std::vector<Sensor*> sensors;
MotionSensor* motionSensor;

#ifdef RUUVI_MAC
  BLEScan* pBLEScan; 
  RuuviTagScanner* scanner; 
  std::vector<RuuviTagSensor*> ruuviSensors;
#endif

// Auxiliary functions

static void connectToWiFi() {
  Logger.Info("Connecting to WIFI SSID " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
}

static void initializeTime() {
  Logger.Info("Setting time using SNTP");

  configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
  time_t now = time(NULL);
  while (now < UNIX_TIME_NOV_13_2017) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  Logger.Info("Time initialized!");
}

static String generateIdentifier(unsigned long uptime, unsigned long messageOrder, uint8_t numOfRandParts=2)
{
  char buffer[9];
  String result;
  // Message order / count of sent messages
  sprintf(buffer, "%08X", messageOrder);
  result = String(buffer);
  // Uptime / millis
  sprintf(buffer, "%08X", uptime);
  result += String(buffer);
  for (uint8_t i = 0; i < numOfRandParts; i++) 
  {
    sprintf(buffer, "%08X", esp_random());
    result += String(buffer);
  }

  return result;
}

void setStatus(int statusToSet) {

  deviceStatus = statusToSet;
  if (statusToSet == 0) {
    digitalWrite(REDLEDPIN, HIGH);
    digitalWrite(GREENLEDPIN, LOW);
    digitalWrite(YELLOWLEDPIN, LOW);
  } else if (statusToSet == 1) {
    digitalWrite(REDLEDPIN, LOW);
    digitalWrite(GREENLEDPIN, HIGH);
    digitalWrite(YELLOWLEDPIN, LOW);
  } else if (statusToSet == 2) {
    digitalWrite(GREENLEDPIN, LOW);
    digitalWrite(REDLEDPIN, LOW);
    digitalWrite(YELLOWLEDPIN, LOW);
  } else if (statusToSet == 3)
  {
    digitalWrite(GREENLEDPIN, LOW);
    digitalWrite(REDLEDPIN, LOW);
    digitalWrite(YELLOWLEDPIN, HIGH);
  } else if (statusToSet == 4) // Indicate read attributes
  {
    digitalWrite(GREENLEDPIN, HIGH);
    digitalWrite(REDLEDPIN, LOW);
    digitalWrite(YELLOWLEDPIN, HIGH);
  }
}

static void printCommunicationError()
{
  Logger.Error("COMMUNICATION ERROR");
  Logger.Error("Count: " + String(communicationErrorCount.load()));

  #ifdef USE_DISPLAY
    showCommunicationError(communicationErrorCount.load());
  #endif
}

void recordCommunicationError()
{
  ++communicationErrorCount;
}

void clearCommunicationErrors()
{
  communicationErrorCount = 0;
}

void reportCommunicationError()
{
  recordCommunicationError();
  setStatus(0);
  printCommunicationError();
}

#if defined(USE_IOT_HUB) || defined(USE_MQQT_BROKER)
  #define CMD_REBOOT "REBOOT"

  bool parseMotionControlStatus(const String& message, MotionControlStatus& status);
  bool parseMotionControlDelay(const String& message, unsigned long& delayMs);

  void handleCommand(const char* command)
  {
    if (strcmp(command, CMD_REBOOT) == 0)
    {
      Logger.Info("Reboot command received. Rebooting after 2 s...");
      delay(2000);
      ESP.restart();
    }
    MotionControlStatus status;
    if (parseMotionControlStatus(command, status))
    {
      Logger.Info("Setting motion control status to: " + String(status));
      #ifdef MOTIONSENSOR_IN_PINS
        motionSensor->setMotionControlStatus(status);
      #endif
    }
    unsigned long delay;
    if (parseMotionControlDelay(command, delay))
    {
      Logger.Info("Setting motion control delays");
      #ifdef MOTIONSENSOR_IN_PINS
        Logger.Info("Setting delay to: " + String(delay));
        motionSensor->setMotionControlDelay(delay);
      #endif
    }
  }
#endif

/*
 * @brief           Gets the number of seconds since UNIX epoch until now.
 * @return uint32_t Number of seconds.
 */
static uint32_t getEpochTimeInSecs() {
  return (uint32_t)time(NULL);
}

static void establishConnection() {
  connectToWiFi();
  initializeTime();
  #ifdef USE_IOT_HUB
    IoTHub::begin();
  #endif
  #ifdef USE_MQQT_BROKER
    MqttBroker::begin();
    MqttBroker::service();
  #endif
}

int calculateMeasurements() 
{
  Logger.Info("Calculating measurements");
  float humi = 0;
  float tempC = 0;
  float lightV = 0;
  int motionV = 0;
  bool readingFailed = false;
  uint8_t succeededReadings = 0;
  uint8_t failedReadings = 0;
  
  #ifdef USE_DISPLAY
    clearDisplay();
  #endif

  for (Sensor* sensor : sensors) 
  {
    tempC = sensor->readTemperature(false);
    humi = sensor->readHumidity(false);
    lightV = sensor->readLight(false);
    motionV = sensor->readMotion(false);

    int sensorId = sensor->getSensorId();
    #ifdef USE_DISPLAY
      printSensorReading(sensorId, tempC, humi, lightV, motionV);
    #endif
    if (loopCount <= MEASURE_START_LOOP_LIMIT) 
    {
      sensor->resetAverages();
      Logger.Info("Ignoring measurements. LoopCount: " + String(loopCount));
    }
    if (humi == Sensor::ERROR_FAILED_READING || tempC == Sensor::ERROR_FAILED_READING || lightV == Sensor::ERROR_FAILED_READING) 
    {
      Logger.Error("Reading sensor data failed for Sensor: " + String(sensorId));
      readingFailed = true;
      failedReadings++;
    } else 
    {
      succeededReadings++;
    }
  }
  #ifdef USE_DISPLAY
    MotionSensor* displayMotionSensor = nullptr;
    #if defined(MOTIONSENSOR_IN_PINS) && defined(MOTIONSENSOR_DISPLAY_ON_DELAY)
      displayMotionSensor = motionSensor;
    #endif
    showMeasurementFooter(sensors.size(), measureCount, MEASURE_LIMIT, sentMessageCount, displayMotionSensor);
  #endif
  if (loopCount <= MEASURE_START_LOOP_LIMIT) 
  {
    return 1;
  }

  if (succeededReadings == 0) 
  {
    return 0;
  }
  // With any succeeded reading, allow to continue and increase measurement count to keep sending messages.
  measureCount++;
  Logger.Info("Calculated. Measure count: " + String(measureCount));
  return succeededReadings;
}

static int generateTelemetryPayload(bool sendEmpty = false) 
{
  Logger.Info("--Generating measurement message--");
  Logger.Info("MeasureCount: " + String(measureCount));
  unsigned long millisNow = millis();
  uint32_t randomNumber = esp_random();
  JsonDocument doc;
  String identifier = generateIdentifier(millisNow, sentMessageCount);
  Logger.Info("Message identifier: " + identifier);
  doc["deviceId"] = IOT_CONFIG_DEVICE_ID;
  doc["firstMessage"] = !hasSentMessage; 
  doc["identifier"] = identifier;
  doc["messageCount"] = sentMessageCount;
  doc["loopCount"] = loopCount;
  doc["uptime"] = millisNow;

  if (sendEmpty)
  {
    serializeJson(doc, telemetry_payload);
    Logger.Info("--IOT HUB message generated--");
    return 1;    
  }

  int jsonMeasureCount = 0;
  for (Sensor* sensor : sensors) 
  {
    int sensorId = sensor->getSensorId();
    Logger.Info("SensorId: " + String(sensorId));
    float humi = sensor->readHumidity(true); 
    float tempC = sensor->readTemperature(true);
    float lightV = sensor->readLight(true);
    float pressureV = sensor->readPressure(true);
    int motionV = sensor->readMotion(true);
    if (tempC != Sensor::ERROR_UNSUPPORTED && tempC != Sensor::ERROR_FAILED_READING) 
    {
      Logger.Info("TempC average: " + String(tempC));
      doc["measurements"][jsonMeasureCount]["SensorId"] = sensorId;
      doc["measurements"][jsonMeasureCount]["SensorValue"]= tempC;
      doc["measurements"][jsonMeasureCount]["TypeId"]= (int)MeasurementTypes::Temperature;
      jsonMeasureCount++;
    }
    if (humi != Sensor::ERROR_UNSUPPORTED && humi != Sensor::ERROR_FAILED_READING) 
    {
      Logger.Info("Humidity average: " + String(humi));
      doc["measurements"][jsonMeasureCount]["SensorId"] = sensorId;
      doc["measurements"][jsonMeasureCount]["SensorValue"] = humi;
      doc["measurements"][jsonMeasureCount]["TypeId"]= (int)MeasurementTypes::Humidity;
      jsonMeasureCount++;
    }
    if (lightV != Sensor::ERROR_UNSUPPORTED && lightV != Sensor::ERROR_FAILED_READING) 
    {
      Logger.Info("Light average: " + String(lightV));
      doc["measurements"][jsonMeasureCount]["SensorId"] = sensorId;
      doc["measurements"][jsonMeasureCount]["SensorValue"] = lightV;
      doc["measurements"][jsonMeasureCount]["TypeId"]= (int)MeasurementTypes::Light;
      jsonMeasureCount++;
    }

    if (pressureV != Sensor::ERROR_UNSUPPORTED && pressureV != Sensor::ERROR_FAILED_READING) 
    {
      Logger.Info("Pressure average: " + String(pressureV));
      doc["measurements"][jsonMeasureCount]["SensorId"] = sensorId;
      doc["measurements"][jsonMeasureCount]["SensorValue"] = pressureV;
      doc["measurements"][jsonMeasureCount]["TypeId"]= (int)MeasurementTypes::Pressure;
      jsonMeasureCount++;
    }

    if (motionV >= 0) 
    {
      Logger.Info("Motion detected: " + String(motionV));
      doc["measurements"][jsonMeasureCount]["SensorId"] = sensorId;
      doc["measurements"][jsonMeasureCount]["SensorValue"] = motionV;
      doc["measurements"][jsonMeasureCount]["TypeId"]= (int)MeasurementTypes::Motion;      
      jsonMeasureCount++;
    }
  }
  serializeJson(doc, telemetry_payload);

  measureCount = 0; 
  lastRuuviMeasureCount = 0;
  Logger.Info("--IOT HUB message generated--");
  return 1;
}

#ifdef USE_HTTP_API
  #define ATTRIBUTE_READ_INTERVAL_MS 60000

  static unsigned long lastAttributesReadMillis = 0;
  static int lastDeviceStatus = 0;
  static unsigned long indicationStatusSet = 0;

  static int publishTelemetryToHttpApi()
  {
    int postResult = HttpApi::publishTelemetry(telemetry_payload);
    if (postResult)
    {
      clearCommunicationErrors();
    } else
    {
      reportCommunicationError();
    }
    return postResult;
  }

  static void serviceHttpAttributes()
  {
    if (millis() - lastAttributesReadMillis > ATTRIBUTE_READ_INTERVAL_MS)
    {
      lastAttributesReadMillis = millis();
      Logger.Info("Trying to read attributes");
      int newStatus;
      unsigned long newOnDelayMs;

      if (!HttpApi::readAttributes(newStatus, newOnDelayMs))
      {
        Logger.Error("[ATTR] Failed to read attributes");
        return;
      }

      Logger.Info("[ATTR] Attributes updated.");
      Logger.Info("Status: " + String(newStatus));
      Logger.Info("Delays: " + String(newOnDelayMs));
      #ifdef MOTIONSENSOR_IN_PINS
        bool statusChanged = motionSensor->setMotionControlStatus(static_cast<MotionControlStatus>(newStatus));
        bool delayChanged = motionSensor->setMotionControlDelay(newOnDelayMs);
        if (statusChanged || delayChanged)
        {
          lastDeviceStatus = deviceStatus;
          indicationStatusSet = millis();
          setStatus(4);
        }
      #endif
    } else if (indicationStatusSet > 0 && millis() - indicationStatusSet > 5000)
    {
      Logger.Info("Resetting status indicator");
      indicationStatusSet = 0;
      if (deviceStatus == 4)
      {
        setStatus(lastDeviceStatus);
      }
    }
  }  

#endif

static int sendTelemetry(bool sendEmpty = false) {
  Logger.Info("Sending telemetry ...");

  #ifdef USE_MQQT_BROKER
    if (!MqttBroker::canPublish())
    {
      return 0;
    }
  #endif
  #ifdef USE_IOT_HUB
    if (!IoTHub::prepareTelemetryTopic())
    {
      return 0;
    }
  #endif

  if (!generateTelemetryPayload(sendEmpty))
  {
    Logger.Error("Failed to get sensor data");
    return 0;
  }

  #ifdef USE_IOT_HUB
    return IoTHub::publishTelemetry(telemetry_payload);
  #elif defined(USE_MQQT_BROKER)
    return MqttBroker::publishTelemetry(telemetry_payload);
  #elif defined(USE_HTTP_API)
    return publishTelemetryToHttpApi();
  #endif
}

bool parseMotionControlStatus(const String& message, MotionControlStatus& status) {
    const String prefix = "MOTIONCONTROLSTATUS:";
    int index = message.indexOf(prefix); 
    if (index != -1) {
        String statusValue = message.substring(index + prefix.length());
        int statusInt = statusValue.toInt(); 
        if (statusInt >= AlwaysOff && statusInt <= MotionControl) {
            status = static_cast<MotionControlStatus>(statusInt); 
            return true; 
        }
    }
    return false; 
}

bool parseMotionControlDelay(const String& message, unsigned long& delayMs) {
    const String prefix = "MOTIONCONTROLDELAY:";
    int index = message.indexOf(prefix); // Find the prefix in the message
    if (index != -1) {
        String delayValue = message.substring(index + prefix.length());
        unsigned long delayInt = delayValue.toInt(); // Convert to an integer

        if (delayInt > 0) {
            delayMs = delayInt;
            return true;
        }
    }
    return false; 
}

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = true,
    };

static void rebootDevice()
{
  Logger.Error("REBOOTING");
  bool hasCommunicationErrors = communicationErrorCount.load() > COMMUNICATION_ERROR_LIMIT;
  if (hasCommunicationErrors) 
  {
    Logger.Info("Trying to restart due to communication errors");
    Logger.Error("Communication errors: " + String(communicationErrorCount.load()));
  }

  #ifdef USE_DISPLAY
    showReboot(hasCommunicationErrors, hasCommunicationErrors ? communicationErrorCount.load() : 0);
  #endif
  delay(5000);
  ESP.restart();
}

#ifdef RUUVI_MAC
  static void setupRuuviSensors()
  {
    RuuviTagSensor* ruuviSensor = new RuuviTagSensor(RUUVI_MAC, RUUVI_SENSORID);
    ruuviSensors.push_back(ruuviSensor);
    sensors.push_back(ruuviSensor);
    #ifdef RUUVI_MAC_2
      RuuviTagSensor* ruuviSensor2 = new RuuviTagSensor(RUUVI_MAC_2, RUUVI_SENSORID_2);
      ruuviSensors.push_back(ruuviSensor2);
      sensors.push_back(ruuviSensor2);
    #endif
    scanner = new RuuviTagScanner(ruuviSensors);
  }
#endif

#ifdef RUUVI_MAC
  void initRuuvi()
  {
    Logger.Info("Free heap (before init) (bytes): " + String(ESP.getFreeHeap()));
    esp_task_wdt_reset();
    NimBLEDevice::init("ESP32-Ruuvi");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setScanCallbacks(scanner, false);
    pBLEScan->setActiveScan(false);
    pBLEScan->setMaxResults(0); // Using just the callback
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(100);
    Logger.Info("Free heap (after init) (bytes): " + String(ESP.getFreeHeap()));
  }

  void readRuuvi()
  {
    esp_task_wdt_reset();
    Logger.Info("Starting RUUVI SCAN");
    pBLEScan->start(RUUVI_SCAN_TIME, false, true);
    esp_task_wdt_reset();
    Logger.Info("RUUVI SCAN DONE");
    Serial.printf("Free heap (after scan): %u bytes\n", ESP.getFreeHeap());
  }

  static void serviceRuuvi()
  {
    if (loopCount == 20 && !hasInitedRuuvi)
    {
      initRuuvi();
      hasInitedRuuvi = true;
    } else if (measureCount != lastRuuviMeasureCount && measureCount > 0 && measureCount % 20 == 0)
    {
      readRuuvi();
      lastRuuviMeasureCount = measureCount;
    }
  }
#endif



// Arduino setup and loop main functions.
void setup() 
{
  pinMode(REDLEDPIN, OUTPUT);
  pinMode(GREENLEDPIN, OUTPUT);
  if (SENSORPOWERPIN) 
  {
    pinMode(SENSORPOWERPIN, OUTPUT);
  }
  // Init sensors
  #ifdef DHT22_PIN
    Logger.Info("DHT22");
    sensors.push_back(new DHT22Sensor(DHT22_SENSORID, DHT22_PIN));
  #endif

  #ifdef TMP36_PIN
    Logger.Info("TMP36");
    sensors.push_back(new TMP36Sensor(TMP36_SENSORID, TMP36_PIN));
  #endif 

  #ifdef DS18B20_PIN
    sensors.push_back(new DS18B20Sensor(DS18B20_SENSORID, DS18B20_PIN));
  #endif

  #ifdef DS18B20_2_PIN
    sensors.push_back(new DS18B20Sensor(DS18B20_2_SENSORID, DS18B20_2_PIN));
  #endif

  #ifdef BH1750FVI_SENSORID
    sensors.push_back(new BH1750FVISensor(BH1750FVI_SENSORID));
  #endif

  #ifdef MOTIONSENSOR_IN_PINS
    motionSensor = new MotionSensor(MOTIONSENSOR_SENSORID, MOTIONSENSOR_IN_PINS, MOTIONSENSOR_OUT_PINS, MOTIONSENSOR_MULTI_TRIGGER_MODE, MOTIONSENSOR_OR_MODE);
    sensors.push_back(motionSensor);
  #endif 

  #ifdef RUUVI_MAC
    setupRuuviSensors();
  #endif

  for (Sensor* sensor : sensors)
  {
    Logger.Info("Call begin for SensorId: " + String(sensor->getSensorId()));
    sensor->begin();
  }  
  // Delay stuff
  esp_task_wdt_deinit(); //wdt is enabled by default, so we need to deinit it first
  esp_task_wdt_init(&twdt_config); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinMode(YELLOWLEDPIN, OUTPUT);
  setStatus(3);
  #ifdef USE_DISPLAY
    initDisplay();
  #endif
  Logger.Info("Establish connection");
  Serial.printf("Free heap (setup): %u bytes\n", ESP.getFreeHeap());
  establishConnection();
  Serial.printf("Free heap (after connection): %u bytes\n", ESP.getFreeHeap());
}

void loop() {
  #ifdef USE_MQQT_BROKER
    if (WiFi.status() == WL_CONNECTED)
    {
      MqttBroker::service();
    }
  #endif
  if (WiFi.status() != WL_CONNECTED) 
  {
    setStatus(0);
    Logger.Error("WIFI connection lost");
    connectToWiFi();
    setStatus(3);
  }
  #if defined(USE_IOT_HUB) && !defined(IOT_CONFIG_USE_X509_CERT)
    else if (IoTHub::tokenExpired())
    {
      if (!IoTHub::refreshToken())
      {
        setStatus(0);
        Logger.Error("Failed to reinit MQTT client. Restarting...");
        rebootDevice();
      }
    }
  #endif
  else if (communicationErrorCount.load() > COMMUNICATION_ERROR_LIMIT)
  {
    setStatus(0);
    Logger.Info("Trying to restart due to communication errors");
    rebootDevice();
  }
  else if (measureCount > MEASURE_LIMIT) 
  { 
    if (sendTelemetry()) {
      successCount++;
      sentMessageCount++;
      hasSentMessage = true;
      for (Sensor* sensor : sensors)
      {
        sensor->resetAverages();
      }
      if (successCount < SUCCESS_LIMIT) {
        setStatus(1); 
      } else {
        setStatus(2); 
      }
    } else 
    {
      setStatus(0);
      successCount = 0;
      delay(1000);
      return;
    }
  }

  if (inited) 
  {
    loopCount++;
    if (!hasSentMessage && millis() > FIRST_MESSAGE_MILLIS)
    {
      Logger.Info("Sending empty first message");
      if (sendTelemetry(true))
      {
        Logger.Info("Empty first message sent");
        hasSentMessage = true;
      } else 
      {
        // Sending the empty first message failed, wait a while before trying again
        delay(4000);
      }
    }
    // loopCount < lastLoopCount = ovelap
    if (loopCount < lastLoopCount ||  loopCount - lastLoopCount >= MEASURE_LOOP_COUNT) 
    {
      Logger.Info("Loop count: " + String(loopCount) + ", Last loop count: " + lastLoopCount);
      lastLoopCount = loopCount;
      if(!calculateMeasurements()) 
      {
        setStatus(0);
        successCount = 0;
      }
    }
    #ifdef USE_HTTP_API
      serviceHttpAttributes();
    #endif
    #ifdef RUUVI_MAC
      serviceRuuvi();
    #endif
  } else {
    if (millis() > 4000) 
    {
      inited = 1;
      if (SENSORPOWERPIN) 
      {
        Logger.Info("Powering sensor...");
        digitalWrite(SENSORPOWERPIN, HIGH);
        delay(4000); 
      }    
    }
  }
  #ifdef MOTIONSENSOR_IN_PINS
    motionSensor->checkOutputs();
  #endif
  esp_task_wdt_reset();
  delay(LOOP_WAIT);
}
