// Sensors
#include <ArduinoJson.h>
#include <vector>
// Sensors implementations
#include "DHT22Sensor.h" 
#include "TMP36Sensor.h"
#include "DS18B20Sensor.h"
#include "BH1750FVISensor.h"
#include "MotionSensor.h"
// Display
#include <Wire.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define DISPLAY_TEXT_SIZE 1

#include "configs/pins.h"

#ifdef USE_DISPLAY
  #ifdef SH1106
    #define DISPLAY_ROW_COUNT 8
    #define SCREEN_HEIGHT 64
    #include <Adafruit_SH110X.h>
    Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
  #else
    #define DISPLAY_ROW_COUNT 4
    #define SCREEN_HEIGHT 32
    #include <Adafruit_SSD1306.h>
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
  #endif
#endif

#define DEBUG 0
// RUUVI

#define RUUVI_SCAN_TIME 20000 // RUUVI SCAN TIME IN MS
#ifdef RUUVI_MAC
  #include <NimBLEDevice.h>
  #include <NimBLEAdvertisedDevice.h>
  #include "RuuviTagScanner.h"
#endif


#include "configs/iot_configs.h" // Place DeviceId / IOT HUB settings + WIFI settings in this file
// Commands from IOT HUB
#define CMD_REBOOT "REBOOT"
// Loop definitions
// MEASURE_LOOP_COUNT * MEASURE_LIMIT*LOOP_WAIT = Send Interval in ms
#define LOOP_WAIT 100
#define MEASURE_LOOP_COUNT  40 
#define FIRST_MESSAGE_MILLIS 5000
// Message settings
#define SUCCESS_LIMIT 3
#define MEASURE_LIMIT 20 
#define MEASURE_START_LOOP_LIMIT 6
#define MOTION_DETECTION_SHUTDOWN_DELAY_MS 45000 // Set to 0 if controlled by sensor

// https://iotassistant.io/esp32/fixing-error-hardware-wdt-arduino-esp32/
#define WDT_TIMEOUT 15000 // 15 s
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1 

#include "esp_task_wdt.h"
#include "esp_system.h"
// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#ifdef USE_IOT_HUB
  #include <mqtt_client.h>
  // Azure IoT SDK for C includes
  #include <az_core.h>
  #include <az_iot.h>
  #include <azure_ca.h>
  // Additional sample headers
  #include "AzIoTSasToken.h"
#endif

#ifdef USE_HTTP_API
  #include <HTTPClient.h>
  #include <WiFiClientSecure.h>
#endif

#include "SerialLogger.h"

// When developing for your own Arduino-based platform,
// please follow the format '(ard;<platform>)'.
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"

// Utility macros and defines
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
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

#ifdef USE_IOT_HUB
  static const char* host = IOT_CONFIG_IOTHUB_FQDN;
  static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
  static const char* device_id = IOT_CONFIG_DEVICE_ID;
  static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

  // Memory allocated for the sample's variables and structures.
  static esp_mqtt_client_handle_t mqtt_client;
  static az_iot_hub_client client;

  static char mqtt_client_id[256]; // Changed
  static char mqtt_username[256];  // Changed
  static char mqtt_password[200];
#endif
static uint8_t sas_signature_buffer[256];


#ifdef USE_HTTP_API
  static const char* api_endpoint_url = API_ENDPOINT_URL;
#endif

static char telemetry_topic[128];
static String telemetry_payload = "{}";

// Status variables
int communicationErrorCount = 0;
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

MotionControlStatus motionControlStatus = MotionControl;
// Sensors
std::vector<Sensor*> sensors;
MotionSensor* motionSensor;

#ifdef RUUVI_MAC
  BLEScan* pBLEScan; 
  RuuviTagScanner* scanner; 
  std::vector<RuuviTagSensor*> ruuviSensors;
#endif

#define INCOMING_DATA_BUFFER_SIZE 512
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Auxiliary functions

#ifdef USE_IOT_HUB
  #ifndef IOT_CONFIG_USE_X509_CERT
  static AzIoTSasToken sasToken(
    &client,
    AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
    AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
    AZ_SPAN_FROM_BUFFER(mqtt_password));
  #endif  // IOT_CONFIG_USE_X509_CERT
#endif

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

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Logger.Info("Received [");
  Logger.Info(topic);
  Logger.Info("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}


#ifdef USE_IOT_HUB
  void handleMessageFromHub()
  {
    if (strcmp(incoming_data, CMD_REBOOT) == 0) 
    {
      Logger.Info("Reboot command from IOT HUB. Rebooting after 2 s...");
      delay(2000);
      ESP.restart();
    }
    MotionControlStatus status;
    if (parseMotionControlStatus(incoming_data, status)) 
    {
      Logger.Info("Setting motion control status to: " + String(status));
      #ifdef MOTIONSENSOR_IN_PINS
        motionSensor->setMotionControlStatus(status);
      #endif
    }
    unsigned long delay;
    if (parseMotionControlDelay(incoming_data, delay))
    {
      Logger.Info("Setting motion control delays");
      #ifdef MOTIONSENSOR_IN_PINS
        Logger.Info("Setting delay to: " + String(delay));
        motionSensor->setMotionControlDelay(delay);
      #endif
    }
  }
#endif

#ifdef USE_IOT_HUB
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    (void)handler_args;
    (void)base;
    (void)event_id;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  #else   // ESP_ARDUINO_VERSION_MAJOR
  static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
  #endif  // ESP_ARDUINO_VERSION_MAJOR
    switch (event->event_id) {
      int i, r;

      case MQTT_EVENT_ERROR:
        communicationErrorCount++;
        Logger.Info("MQTT event MQTT_EVENT_ERROR");
        break;
      case MQTT_EVENT_CONNECTED:
        Logger.Info("MQTT event MQTT_EVENT_CONNECTED");
        communicationErrorCount = 0;
        r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
        if (r == -1) {
          Logger.Error("Could not subscribe for cloud-to-device messages.");
        } else {
          Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
        }

        break;
      case MQTT_EVENT_DISCONNECTED:
        communicationErrorCount++;
        Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
        break;
      case MQTT_EVENT_SUBSCRIBED:
        Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
        break;
      case MQTT_EVENT_UNSUBSCRIBED:
        Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
        break;
      case MQTT_EVENT_PUBLISHED:
        Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
        break;
      case MQTT_EVENT_DATA:
        Logger.Info("MQTT event MQTT_EVENT_DATA");

        for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++) {
          incoming_data[i] = event->topic[i];
        }
        incoming_data[i] = '\0';
        Logger.Info("Topic: " + String(incoming_data));

        for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++) {
          incoming_data[i] = event->data[i];
        }
        incoming_data[i] = '\0';
        Logger.Info("Data: " + String(incoming_data));
        handleMessageFromHub();
        break;
      case MQTT_EVENT_BEFORE_CONNECT:
        Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
        break;
      default:
        Logger.Error("MQTT event UNKNOWN");
        break;
    }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #else   // ESP_ARDUINO_VERSION_MAJOR
    return ESP_OK;
  #endif  // ESP_ARDUINO_VERSION_MAJOR
  }

#endif

#ifdef USE_IOT_HUB
  static void initializeIoTHubClient() 
  {
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

    if (az_result_failed(az_iot_hub_client_init(
          &client,
          az_span_create((uint8_t*)host, strlen(host)),
          az_span_create((uint8_t*)device_id, strlen(device_id)),
          &options))) {
      Logger.Error("Failed initializing Azure IoT Hub client");
      return;
    }

    size_t client_id_length;
    if (az_result_failed(az_iot_hub_client_get_client_id(
          &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))) {
      Logger.Error("Failed getting client id");
      return;
    }

    int result = az_result_failed(az_iot_hub_client_get_user_name(
      &client, mqtt_username, sizeofarray(mqtt_username), NULL));

    if (result) {
      Logger.Error("Failed to get MQTT clientId, return code");
      Logger.Error(String(result));
      return;
    }

    Logger.Info("Client ID: " + String(mqtt_client_id));
    Logger.Info("Username: " + String(mqtt_username));
  }

  static int initializeMqttClient() 
  {
  #ifndef IOT_CONFIG_USE_X509_CERT
    if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
      Logger.Error("Failed generating SAS token");
      return 1;
    }
  #endif

    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(mqtt_config));

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    mqtt_config.broker.address.uri = mqtt_broker_uri;
    mqtt_config.broker.address.port = mqtt_port;
    mqtt_config.credentials.client_id = mqtt_client_id;
    mqtt_config.credentials.username = mqtt_username;

  #ifdef IOT_CONFIG_USE_X509_CERT
    LogInfo("MQTT client using X509 Certificate authentication");
    mqtt_config.credentials.authentication.certificate = IOT_CONFIG_DEVICE_CERT;
    mqtt_config.credentials.authentication.certificate_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT);
    mqtt_config.credentials.authentication.key = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
    mqtt_config.credentials.authentication.key_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY);
  #else  // Using SAS key
    mqtt_config.credentials.authentication.password = (const char*)az_span_ptr(sasToken.Get());
  #endif

    mqtt_config.session.keepalive = 30;
    mqtt_config.session.disable_clean_session = 0;
    mqtt_config.network.disable_auto_reconnect = false;
    mqtt_config.broker.verification.certificate = (const char*)ca_pem;
    mqtt_config.broker.verification.certificate_len = (size_t)ca_pem_len;
  #else  // ESP_ARDUINO_VERSION_MAJOR
    mqtt_config.uri = mqtt_broker_uri;
    mqtt_config.port = mqtt_port;
    mqtt_config.client_id = mqtt_client_id;
    mqtt_config.username = mqtt_username;

  #ifdef IOT_CONFIG_USE_X509_CERT
    Logger.Info("MQTT client using X509 Certificate authentication");
    mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
    mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
  #else  // Using SAS key
    mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
  #endif

    mqtt_config.keepalive = 30;
    mqtt_config.disable_clean_session = 0;
    mqtt_config.disable_auto_reconnect = false;
    mqtt_config.event_handle = mqtt_event_handler;
    mqtt_config.user_context = NULL;
    mqtt_config.cert_pem = (const char*)ca_pem;
  #endif  // ESP_ARDUINO_VERSION_MAJOR

    mqtt_client = esp_mqtt_client_init(&mqtt_config);

    if (mqtt_client == NULL) {
      Logger.Error("Failed creating mqtt client");
      return 1;
    }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
  #endif  // ESP_ARDUINO_VERSION_MAJOR

    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

    if (start_result != ESP_OK) {
      Logger.Error("Could not start mqtt client; error code:" + start_result);
      return 1;
    } else {
      Logger.Info("MQTT client started");
      return 0;
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
    initializeIoTHubClient();
    (void)initializeMqttClient();
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
    display.clearDisplay();
    display.setCursor(0,0);
  #endif

  for (Sensor* sensor : sensors) 
  {
    tempC = sensor->readTemperature(false);
    humi = sensor->readHumidity(false);
    lightV = sensor->readLight(false);
    motionV = sensor->readMotion(false);

    int sensorId = sensor->getSensorId();
    #ifdef USE_DISPLAY
      String messageToPrint = String(sensorId) + ": ";
      if (tempC != Sensor::ERROR_FAILED_READING && tempC != Sensor::ERROR_UNSUPPORTED) 
      {
        messageToPrint = messageToPrint + String(tempC) + " C";
      } else if (tempC == Sensor::ERROR_FAILED_READING)
      {
        messageToPrint = messageToPrint + "ERR" + " C";
      }
      if (humi != Sensor::ERROR_FAILED_READING && humi != Sensor::ERROR_UNSUPPORTED)
      {
        messageToPrint = messageToPrint + ", " + String(humi) + " %";
      } else if (humi == Sensor::ERROR_FAILED_READING) 
      {
        messageToPrint = messageToPrint + ", ERR" + " %";
      }

      if (lightV != Sensor::ERROR_FAILED_READING && lightV != Sensor::ERROR_UNSUPPORTED)
      {
        messageToPrint = messageToPrint + String(lightV) + " lx";
      } else if (lightV == Sensor::ERROR_FAILED_READING)
      {
        messageToPrint = messageToPrint + "ERR" + " lx";
      }

      if (motionV >= 0) 
      {
        if (motionV) 
        {
          messageToPrint = messageToPrint + "ON";
        } else 
        {
          messageToPrint = messageToPrint + "OFF";
        }   
      }
      display.println(messageToPrint);                           
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
    int rowCount = sensors.size();
    #ifdef MOTIONSENSOR_IN_PINS
      #ifdef MOTIONSENSOR_DISPLAY_ON_DELAY
        float delayInS = motionSensor->getOutputDelayLeft();
        if (delayInS > 0) 
        {
          display.println("ON DELAY (s): " + String(delayInS));
          rowCount++;
        }
      #endif
    #endif   

    if (rowCount < DISPLAY_ROW_COUNT) 
    {
      display.println("Measure count: " + String(measureCount) + "/" + String(MEASURE_LIMIT));
      rowCount++;
    }

    if (rowCount < DISPLAY_ROW_COUNT) 
    {
      display.println("Sent messages: " + String(sentMessageCount));
      rowCount++;
    }

    display.display();
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
// Sends the JSON telemetry payload to the API endpoint.
// Returns 1 on success (HTTP 2xx), 0 otherwise.
static int postTelemetryToApi() {
  HTTPClient http;
  WiFiClientSecure secureClient;

  // If you've defined api_root_ca in iot_configs.h, load it to verify the HTTPS certificate;
  // otherwise call setInsecure() for testing only.
#ifdef USE_API_ROOT_CA
  secureClient.setCACert(api_root_ca);
#else
  secureClient.setInsecure();
#endif

  int result = 0;
  if (http.begin(secureClient, api_endpoint_url)) {
    http.addHeader("Content-Type", "application/json");

    // Custom authentication headers defined in iot_configs.h
    if (strlen(API_KEY_HEADER_NAME) > 0 && strlen(API_KEY_HEADER_VALUE) > 0) {
      http.addHeader(API_KEY_HEADER_NAME, API_KEY_HEADER_VALUE);
    }
    if (strlen(SECRET_ID_HEADER_NAME) > 0 && strlen(SECRET_ID_HEADER_VALUE) > 0) {
      http.addHeader(SECRET_ID_HEADER_NAME, SECRET_ID_HEADER_VALUE);
    }
    if (strlen(SECRET_VALUE_HEADER_NAME) > 0 && strlen(SECRET_VALUE_HEADER_VALUE) > 0) {
      http.addHeader(SECRET_VALUE_HEADER_NAME, SECRET_VALUE_HEADER_VALUE);
    }

    // Post the telemetry payload (telemetry_payload holds the JSON string)
    int httpCode = http.POST(telemetry_payload);
    if (httpCode > 0 && httpCode < 300) {
      Logger.Info("[HTTP] POST successful, code: " + String(httpCode));
      result = 1;
    } else {
      Logger.Error("[HTTP] POST failed, code: " + String(httpCode));
    }
    http.end();
  } else {
    Logger.Error("[HTTP] Unable to connect to API endpoint");
  }
  return result;
}
#endif

static int sendTelemetry(bool sendEmpty = false) {
  Logger.Info("Sending telemetry ...");   

  #ifdef USE_IOT_HUB
    // The topic could be obtained just once during setup,
    // however if properties are used the topic need to be generated again to reflect the
    // current values of the properties.
    if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
          &client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))) {
      Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
      return 0;
    }
  #endif

  if(!generateTelemetryPayload(sendEmpty)) 
  {
    Logger.Error("Failed to get sensor data");
    return 0;
  }

  #ifdef USE_IOT_HUB
    if (esp_mqtt_client_publish(
          mqtt_client,
          telemetry_topic,
          (const char*)telemetry_payload.c_str(),
          telemetry_payload.length(),
          MQTT_QOS1,
          DO_NOT_RETAIN_MSG)
        == 0) {
      Logger.Error("Failed publishing");
      return 0;
    } else {
      Logger.Info("Message published successfully");
      return 1;
    }  

  #else
    return postTelemetryToApi();
  #endif

}

void setStatus(int statusToSet) {
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
  }
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
    motionSensor = new MotionSensor(MOTIONSENSOR_SENSORID, MOTIONSENSOR_IN_PINS, MOTIONSENSOR_OUT_PINS, MOTIONSENSOR_MULTI_TRIGGER_MODE);
    sensors.push_back(motionSensor);
  #endif 

  #ifdef RUUVI_MAC
    RuuviTagSensor* ruuviSensor = new RuuviTagSensor(RUUVI_MAC, RUUVI_SENSORID);
    ruuviSensors.push_back(ruuviSensor);
    sensors.push_back(ruuviSensor);
    #ifdef RUUVI_MAC_2
      RuuviTagSensor* ruuviSensor2 = new RuuviTagSensor(RUUVI_MAC_2, RUUVI_SENSORID_2);
      ruuviSensors.push_back(ruuviSensor2);
      sensors.push_back(ruuviSensor2);
    #endif
    scanner = new RuuviTagScanner(ruuviSensors);
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
  // Display
  #ifdef USE_DISPLAY

    #ifdef SH1106
      display.begin(0x3C, true);
    #else
      if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
      }
    #endif
    Logger.Info("DISPLAY INITED");
    delay(2000);
    Logger.Info("DISPLAY INITED. WILL PRINT");
    display.clearDisplay();
    display.setTextSize(DISPLAY_TEXT_SIZE);
    #ifdef SH1106
      display.setTextColor(SH110X_WHITE);
    #else
      display.setTextColor(WHITE);
    #endif
    display.setCursor(0,0);
    // Display static text
    display.println("App initing...");
    display.display();
    delay(2000);     
  #endif
  Logger.Info("Establish connection");
  Serial.printf("Free heap (setup): %u bytes\n", ESP.getFreeHeap());
  establishConnection();
  Serial.printf("Free heap (after connection): %u bytes\n", ESP.getFreeHeap());
}

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
#endif

void loop() {
  if (WiFi.status() != WL_CONNECTED) 
  {
    setStatus(0);
    Logger.Error("WIFI connection lost");
    connectToWiFi();
    setStatus(3);
  }
  #ifdef USE_IOT_HUB
    else if (sasToken.IsExpired()) 
    {
      Logger.Info("SAS token expired; reconnecting with a new one.");
      (void)esp_mqtt_client_destroy(mqtt_client);
      if (initializeMqttClient() != 0) {
        setStatus(0);
        Logger.Error("Failed to reinit MQTT client. Restarting...");
        delay(5000);
        ESP.restart();
      }
      Logger.Info("SAS token refreshed");
    }
  #endif
  else if (communicationErrorCount > 0) 
  {
    setStatus(0);
    if (communicationErrorCount > 12) 
    {
      Logger.Info("Trying to restart due to communication errors");
      delay(5000);
      ESP.restart();
    }
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

    if (DEBUG) 
    {
      Logger.Info("Loop count: " + String(loopCount));
    }
    #ifdef RUUVI_MAC
      if (loopCount == 20 && !hasInitedRuuvi)
      {
        initRuuvi();
        hasInitedRuuvi = true;
      } else if (measureCount != lastRuuviMeasureCount && measureCount > 0 && measureCount % 20 == 0)
      {
        readRuuvi();
        lastRuuviMeasureCount = measureCount;
      }
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
