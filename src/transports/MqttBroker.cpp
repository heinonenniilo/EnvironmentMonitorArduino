#include "MqttBroker.h"

#ifdef USE_MQQT_BROKER
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <string.h>
#if MQTT_BROKER_USE_TLS
  #include <WiFiClientSecure.h>
#endif

#include "../SerialLogger.h"
#include "TransportEvents.h"

namespace MqttBroker
{
  #define MQTT_QOS1 1
  #define MQTT_BROKER_TIMEOUT_MS 3000
  #define MQTT_BROKER_RECONNECT_INTERVAL_MS 5000

  #if MQTT_BROKER_USE_TLS
    static WiFiClientSecure brokerTransport;
  #else
    static WiFiClient brokerTransport;
  #endif
  static char incoming_data[512];
  static MqttClient brokerClient(brokerTransport);
  static String brokerMeasurementTopic = MQTT_BROKER_MEASUREMENTS_TOPIC;
  static String brokerCommandTopic = MQTT_BROKER_COMMANDS_TOPIC;
  static unsigned long lastBrokerConnectionAttempt = 0;
  static bool brokerConnectionAttempted = false;
  static bool brokerWasConnected = false;
  static unsigned long brokerConnectionAttempts = 0;
  static unsigned long lastBrokerStatusLog = 0;

  static void handleBrokerMessage(int messageSize)
  {
    Logger.Info("[MQTT] Message received: topic=" + brokerClient.messageTopic() +
      ", bytes=" + String(messageSize) + ", QoS=" + String(brokerClient.messageQoS()) +
      ", retained=" + String(brokerClient.messageRetain()));
    if (brokerClient.messageTopic() != brokerCommandTopic ||
        messageSize <= 0 || messageSize >= static_cast<int>(sizeof(incoming_data)))
    {
      Logger.Error("[MQTT] Rejecting command: expected topic=" + brokerCommandTopic +
        ", allowed bytes=1.." + String(sizeof(incoming_data) - 1) + "; closing connection");
      brokerClient.stop();
      return;
    }

    size_t length = 0;
    unsigned long started = millis();
    while (length < (size_t)messageSize && millis() - started < MQTT_BROKER_TIMEOUT_MS)
    {
      int value = brokerClient.read();
      if (value < 0)
      {
        break;
      }
      incoming_data[length++] = (char)value;
      esp_task_wdt_reset();
    }
    if (length != (size_t)messageSize || memchr(incoming_data, '\0', length) != nullptr)
    {
      Logger.Error("[MQTT] Rejecting incomplete or invalid command: read " +
        String(length) + "/" + String(messageSize) + " bytes; closing connection");
      brokerClient.stop();
      return;
    }
    incoming_data[length] = '\0';
    Logger.Info("[MQTT] Command: " + String(incoming_data));
    Logger.Info("[MQTT] Dispatching command to shared command handler");
    handleCommand(incoming_data);
  }

  void begin()
  {
    brokerMeasurementTopic.replace("{IOT_CONFIG_DEVICE_ID}", IOT_CONFIG_DEVICE_ID);
    brokerCommandTopic.replace("{IOT_CONFIG_DEVICE_ID}", IOT_CONFIG_DEVICE_ID);
    Logger.Info("[MQTT] Initializing ArduinoMqttClient");
    Logger.Info("[MQTT] Broker=" + String(MQTT_BROKER_HOST) + ":" + String(MQTT_BROKER_PORT) +
      ", client ID=" + String(IOT_CONFIG_DEVICE_ID));
    Logger.Info("[MQTT] Measurements topic=" + brokerMeasurementTopic);
    Logger.Info("[MQTT] Commands topic=" + brokerCommandTopic);
    Logger.Info("[MQTT] Username/password authentication=" +
      String(strlen(MQTT_BROKER_USERNAME) > 0 ? "enabled" : "disabled"));
    #if MQTT_BROKER_USE_TLS
      Logger.Info("[MQTT] TLS enabled; certificate verification required");
      #ifdef USE_MQTT_ROOT_CA
        Logger.Info("[MQTT] Root CA configured: " + String(strlen(mqtt_root_ca)) + " bytes");
        if (strlen(mqtt_root_ca) == 0)
        {
          Logger.Error("[MQTT] Root CA is empty; configure mqtt_root_ca before connecting");
        }
        brokerTransport.setCACert(mqtt_root_ca);
      #else
        Logger.Error("[MQTT] Root CA not configured: define USE_MQTT_ROOT_CA and mqtt_root_ca");
      #endif
      brokerTransport.setHandshakeTimeout(MQTT_BROKER_TIMEOUT_MS / 1000);
    #else
      Logger.Info("[MQTT] TLS disabled; connection and credentials are not encrypted");
    #endif
    brokerTransport.setConnectionTimeout(MQTT_BROKER_TIMEOUT_MS);
    brokerClient.setConnectionTimeout(MQTT_BROKER_TIMEOUT_MS);
    brokerClient.setTimeout(MQTT_BROKER_TIMEOUT_MS);
    brokerClient.setId(IOT_CONFIG_DEVICE_ID);
    brokerClient.setUsernamePassword(MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    brokerClient.setCleanSession(true);
    brokerClient.setKeepAliveInterval(30 * 1000UL);
    brokerClient.onMessage(handleBrokerMessage);
    Logger.Info("[MQTT] Keepalive=30 s, timeout=" + String(MQTT_BROKER_TIMEOUT_MS) +
      " ms, reconnect interval=" + String(MQTT_BROKER_RECONNECT_INTERVAL_MS) + " ms");
  }

  void service()
  {
    if (brokerClient.connected())
    {
      brokerWasConnected = true;
      brokerClient.poll();
      if (brokerClient.connected() && millis() - lastBrokerStatusLog >= 30 * 1000UL)
      {
        lastBrokerStatusLog = millis();
        Logger.Info("[MQTT] Still connected; listening on " + brokerCommandTopic);
      }
      return;
    }
    if (brokerWasConnected)
    {
      brokerWasConnected = false;
      Logger.Error("[MQTT] Connection closed; reconnect and resubscription required");
    }
    if (brokerConnectionAttempted &&
        millis() - lastBrokerConnectionAttempt < MQTT_BROKER_RECONNECT_INTERVAL_MS)
    {
      return;
    }
    brokerConnectionAttempted = true;
    brokerConnectionAttempts++;
    unsigned long started = millis();
    Logger.Info("[MQTT] Connection attempt #" + String(brokerConnectionAttempts) +
      " to " + String(MQTT_BROKER_HOST) + ":" + String(MQTT_BROKER_PORT));
    esp_task_wdt_reset();
    bool connected = false;
    #if MQTT_BROKER_USE_TLS && !defined(USE_MQTT_ROOT_CA)
      Logger.Error("[MQTT] TLS requires USE_MQTT_ROOT_CA and mqtt_root_ca");
    #else
      connected = brokerClient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    #endif
    esp_task_wdt_reset();
    if (connected)
    {
      Logger.Info("[MQTT] Broker accepted connection after " + String(millis() - started) + " ms");
      Logger.Info("[MQTT] Subscribing to " + brokerCommandTopic + " with QoS 1");
    }
    else
    {
      Logger.Error("[MQTT] Connect failed after " + String(millis() - started) +
        " ms; connectError=" + String(brokerClient.connectError()) +
        " (-2=transport failure, -1=timeout, 1=protocol, 2=client ID, 3=unavailable, 4=credentials, 5=unauthorized)");
      #if MQTT_BROKER_USE_TLS
        char tlsError[160] = {};
        int tlsErrorCode = brokerTransport.lastError(tlsError, sizeof(tlsError));
        if (tlsErrorCode != 0)
        {
          Logger.Error("[MQTT] TLS error " + String(tlsErrorCode) + ": " + String(tlsError));
        }
      #endif
    }
    bool subscribed = connected && brokerClient.subscribe(brokerCommandTopic, MQTT_QOS1);
    esp_task_wdt_reset();
    lastBrokerConnectionAttempt = millis();
    if (!subscribed)
    {
      if (connected)
      {
        Logger.Error("[MQTT] Subscription failed: granted QoS=" + String(brokerClient.subscribeQoS()) +
          " (128=refused); check broker topic permissions");
      }
      brokerClient.stop();
      reportCommunicationError();
      Logger.Info("[MQTT] Retrying connection in " + String(MQTT_BROKER_RECONNECT_INTERVAL_MS) + " ms");
      return;
    }
    clearCommunicationErrors();
    brokerWasConnected = true;
    lastBrokerStatusLog = millis();
    setStatus(3);
    Logger.Info("[MQTT] Subscription confirmed: topic=" + brokerCommandTopic +
      ", granted QoS=" + String(brokerClient.subscribeQoS()));
  }

  bool canPublish()
  {
    if (!brokerClient.connected())
    {
      Logger.Error("[MQTT] Cannot send telemetry while disconnected");
      return false;
    }
    return true;
  }

  int publishTelemetry(const String& payload)
  {
    unsigned long started = millis();
    Logger.Info("[MQTT] Publishing telemetry: topic=" + brokerMeasurementTopic +
      ", bytes=" + String(payload.length()) + ", QoS=1, retained=0");
    esp_task_wdt_reset();
    bool published = brokerClient.beginMessage(
      brokerMeasurementTopic, (unsigned long)payload.length(), false, MQTT_QOS1);
    if (!published)
    {
      Logger.Error("[MQTT] Failed writing publish header");
    }
    if (published)
    {
      size_t written = brokerClient.write(
        (const uint8_t*)payload.c_str(), payload.length());
      published = written == payload.length();
      if (!published)
      {
        Logger.Error("[MQTT] Incomplete telemetry write: " + String(written) +
          "/" + String(payload.length()) + " bytes");
      }
    }
    if (published)
    {
      Logger.Info("[MQTT] Payload written; waiting for broker acknowledgement (PUBACK)");
      published = brokerClient.endMessage() == 1;
      if (!published)
      {
        Logger.Error("[MQTT] Publish acknowledgement failed or timed out");
      }
    }
    esp_task_wdt_reset();
    if (!published)
    {
      Logger.Error("[MQTT] Failed publishing telemetry after " + String(millis() - started) +
        " ms; closing connection");
      brokerClient.stop();
      reportCommunicationError();
      return 0;
    }
    clearCommunicationErrors();
    Logger.Info("[MQTT] Telemetry acknowledged by broker after " + String(millis() - started) + " ms");
    return 1;
  }
}
#endif
