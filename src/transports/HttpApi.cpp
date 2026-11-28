#include "HttpApi.h"

#ifdef USE_HTTP_API
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <cstdlib>
#include <string.h>

#include "../SerialLogger.h"

namespace HttpApi
{
  #define HTTP_TIMEOUT_MS 8000

  static const char* api_endpoint_url = API_ENDPOINT_URL;
  static const char* api_attributes_url = API_ATTRIBUTES_URL;

  // Sends the JSON telemetry payload to the API endpoint.
  // Returns 1 on success (HTTP 2xx), 0 otherwise.
  int publishTelemetry(const String& payload)
  {
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
      http.setTimeout(HTTP_TIMEOUT_MS);
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

      int httpCode = http.POST(payload);
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

  int readAttributes(int& motionControlStatus, unsigned long& onDelayMs) {
    HTTPClient http;
    WiFiClientSecure secureClient;

  #ifdef USE_API_ROOT_CA
    secureClient.setCACert(api_root_ca);
  #else
    secureClient.setInsecure(); // testing only
  #endif

    motionControlStatus = 0;
    onDelayMs = 0;

    int result = 0;

    if (http.begin(secureClient, api_attributes_url)) {
      http.setTimeout(HTTP_TIMEOUT_MS);
      // If your API needs to know what it returns, add Accept header
      http.addHeader("Accept", "application/json");

      // Same custom authentication headers as telemetry
      if (strlen(API_KEY_HEADER_NAME) > 0 && strlen(API_KEY_HEADER_VALUE) > 0) {
        http.addHeader(API_KEY_HEADER_NAME, API_KEY_HEADER_VALUE);
      }
      if (strlen(SECRET_ID_HEADER_NAME) > 0 && strlen(SECRET_ID_HEADER_VALUE) > 0) {
        http.addHeader(SECRET_ID_HEADER_NAME, SECRET_ID_HEADER_VALUE);
      }
      if (strlen(SECRET_VALUE_HEADER_NAME) > 0 && strlen(SECRET_VALUE_HEADER_VALUE) > 0) {
        http.addHeader(SECRET_VALUE_HEADER_NAME, SECRET_VALUE_HEADER_VALUE);
      }

      int httpCode = http.GET();
      if (httpCode > 0 && httpCode < 300) {
        String payload = http.getString();
        Logger.Info("[HTTP] Attributes GET successful, code: " + String(httpCode));
        Logger.Info("[HTTP] Payload: " + payload); // enable if you want to debug

        // Parse JSON payload
        StaticJsonDocument<256> doc; // a bit larger than needed, still tiny
        DeserializationError err = deserializeJson(doc, payload);

        if (err) {
          Logger.Error(String("[JSON] Parse failed: ") + err.c_str());
        } else {
          // Use defaults if fields are missing
          // 0 = STATUS
          // 1 = DELAY
          // STATUS (enum stored as int)
          const char* statusStr = doc["0"] | "0";
          motionControlStatus = atoi(statusStr);
          // DELAY (milliseconds)
          const char* delayStr = doc["1"] | "0";
          onDelayMs = strtoul(delayStr, nullptr, 10);

          Logger.Info("[ATTR] MotionControlStatus=" + String(motionControlStatus) +
                      " OnDelay=" + String(onDelayMs));
          result = 1;
        }
      } else {
        Logger.Error("[HTTP] Attributes GET failed, code: " + String(httpCode));
        // Optional: show error body if any
        String errBody = http.getString();
        if (errBody.length() > 0) {
          Logger.Error("[HTTP] Error body: " + errBody);
        }
      }

      http.end();
    } else {
      Logger.Error("[HTTP] Unable to connect to attributes endpoint");
    }

    return result;
  }
}
#endif
