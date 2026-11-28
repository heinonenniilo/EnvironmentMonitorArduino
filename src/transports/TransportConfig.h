#pragma once

#include <Arduino.h>
#include "../configs/iot_configs.h"

#ifdef USE_MQQT_BROKER
  #undef USE_IOT_HUB
  #undef USE_HTTP_API
  #ifndef MQTT_BROKER_USE_TLS
    #define MQTT_BROKER_USE_TLS 1
  #endif
  #ifndef MQTT_BROKER_PORT
    #if MQTT_BROKER_USE_TLS
      #define MQTT_BROKER_PORT 8883
    #else
      #define MQTT_BROKER_PORT 1883
    #endif
  #endif
#elif !defined(USE_IOT_HUB) && !defined(USE_HTTP_API)
  #error "Define USE_IOT_HUB, USE_HTTP_API, or USE_MQQT_BROKER in src/configs/iot_configs.h"
#endif
