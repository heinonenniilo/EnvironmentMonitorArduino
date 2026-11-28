#pragma once

#include "TransportConfig.h"

#ifdef USE_IOT_HUB
namespace IoTHub
{
  void begin();
  bool prepareTelemetryTopic();
  int publishTelemetry(const String& payload);

  #ifndef IOT_CONFIG_USE_X509_CERT
    bool tokenExpired();
    bool refreshToken();
  #endif
}
#endif
