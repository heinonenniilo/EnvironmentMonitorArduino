#pragma once

#include "TransportConfig.h"

#ifdef USE_MQQT_BROKER
namespace MqttBroker
{
  void begin();
  void service();
  bool canPublish();
  int publishTelemetry(const String& payload);
}
#endif
