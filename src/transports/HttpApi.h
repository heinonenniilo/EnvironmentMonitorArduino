#pragma once

#include "TransportConfig.h"

#ifdef USE_HTTP_API
namespace HttpApi
{
  int publishTelemetry(const String& payload);
  int readAttributes(int& motionControlStatus, unsigned long& onDelayMs);
}
#endif
