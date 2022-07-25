#pragma once

#include "device.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
extern device_t REMOTEID_device;
#define HAS_REMOTEID
#endif