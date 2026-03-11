#ifndef SuplaZigbeeGateway_h
#define SuplaZigbeeGateway_h

#if defined(ARDUINO_ARCH_ESP32)
#include <sdkconfig.h>
#endif

#if defined(SUPLA_ZIGBEE_GATEWAY) && defined(CONFIG_IDF_TARGET_ESP32C6)

#include <SuplaDevice.h>
#include <ZigbeeCore.h>
#include <ZigbeeGateway.h>

namespace Supla::GUI {

class ZigbeeGatewayMode {
 public:
  static void setup();
  static void iterate();

 private:
  static bool configured;
  static bool started;
  static bool coexist_enabled;
  static ZigbeeGateway gateway;
};

}  // namespace Supla::GUI

#else

namespace Supla::GUI {

class ZigbeeGatewayMode {
 public:
  static void setup() {}
  static void iterate() {}
};

}  // namespace Supla::GUI

#endif

#endif  // SuplaZigbeeGateway_h
