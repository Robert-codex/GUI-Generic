#include "SuplaZigbeeGateway.h"

#if defined(SUPLA_ZIGBEE_GATEWAY) && defined(CONFIG_IDF_TARGET_ESP32C6)

#include "esp_coexist.h"

namespace Supla::GUI {

namespace {
constexpr uint8_t kGatewayEndpoint = 1;
constexpr uint8_t kOpenNetworkSeconds = 180;
}

bool ZigbeeGatewayMode::configured = false;
bool ZigbeeGatewayMode::started = false;
bool ZigbeeGatewayMode::coexist_enabled = false;
ZigbeeGateway ZigbeeGatewayMode::gateway(kGatewayEndpoint);

void ZigbeeGatewayMode::setup() {
  if (configured) {
    return;
  }

  gateway.setManufacturerAndModel("Supla", "Z2SGateway");
  gateway.allowMultipleBinding(true);
  Zigbee.addEndpoint(&gateway);
  Zigbee.setRebootOpenNetwork(kOpenNetworkSeconds);
  configured = true;
}

void ZigbeeGatewayMode::iterate() {
  if (!configured || started) {
    return;
  }

  if (SuplaDevice.getCurrentStatus() != STATUS_REGISTERED_AND_READY) {
    return;
  }

  if (!coexist_enabled) {
    esp_coex_wifi_i154_enable();
    coexist_enabled = true;
  }

  if (Zigbee.begin(ZIGBEE_COORDINATOR)) {
    started = true;
  }
}

}  // namespace Supla::GUI

#endif
