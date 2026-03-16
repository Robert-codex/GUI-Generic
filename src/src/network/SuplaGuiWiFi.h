#ifndef SuplaGuiWiFi_h
#define SuplaGuiWiFi_h

#include <supla/network/esp_wifi.h>

#define MAX_HOSTNAME   32
#define DEFAULT_SERVER "svrX.supla.org"
#define MAX_IPV4_TEXT  16

namespace Supla {
class GUIESPWifi : public Supla::ESPWifi {
 public:
  GUIESPWifi(const char *wifiSsid = nullptr, const char *wifiPassword = nullptr);
  // ~GUIESPWifi();

  // int connect(const char *server, int port = -1);
  void setup();
  void setHostName(const char *wifiHostname);
  void enableSSL(bool value);
  void setSsid(const char *wifiSsid);
  void setPassword(const char *wifiPassword);
  void setIpConfig(bool dhcp, const char *ipAddress, const char *gateway, const char *subnet);
  void forceRestartESP();

 protected:
  char hostname[MAX_HOSTNAME + 1] = {};
  bool useDhcp = true;
  char ipAddress[MAX_IPV4_TEXT] = {};
  char gatewayAddress[MAX_IPV4_TEXT] = {};
  char subnetMask[MAX_IPV4_TEXT] = {};
  int8_t retryCount = 0;
};
};      // namespace Supla
#endif  // SuplaGuiWiFi_h
