#include "SuplaGuiWiFi.h"
#include "../../SuplaDeviceGUI.h"
#include <supla/tools.h>
#include <supla/device/register_device.h>

namespace Supla {

GUIESPWifi::GUIESPWifi(const char *wifiSsid, const char *wifiPassword) : ESPWifi(wifiSsid, wifiPassword) {
#ifdef ARDUINO_ARCH_ESP8266
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP &event) {
    (void)(event);
    Serial.print(F("Connected BSSID: "));
    Serial.println(WiFi.BSSIDstr());
    Serial.print(F("local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("subnetMask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("gatewayIP: "));
    Serial.println(WiFi.gatewayIP());
    int rssi = WiFi.RSSI();
    Serial.print(F("Signal strength (RSSI): "));
    Serial.print(rssi);
    Serial.println(F(" dBm"));
  });
  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &event) {
    (void)(event);
    Serial.println(F("WiFi station disconnected"));
  });
#else
  WiFiEventId_t event_gotIP = WiFi.onEvent(
      [](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.print(F("Connected BSSID: "));
        Serial.println(WiFi.BSSIDstr());
        Serial.print(F("local IP: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("subnetMask: "));
        Serial.println(WiFi.subnetMask());
        Serial.print(F("gatewayIP: "));
        Serial.println(WiFi.gatewayIP());
        int rssi = WiFi.RSSI();
        Serial.print(F("Signal strength (RSSI): "));
        Serial.print(rssi);
        Serial.println(F(" dBm"));
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  (void)(event_gotIP);

  WiFiEventId_t event_disconnected = WiFi.onEvent(
      [](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.println(F("WiFi Station disconnected"));
        // ESP32 doesn't reconnect automatically after lost connection
        WiFi.reconnect();
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  (void)(event_disconnected);
#endif
}

void GUIESPWifi::setup() {
  if (!wifiConfigured) {
    WiFi.setHostname(hostname);  // ESP32 requires setHostname to be called before begin...
    WiFi.softAPdisconnect(true);
    WiFi.persistent(true);
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.setAutoConnect(true);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    // https://github.com/esp8266/Arduino/issues/8412
    // https://forum.supla.org/viewtopic.php?t=11999
    // https://forum.supla.org/viewtopic.php?p=167849#p167849
    WiFi.setPhyMode(WIFI_PHY_MODE_11G);
#endif

    wifiConfigured = true;
  }
  else {
    if (mode == Supla::DEVICE_MODE_NORMAL) {
      Serial.println(F("WiFi: resetting WiFi connection"));
      DisconnectProtocols();
      WiFi.disconnect();
      WiFi.reconnect();  // This does not reset dhcp
      delay(200);        // do not remove, need a delay for disconnect to change status()

      forceRestartESP();
    }
  }

  if (mode == Supla::DEVICE_MODE_CONFIG) {
    SUPLA_LOG_INFO("WiFi: enter config mode with SSID: \"%s\"", getAPName().c_str());
    const bool hasStationSsid = ssid[0] != '\0';
    if (hasStationSsid && (getCountChannels() == 0 || strcmp(Supla::RegisterDevice::getServerName(), DEFAULT_SERVER) == 0)) {
      SUPLA_LOG_INFO("WiFi: WIFI_AP_STA");
      WiFi.mode(WIFI_AP_STA);
      WiFi.begin(ssid, password);
    }
    else {
      SUPLA_LOG_INFO("WiFi: WIFI_MODE_AP");
      WiFi.mode(WIFI_MODE_AP);
    }

    WiFi.softAP(getAPName().c_str(), "", 6);

    Supla::GUI::crateWebServer();
  }
  else {
    SUPLA_LOG_INFO("WiFi: establishing connection with SSID: \"%s\"", ssid);
    WiFi.mode(WIFI_MODE_STA);

    IPAddress emptyIp(0, 0, 0, 0);
    if (useDhcp) {
      WiFi.config(emptyIp, emptyIp, emptyIp, emptyIp, emptyIp);
      Serial.println(F("WiFi: DHCP mode"));
    } else {
      IPAddress ip;
      IPAddress gateway;
      IPAddress subnet;
      if (ip.fromString(ipAddress) && gateway.fromString(gatewayAddress) && subnet.fromString(subnetMask)) {
        WiFi.config(ip, gateway, subnet, emptyIp, emptyIp);
        Serial.print(F("WiFi: static IP "));
        Serial.print(ip);
        Serial.print(F(" gateway "));
        Serial.print(gateway);
        Serial.print(F(" mask "));
        Serial.println(subnet);
      } else {
        WiFi.config(emptyIp, emptyIp, emptyIp, emptyIp, emptyIp);
        Serial.println(F("WiFi: invalid static IP configuration, fallback to DHCP"));
      }
    }

    WiFi.begin(ssid, password);
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.setHostname(hostname);  // ESP8266 requires setHostname to be called after begin...
#endif

    if (ConfigManager->get(KEY_ENABLE_GUI)->getValueInt())
      Supla::GUI::crateWebServer();
  }

  delay(0);
}

void GUIESPWifi::forceRestartESP() {
  retryCount++;

  if (retryCount > 3) {
    retryCount = 0;

    if (ConfigManager->get(KEY_FORCE_RESTART_ESP)->getValueBool())
      ConfigESP->rebootESP();
  }
}
void GUIESPWifi::setHostName(const char *wifiHostname) {
  if (wifiHostname) {
    strncpy(hostname, wifiHostname, MAX_HOSTNAME);
    hostname[MAX_HOSTNAME] = '\0';
  }
}

void GUIESPWifi::enableSSL(bool value) {
  setSSLEnabled(value);
}

void GUIESPWifi::setSsid(const char *wifiSsid) {
  if (wifiSsid) {
    strncpy(ssid, wifiSsid, MAX_SSID_SIZE - 1);
    ssid[MAX_SSID_SIZE - 1] = '\0';
  }
}

void GUIESPWifi::setPassword(const char *wifiPassword) {
  if (wifiPassword) {
    wifiConfigured = false;
    strncpy(password, wifiPassword, MAX_WIFI_PASSWORD_SIZE - 1);
    password[MAX_WIFI_PASSWORD_SIZE - 1] = '\0';
  }
}

void GUIESPWifi::setIpConfig(bool dhcp, const char *ipAddressValue, const char *gatewayValue, const char *subnetValue) {
  useDhcp = dhcp;

  if (ipAddressValue) {
    strncpy(ipAddress, ipAddressValue, MAX_IPV4_TEXT - 1);
    ipAddress[MAX_IPV4_TEXT - 1] = '\0';
  }

  if (gatewayValue) {
    strncpy(gatewayAddress, gatewayValue, MAX_IPV4_TEXT - 1);
    gatewayAddress[MAX_IPV4_TEXT - 1] = '\0';
  }

  if (subnetValue) {
    strncpy(subnetMask, subnetValue, MAX_IPV4_TEXT - 1);
    subnetMask[MAX_IPV4_TEXT - 1] = '\0';
  }
}
};  // namespace Supla
