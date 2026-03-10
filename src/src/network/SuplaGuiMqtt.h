#ifndef SuplaGuiMqtt_h
#define SuplaGuiMqtt_h

#include <SuplaDevice.h>

class MqttClient {
 public:
  explicit MqttClient(SuplaDeviceClass *device);

 private:
  SuplaDeviceClass *device = nullptr;
};

#endif  // SuplaGuiMqtt_h
