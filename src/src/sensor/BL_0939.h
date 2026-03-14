#ifdef SUPLA_BL0939
#ifndef _bl0939_gui_h
#define _bl0939_gui_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <supla/sensor/electricity_meter.h>
#include <supla/storage/storage.h>

namespace Supla {
namespace Sensor {

class BL_0939 : public ElectricityMeter {
 public:
  BL_0939(HardwareSerial &serial, int8_t pinRX, int8_t pinTX);

  void onInit();
  void readValuesFromDevice();
  void onSaveState();
  void onLoadState();
  void resetStorage() override;

 protected:
  static constexpr uint8_t PACKET_HEADER = 0x55;
  static constexpr uint8_t READ_COMMAND = 0x50;
  static constexpr uint8_t WRITE_COMMAND = 0xA0;
  static constexpr uint8_t FULL_PACKET = 0xAA;
  static constexpr uint8_t DEVICE_ADDRESS = 0x05;
  static constexpr uint32_t BAUDRATE = 4800;
  static constexpr size_t PACKET_SIZE = 35;
  static constexpr uint32_t RESPONSE_TIMEOUT_MS = 200;

  static constexpr uint32_t POWER_REFERENCE = 713;
  static constexpr uint32_t VOLTAGE_REFERENCE = 17159;
  static constexpr uint32_t CURRENT_REFERENCE = 266013;

  bool readPacket(uint8_t *buffer, size_t size);
  bool parsePacket(const uint8_t *buffer, size_t size);
  void sendReadRequest();
  void sendInitSequence();
  void writeRegister(uint8_t reg, uint8_t data0, uint8_t data1, uint8_t data2);
  void clearMeasurements();
  uint32_t readUnsigned24(const uint8_t *buffer, size_t offset) const;
  int32_t readSigned24(const uint8_t *buffer, size_t offset) const;

  HardwareSerial &serial;
  int8_t pinRX;
  int8_t pinTX;
  uint64_t forwardEnergy[2] = {0, 0};
  uint64_t reverseEnergy[2] = {0, 0};
  uint32_t lastReadMs = 0;
  uint32_t lastGoodReadMs = 0;
};

};  // namespace Sensor
};  // namespace Supla

#endif
#endif
