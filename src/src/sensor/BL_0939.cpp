#ifdef SUPLA_BL0939

#include "BL_0939.h"
#include <math.h>

namespace Supla {
namespace Sensor {

namespace {
constexpr uint8_t REG_I_FAST_RMS_CTRL = 0x10;
constexpr uint8_t REG_MODE = 0x18;
constexpr uint8_t REG_SOFT_RESET = 0x19;
constexpr uint8_t REG_USR_WRPROT = 0x1A;
constexpr uint8_t REG_TPS_CTRL = 0x1B;

constexpr uint8_t kInitRegisters[][4] = {
    {REG_SOFT_RESET, 0x5A, 0x5A, 0x5A},
    {REG_USR_WRPROT, 0x55, 0x00, 0x00},
    {REG_MODE, 0x00, 0x10, 0x00},
    {REG_TPS_CTRL, 0xFF, 0x47, 0x00},
    {REG_I_FAST_RMS_CTRL, 0x1C, 0x18, 0x00},
};
}  // namespace

BL_0939::BL_0939(HardwareSerial &serial, int8_t pinRX, int8_t pinTX)
    : serial(serial), pinRX(pinRX), pinTX(pinTX) {
  extChannel.setFlag(SUPLA_CHANNEL_FLAG_PHASE3_UNSUPPORTED);
}

void BL_0939::onInit() {
  setRefreshRate(1);
  serial.begin(BAUDRATE, SERIAL_8N1, pinRX, pinTX);
  delay(10);

  sendInitSequence();
  delay(10);

  readValuesFromDevice();
  updateChannelValues();
}

void BL_0939::readValuesFromDevice() {
  uint8_t buffer[PACKET_SIZE] = {0};
  if (readPacket(buffer, sizeof(buffer)) && parsePacket(buffer, sizeof(buffer))) {
    lastGoodReadMs = millis();
    return;
  }

  if (lastGoodReadMs != 0 && millis() - lastGoodReadMs > 5000) {
    clearMeasurements();
  }
}

void BL_0939::onSaveState() {
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(forwardEnergy), sizeof(forwardEnergy));
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(reverseEnergy), sizeof(reverseEnergy));
}

void BL_0939::onLoadState() {
  Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(forwardEnergy), sizeof(forwardEnergy));
  Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(reverseEnergy), sizeof(reverseEnergy));

  for (uint8_t channel = 0; channel < 2; channel++) {
    setFwdActEnergy(channel, forwardEnergy[channel]);
    setRvrActEnergy(channel, reverseEnergy[channel]);
  }
}

void BL_0939::resetStorage() {
  forwardEnergy[0] = 0;
  forwardEnergy[1] = 0;
  reverseEnergy[0] = 0;
  reverseEnergy[1] = 0;
  lastReadMs = 0;
  lastGoodReadMs = 0;

  for (uint8_t channel = 0; channel < 2; channel++) {
    setFwdActEnergy(channel, 0);
    setRvrActEnergy(channel, 0);
  }

  Supla::Storage::ScheduleSave(1000);
}

bool BL_0939::readPacket(uint8_t *buffer, size_t size) {
  while (serial.available()) {
    serial.read();
  }

  sendReadRequest();

  bool started = false;
  size_t received = 0;
  uint32_t start = millis();

  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (serial.available()) {
      int byteRead = serial.read();
      if (byteRead < 0) {
        continue;
      }

      uint8_t value = static_cast<uint8_t>(byteRead);
      if (!started) {
        if (value != PACKET_HEADER) {
          continue;
        }
        started = true;
      }

      if (received < size) {
        buffer[received++] = value;
      }

      if (received == size) {
        uint8_t checksum = READ_COMMAND | DEVICE_ADDRESS;
        for (size_t i = 0; i < size - 1; i++) {
          checksum += buffer[i];
        }
        checksum ^= 0xFF;
        return checksum == buffer[size - 1];
      }
    }
    delay(1);
  }

  return false;
}

bool BL_0939::parsePacket(const uint8_t *buffer, size_t size) {
  if (size < PACKET_SIZE || buffer[0] != PACKET_HEADER) {
    return false;
  }

  const uint32_t voltageRaw = readUnsigned24(buffer, 10);
  const uint32_t currentRawA = readUnsigned24(buffer, 4);
  const uint32_t currentRawB = readUnsigned24(buffer, 7);
  const int32_t activeRawA = readSigned24(buffer, 16);
  const int32_t activeRawB = readSigned24(buffer, 19);

  const float voltage = static_cast<float>(voltageRaw) / VOLTAGE_REFERENCE;
  const float currents[2] = {
      static_cast<float>(currentRawA) / CURRENT_REFERENCE,
      static_cast<float>(currentRawB) / CURRENT_REFERENCE,
  };
  const float actives[2] = {
      static_cast<float>(activeRawA) / POWER_REFERENCE,
      static_cast<float>(activeRawB) / POWER_REFERENCE,
  };

  const uint32_t now = millis();
  if (lastReadMs != 0) {
    const uint32_t elapsedMs = now - lastReadMs;
    for (uint8_t channel = 0; channel < 2; channel++) {
      const float active = actives[channel];
      if (active > 0.0f) {
        forwardEnergy[channel] += static_cast<uint64_t>((active * elapsedMs) / 36.0f);
      } else if (active < 0.0f) {
        reverseEnergy[channel] += static_cast<uint64_t>(((-active) * elapsedMs) / 36.0f);
      }
    }
  }
  lastReadMs = now;

  for (uint8_t channel = 0; channel < 2; channel++) {
    const float active = actives[channel];
    const float activeAbs = active < 0.0f ? -active : active;
    const float current = activeAbs > 1.0f ? currents[channel] : 0.0f;
    const float apparent = voltage > 0.0f ? voltage * current : 0.0f;
    float reactive = 0.0f;
    if (apparent > activeAbs) {
      reactive = sqrtf(apparent * apparent - activeAbs * activeAbs);
    }
    float pf = apparent > 0.0f ? activeAbs / apparent : 0.0f;
    if (pf > 1.0f) {
      pf = 1.0f;
    }

    setVoltage(channel, static_cast<unsigned _supla_int16_t>(voltage * 100.0f));
    setCurrent(channel, static_cast<unsigned _supla_int_t>(current * 1000.0f));
    setPowerActive(channel, static_cast<int64_t>(active * 100000.0f));
    setPowerApparent(channel, static_cast<int64_t>(apparent * 100000.0f));
    setPowerReactive(channel, static_cast<int64_t>(reactive * 100000.0f));
    setPowerFactor(channel, static_cast<_supla_int_t>(pf * 1000.0f));
    setFwdActEnergy(channel, forwardEnergy[channel]);
    setRvrActEnergy(channel, reverseEnergy[channel]);
  }

  return true;
}

void BL_0939::sendReadRequest() {
  serial.flush();
  serial.write(READ_COMMAND | DEVICE_ADDRESS);
  serial.write(FULL_PACKET);
  serial.flush();
}

void BL_0939::sendInitSequence() {
  for (const auto &reg : kInitRegisters) {
    writeRegister(reg[0], reg[1], reg[2], reg[3]);
    delay(1);
  }
}

void BL_0939::writeRegister(uint8_t reg, uint8_t data0, uint8_t data1, uint8_t data2) {
  uint8_t crc = (WRITE_COMMAND | DEVICE_ADDRESS) + reg + data0 + data1 + data2;
  serial.write(WRITE_COMMAND | DEVICE_ADDRESS);
  serial.write(reg);
  serial.write(data0);
  serial.write(data1);
  serial.write(data2);
  serial.write(static_cast<uint8_t>(0xFF ^ crc));
  serial.flush();
}

void BL_0939::clearMeasurements() {
  for (uint8_t channel = 0; channel < 2; channel++) {
    setVoltage(channel, 0);
    setCurrent(channel, 0);
    setPowerActive(channel, 0);
    setPowerApparent(channel, 0);
    setPowerReactive(channel, 0);
    setPowerFactor(channel, 0);
  }
}

uint32_t BL_0939::readUnsigned24(const uint8_t *buffer, size_t offset) const {
  return static_cast<uint32_t>(buffer[offset + 2]) << 16 |
         static_cast<uint32_t>(buffer[offset + 1]) << 8 |
         static_cast<uint32_t>(buffer[offset]);
}

int32_t BL_0939::readSigned24(const uint8_t *buffer, size_t offset) const {
  int32_t value = static_cast<int32_t>(readUnsigned24(buffer, offset));
  if (value & 0x00800000) {
    value |= 0xFF000000;
  }
  return value;
}

};  // namespace Sensor
};  // namespace Supla

#endif
