#ifdef SUPLA_CSE7761

#include "CSE_7761.h"

namespace Supla {
namespace Sensor {

namespace {
constexpr int CSE7761_UREF = 42563;
constexpr int CSE7761_IREF = 52241;
constexpr int CSE7761_PREF = 44513;

constexpr uint8_t REG_SYSCON = 0x00;
constexpr uint8_t REG_EMUCON = 0x01;
constexpr uint8_t REG_EMUCON2 = 0x13;
constexpr uint8_t REG_PULSE1SEL = 0x1D;
constexpr uint8_t REG_RMSIA = 0x24;
constexpr uint8_t REG_RMSIB = 0x25;
constexpr uint8_t REG_RMSU = 0x26;
constexpr uint8_t REG_POWERPA = 0x2C;
constexpr uint8_t REG_POWERPB = 0x2D;
constexpr uint8_t REG_SYSSTATUS = 0x43;
constexpr uint8_t REG_COEFFCHKSUM = 0x6F;
constexpr uint8_t REG_RMSIAC = 0x70;

constexpr uint8_t SPECIAL_COMMAND = 0xEA;
constexpr uint8_t CMD_RESET = 0x96;
constexpr uint8_t CMD_CLOSE_WRITE = 0xDC;
constexpr uint8_t CMD_ENABLE_WRITE = 0xE5;
}  // namespace

CSE_7761::CSE_7761(HardwareSerial &serial, int8_t pinRX, int8_t pinTX, uint8_t currentChannel)
    : serial(serial), pinRX(pinRX), pinTX(pinTX), currentChannel(currentChannel > 0 ? 1 : 0) {
}

void CSE_7761::onInit() {
  extChannel.setFlag(SUPLA_CHANNEL_FLAG_CALCFG_RESET_COUNTERS);
  serial.begin(38400, SERIAL_8E1, pinRX, pinTX);
  delay(10);

  writeRegister(SPECIAL_COMMAND, CMD_RESET);
  delay(10);

  uint16_t syscon = readRegister(REG_SYSCON, 2);
  if (syscon == 0x0A04 && chipInit()) {
    writeRegister(SPECIAL_COMMAND, CMD_CLOSE_WRITE);
    ready = true;
    lastReadMs = millis();
    readValuesFromDevice();
    updateChannelValues();
  }
}

void CSE_7761::readValuesFromDevice() {
  if (!ready) {
    return;
  }

  uint32_t value = readRegister(REG_RMSU, 3);
  voltageRms = value >= 0x800000 ? 0 : value;

  value = readRegister(REG_RMSIA, 3);
  currentRms[0] = (value >= 0x800000 || value < 1600) ? 0 : value;
  value = readRegister(REG_POWERPA, 4);
  if (currentRms[0] == 0) {
    activePower[0] = 0;
  } else {
    int32_t signedValue = static_cast<int32_t>(value);
    activePower[0] = static_cast<uint32_t>(signedValue < 0 ? -signedValue : signedValue);
  }

  value = readRegister(REG_RMSIB, 3);
  currentRms[1] = (value >= 0x800000 || value < 1600) ? 0 : value;
  value = readRegister(REG_POWERPB, 4);
  if (currentRms[1] == 0) {
    activePower[1] = 0;
  } else {
    int32_t signedValue = static_cast<int32_t>(value);
    activePower[1] = static_cast<uint32_t>(signedValue < 0 ? -signedValue : signedValue);
  }

  float voltage = static_cast<float>(voltageRms) / coefficientByUnit(RMS_UC);
  float current = static_cast<float>(currentRms[currentChannel]) /
                  coefficientByUnit(currentChannel == 0 ? RMS_IAC : RMS_IBC);
  float active = static_cast<float>(activePower[currentChannel]) /
                 coefficientByUnit(currentChannel == 0 ? POWER_PAC : POWER_PBC);
  float apparent = computeApparentPower(voltage, current);
  float reactive = 0;
  if (apparent > active) {
    reactive = sqrtf(apparent * apparent - active * active);
  }
  float pf = apparent <= 0 ? 0 : active / apparent;
  if (pf > 1) {
    pf = 1;
  }

  uint32_t now = millis();
  if (lastReadMs != 0 && active > 0) {
    uint32_t elapsedMs = now - lastReadMs;
    energyAtLoad += static_cast<uint64_t>((active * elapsedMs) / 36.0f);
    energy = energyAtLoad;
  }
  lastReadMs = now;

  setVoltage(0, voltage * 100);
  setCurrent(0, current * 1000);
  setPowerActive(0, active * 100000);
  setPowerApparent(0, apparent * 100000);
  setPowerReactive(0, reactive * 100000);
  setPowerFactor(0, pf * 1000);
  setFwdActEnergy(0, energy);
}

void CSE_7761::onSaveState() {
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy));
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&currentChannel), sizeof(currentChannel));
}

void CSE_7761::onLoadState() {
  if (Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy))) {
    setCounter(energy);
  }
  Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&currentChannel), sizeof(currentChannel));
  currentChannel = currentChannel > 0 ? 1 : 0;
}

void CSE_7761::setCounter(_supla_int64_t value) {
  energyAtLoad = value;
  energy = value;
  setFwdActEnergy(0, value);
}

_supla_int64_t CSE_7761::getCounter() const {
  return energy;
}

uint8_t CSE_7761::getCurrentChannel() const {
  return currentChannel;
}

void CSE_7761::setCurrentChannel(uint8_t value) {
  currentChannel = value > 0 ? 1 : 0;
  Supla::Storage::ScheduleSave(1000);
}

bool CSE_7761::chipInit() {
  uint16_t checksum = 0xFFFF;
  for (uint8_t i = 0; i < 8; i++) {
    coefficients[i] = readRegister(REG_RMSIAC + i, 2);
    checksum += coefficients[i];
  }
  checksum = ~checksum;

  uint16_t coeffChecksum = readRegister(REG_COEFFCHKSUM, 2);
  if (checksum != coeffChecksum || !checksum) {
    coefficients[RMS_IAC] = CSE7761_IREF;
    coefficients[RMS_IBC] = CSE7761_IREF;
    coefficients[RMS_UC] = CSE7761_UREF;
    coefficients[POWER_PAC] = CSE7761_PREF;
    coefficients[POWER_PBC] = CSE7761_PREF;
  }

  writeRegister(SPECIAL_COMMAND, CMD_ENABLE_WRITE);
  uint8_t status = readRegister(REG_SYSSTATUS, 1);
  if (!(status & 0x10)) {
    return false;
  }

  writeRegister(REG_SYSCON | 0x80, 0xFF04);
  writeRegister(REG_EMUCON | 0x80, 0x1183);
  writeRegister(REG_EMUCON2 | 0x80, 0x0FC1);
  writeRegister(REG_PULSE1SEL | 0x80, 0x3290);
  return true;
}

void CSE_7761::writeRegister(uint8_t reg, uint16_t data) {
  uint8_t buffer[5] = {0xA5, reg, 0, 0, 0};
  uint32_t len = 2;
  if (data) {
    if (data < 0xFF) {
      buffer[2] = data & 0xFF;
      len = 3;
    } else {
      buffer[2] = (data >> 8) & 0xFF;
      buffer[3] = data & 0xFF;
      len = 4;
    }

    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
      crc += buffer[i];
    }
    buffer[len++] = ~crc;
  }
  serial.write(buffer, len);
  serial.flush();
}

bool CSE_7761::readOnce(uint8_t reg, uint8_t size, uint32_t *value) {
  while (serial.available()) {
    serial.read();
  }

  writeRegister(reg, 0);
  uint8_t buffer[8] = {0};
  uint32_t received = 0;
  uint32_t start = millis();
  while (received <= size && millis() - start < 200) {
    if (serial.available()) {
      int byteRead = serial.read();
      if (byteRead > -1 && received < sizeof(buffer)) {
        buffer[received++] = byteRead;
      }
    } else {
      delay(1);
    }
  }

  if (received == 0) {
    return false;
  }

  received--;
  uint32_t result = 0;
  uint8_t crc = 0xA5 + reg;
  for (uint32_t i = 0; i < received; i++) {
    result = (result << 8) | buffer[i];
    crc += buffer[i];
  }
  crc = ~crc;
  if (crc != buffer[received]) {
    return false;
  }

  *value = result;
  return true;
}

uint32_t CSE_7761::readRegister(uint8_t reg, uint8_t size) {
  uint32_t value = 0;
  for (uint8_t retry = 0; retry < 3; retry++) {
    if (readOnce(reg, size, &value)) {
      return value;
    }
  }
  return value;
}

uint32_t CSE_7761::coefficientByUnit(CoefficientIndex unit) const {
  switch (unit) {
    case RMS_UC:
      return 0x400000 * 100 / coefficients[RMS_UC];
    case RMS_IAC:
      return (0x800000 * 100 / coefficients[RMS_IAC]) * 10;
    case RMS_IBC:
      return (0x800000 * 100 / coefficients[RMS_IBC]) * 10;
    case POWER_PAC:
      return 0x80000000 / coefficients[POWER_PAC];
    case POWER_PBC:
      return 0x80000000 / coefficients[POWER_PBC];
    default:
      return 1;
  }
}

float CSE_7761::computeApparentPower(float voltage, float current) const {
  return voltage * current;
}

};  // namespace Sensor
};  // namespace Supla

#endif
