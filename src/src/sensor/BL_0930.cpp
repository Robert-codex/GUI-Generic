#ifdef SUPLA_BL0930

#include "BL_0930.h"

namespace Supla {
namespace Sensor {

BL_0930 *BL_0930::instance = nullptr;

BL_0930::BL_0930(int8_t pinCF) : pinCF(pinCF) {
  instance = this;
}

void BL_0930::onInit() {
  extChannel.setFlag(SUPLA_CHANNEL_FLAG_CALCFG_RESET_COUNTERS);
  pinMode(pinCF, INPUT);
  pulseOffset = pulseCount;
  attachInterrupt(pinCF, onPulse, FALLING);
  readValuesFromDevice();
  updateChannelValues();
}

void BL_0930::readValuesFromDevice() {
  uint32_t localPulseCount = 0;
  uint32_t localLastPulseUs = 0;
  uint32_t localPulsePeriodUs = 0;

  noInterrupts();
  localPulseCount = pulseCount;
  localLastPulseUs = lastPulseUs;
  localPulsePeriodUs = pulsePeriodUs;
  interrupts();

  if (pulseConstant == 0) {
    pulseConstant = 3200;
  }

  energy = energyBase + ((static_cast<uint64_t>(localPulseCount - pulseOffset) * 100000ULL) / pulseConstant);

  double activePower = 0;
  uint32_t nowUs = micros();
  if (localPulsePeriodUs > 0 && static_cast<uint32_t>(nowUs - localLastPulseUs) < (localPulsePeriodUs * 3UL + 500000UL)) {
    activePower = 3600000000000.0 / (static_cast<double>(pulseConstant) * static_cast<double>(localPulsePeriodUs));
  }

  setVoltage(0, 0);
  setCurrent(0, 0);
  setPowerActive(0, activePower * 100000.0);
  setPowerApparent(0, 0);
  setPowerReactive(0, 0);
  setPowerFactor(0, 0);
  setFwdActEnergy(0, energy);
}

void BL_0930::onSaveState() {
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy));
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&pulseConstant), sizeof(pulseConstant));
}

void BL_0930::onLoadState() {
  if (Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy))) {
    setCounter(energy);
  }

  Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&pulseConstant), sizeof(pulseConstant));
  if (pulseConstant == 0) {
    pulseConstant = 3200;
  }
}

_supla_int64_t BL_0930::getCounter() const {
  return energy;
}

void BL_0930::setCounter(_supla_int64_t value) {
  noInterrupts();
  pulseOffset = pulseCount;
  interrupts();

  energyBase = value;
  energy = value;
  setFwdActEnergy(0, value);
  Supla::Storage::ScheduleSave(1000);
}

uint32_t BL_0930::getPulseConstant() const {
  return pulseConstant;
}

void BL_0930::setPulseConstant(uint32_t value) {
  if (value == 0) {
    return;
  }

  readValuesFromDevice();
  energyBase = energy;
  noInterrupts();
  pulseOffset = pulseCount;
  interrupts();
  pulseConstant = value;
  Supla::Storage::ScheduleSave(1000);
}

int BL_0930::handleCalcfgFromServer(TSD_DeviceCalCfgRequest *request) {
  if (request && request->Command == SUPLA_CALCFG_CMD_RESET_COUNTERS) {
    setCounter(0);
    return SUPLA_CALCFG_RESULT_DONE;
  }
  return SUPLA_CALCFG_RESULT_NOT_SUPPORTED;
}

void IRAM_ATTR BL_0930::onPulse() {
  if (instance == nullptr) {
    return;
  }

  uint32_t nowUs = micros();
  if (instance->lastPulseUs != 0) {
    instance->pulsePeriodUs = nowUs - instance->lastPulseUs;
  }
  instance->lastPulseUs = nowUs;
  instance->pulseCount++;
}

};  // namespace Sensor
};  // namespace Supla

#endif
