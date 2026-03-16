#ifdef SUPLA_CSE7759B_FG

#include "CSE_7759B_FG.h"

namespace Supla {
namespace Sensor {

CSE_7759B_FG *CSE_7759B_FG::instance = nullptr;

CSE_7759B_FG::CSE_7759B_FG(int8_t pinCF) : pinCF(pinCF) {
  instance = this;
}

void CSE_7759B_FG::onInit() {
  extChannel.setFlag(SUPLA_CHANNEL_FLAG_CALCFG_RESET_COUNTERS);
  pinMode(pinCF, INPUT);
  pulseOffset = pulseCount;
  attachInterrupt(pinCF, onPulse, FALLING);
  readValuesFromDevice();
  updateChannelValues();
}

void CSE_7759B_FG::readValuesFromDevice() {
  uint32_t localPulseCount = 0;
  uint32_t localLastPulseUs = 0;
  uint32_t localPulsePeriodUs = 0;

  noInterrupts();
  localPulseCount = pulseCount;
  localLastPulseUs = lastPulseUs;
  localPulsePeriodUs = pulsePeriodUs;
  interrupts();

  if (pulseConstant == 0) {
    pulseConstant = 1000;
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

void CSE_7759B_FG::onSaveState() {
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy));
  Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&pulseConstant), sizeof(pulseConstant));
}

void CSE_7759B_FG::onLoadState() {
  if (Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&energy), sizeof(energy))) {
    setCounter(energy);
  }

  Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&pulseConstant), sizeof(pulseConstant));
  if (pulseConstant == 0) {
    pulseConstant = 1000;
  }
}

_supla_int64_t CSE_7759B_FG::getCounter() const {
  return energy;
}

void CSE_7759B_FG::setCounter(_supla_int64_t value) {
  noInterrupts();
  pulseOffset = pulseCount;
  interrupts();

  energyBase = value;
  energy = value;
  setFwdActEnergy(0, value);
  Supla::Storage::ScheduleSave(1000);
}

uint32_t CSE_7759B_FG::getPulseConstant() const {
  return pulseConstant;
}

void CSE_7759B_FG::setPulseConstant(uint32_t value) {
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

int CSE_7759B_FG::handleCalcfgFromServer(TSD_DeviceCalCfgRequest *request) {
  if (request && request->Command == SUPLA_CALCFG_CMD_RESET_COUNTERS) {
    setCounter(0);
    return SUPLA_CALCFG_RESULT_DONE;
  }
  return SUPLA_CALCFG_RESULT_NOT_SUPPORTED;
}

void IRAM_ATTR CSE_7759B_FG::onPulse() {
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
