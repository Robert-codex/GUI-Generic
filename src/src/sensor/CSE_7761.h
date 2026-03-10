#ifdef SUPLA_CSE7761
#ifndef _cse7761_gui_h
#define _cse7761_gui_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <supla/sensor/one_phase_electricity_meter.h>
#include <supla/storage/storage.h>

namespace Supla {
namespace Sensor {

class CSE_7761 : public OnePhaseElectricityMeter {
 public:
  CSE_7761(HardwareSerial &serial, int8_t pinRX, int8_t pinTX, uint8_t currentChannel = 0);

  void onInit();
  void readValuesFromDevice();
  void onSaveState();
  void onLoadState();

  void setCounter(_supla_int64_t value);
  _supla_int64_t getCounter() const;
  uint8_t getCurrentChannel() const;
  void setCurrentChannel(uint8_t value);

 protected:
  enum CoefficientIndex {
    RMS_IAC = 0,
    RMS_IBC,
    RMS_UC,
    POWER_PAC,
    POWER_PBC,
    POWER_SC,
    ENERGY_AC,
    ENERGY_BC
  };

  bool chipInit();
  void writeRegister(uint8_t reg, uint16_t data);
  bool readOnce(uint8_t reg, uint8_t size, uint32_t *value);
  uint32_t readRegister(uint8_t reg, uint8_t size);
  uint32_t coefficientByUnit(CoefficientIndex unit) const;
  float computeApparentPower(float voltage, float current) const;

  HardwareSerial &serial;
  int8_t pinRX;
  int8_t pinTX;
  uint8_t currentChannel;
  uint16_t coefficients[8] = {0};
  uint32_t voltageRms = 0;
  uint32_t currentRms[2] = {0, 0};
  uint32_t activePower[2] = {0, 0};
  uint64_t energy = 0;
  uint64_t energyAtLoad = 0;
  uint32_t lastReadMs = 0;
  bool ready = false;
};

};  // namespace Sensor
};  // namespace Supla

#endif
#endif
