#ifdef SUPLA_CSE7759B_FG
#ifndef _cse_7759b_fg_gui_h
#define _cse_7759b_fg_gui_h

#include <Arduino.h>
#include <supla/sensor/one_phase_electricity_meter.h>
#include <supla/storage/storage.h>

namespace Supla {
namespace Sensor {

class CSE_7759B_FG : public OnePhaseElectricityMeter {
 public:
  explicit CSE_7759B_FG(int8_t pinCF);

  void onInit();
  void readValuesFromDevice();
  void onSaveState();
  void onLoadState();

  _supla_int64_t getCounter() const;
  void setCounter(_supla_int64_t value);

  uint32_t getPulseConstant() const;
  void setPulseConstant(uint32_t value);

  int handleCalcfgFromServer(TSD_DeviceCalCfgRequest *request);

 protected:
  static void IRAM_ATTR onPulse();

  int8_t pinCF;
  uint64_t energyBase = 0;
  uint64_t energy = 0;
  uint32_t pulseConstant = 1000;
  uint32_t pulseOffset = 0;

  static CSE_7759B_FG *instance;
  volatile uint32_t pulseCount = 0;
  volatile uint32_t lastPulseUs = 0;
  volatile uint32_t pulsePeriodUs = 0;
};

};  // namespace Sensor
};  // namespace Supla

#endif
#endif
