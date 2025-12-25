#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include "ecu_config.h"
#include "ecu_types.h"
#include "sensors.h"

class SafetySystem {
public:
  SafetySystem();
  void initialize();
  void update(const SensorData& sensors, float rpm);
  SafetyStatus getStatus() const { return status; }
  bool isSafeToRun() const { return !status.safeMode; }
  void resetFaults();
  
private:
  SafetyStatus status;
  
  // Fault detection functions
  void checkTPSFault(const SensorData& sensors);
  void checkMAPFault(const SensorData& sensors);
  void checkIATFault(const SensorData& sensors);
  void checkCLTFault(const SensorData& sensors);
  void checkCrankFault(float rpm);
  void checkWBO2Fault(const SensorData& sensors);
  void checkFuelPressureFault(const SensorData& sensors);
  void checkOilPressureFault(const SensorData& sensors);
  void checkRPMLimit(float rpm);
  void checkOvervoltage();
  void checkOvertemperature(const SensorData& sensors);
  
  // Sensor range validation
  bool isInRange(float value, float min, float max);
};

#endif // SAFETY_SYSTEM_H

