#ifndef IGNITION_SYSTEM_H
#define IGNITION_SYSTEM_H

#include "ecu_config.h"
#include "ecu_types.h"
#include "sensors.h"

class IgnitionSystem {
public:
  IgnitionSystem();
  void initialize();
  void update(const SensorData& sensors, float rpm, const IgnitionConfig& config);
  float calculateTimingAdvance(float rpm, float load, const IgnitionConfig& config);
  float calculateTimingAdvance(float rpm, float load, float clt, float iat, const IgnitionConfig& config);
  void fireCoil(uint8_t cylinder, float advanceDegrees, float rpm, const IgnitionConfig& config);
  
  // Table management
  void setIgnitionTable(const IgnitionTable& table) { ignTable = table; }
  IgnitionTable getIgnitionTable() const { return ignTable; }
  
private:
  IgnitionTable ignTable;
  float lastTimingAdvance;
  
  // Lookup functions
  float lookupTiming(float rpm, float load);
  
  // Correction factors
  float calculateCoolantCorrection(float clt, const IgnitionConfig& config);
  float calculateIATAdder(float iat, const IgnitionConfig& config);
  float calculatePerCylinderTrim(uint8_t cylinder, const IgnitionConfig& config);
  
  // Timing calculation
  float degreesToMicroseconds(float degrees, float rpm);
  void scheduleIgnition(uint8_t cylinder, uint32_t delayMicroseconds);
};

#endif // IGNITION_SYSTEM_H

