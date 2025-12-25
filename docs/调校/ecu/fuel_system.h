#ifndef FUEL_SYSTEM_H
#define FUEL_SYSTEM_H

#include "ecu_config.h"
#include "ecu_types.h"
#include "sensors.h"

class FuelSystem {
public:
  FuelSystem();
  void initialize();
  void update(const SensorData& sensors, float rpm, const FuelConfig& config);
  float calculatePulseWidth(const SensorData& sensors, float rpm, const FuelConfig& config);
  void inject(uint8_t cylinder, float pulseWidthMs);
  
  // Table management
  void setVETable(const VETable& table) { veTable = table; }
  VETable getVETable() const { return veTable; }
  
private:
  VETable veTable;
  float lastPulseWidth;
  
  // Algorithm implementations
  float calculateSpeedDensity(float map, float iat, float rpm, float ve);
  float calculateAlphaN(float tps, float rpm, float ve);
  float calculateMAF(float mafVoltage, float iat, float rpm);
  
  // Lookup functions
  float lookupVE(float rpm, float load);
  float lookupAFR(float rpm, float load, const AFRTable& table);
  
  // Correction factors
  float calculateCoolantCorrection(float clt, const FuelConfig& config);
  float calculateIATCorrection(float iat, const FuelConfig& config);
  float calculateTPSCorrection(float tps, const FuelConfig& config);
  float calculateFlexFuelCorrection(float ethanolPercent, const FuelConfig& config);
  float applyInjectorDeadtime(float basePulseWidth, const FuelConfig& config);
  float applySmallPulseCorrection(float pulseWidth, const FuelConfig& config);
  
  // PID controller for closed-loop AFR control
  float pidOutput;
  float pidError;
  float pidIntegral;
  float pidLastError;
  float calculatePID(float targetAFR, float actualAFR, float dt);
};

#endif // FUEL_SYSTEM_H

