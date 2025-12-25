#include "safety_system.h"
#include <Arduino.h>
#include <string.h>

SafetySystem::SafetySystem() {
  memset(&status, 0, sizeof(SafetyStatus));
}

void SafetySystem::initialize() {
  resetFaults();
}

void SafetySystem::update(const SensorData& sensors, float rpm) {
  // Reset safe mode flag (will be set if any fault is detected)
  status.safeMode = false;
  
  // Check all sensors and conditions
  checkTPSFault(sensors);
  checkMAPFault(sensors);
  checkIATFault(sensors);
  checkCLTFault(sensors);
  checkCrankFault(rpm);
  checkWBO2Fault(sensors);
  checkFuelPressureFault(sensors);
  checkOilPressureFault(sensors);
  checkRPMLimit(rpm);
  checkOvervoltage();
  checkOvertemperature(sensors);
  
  // Set safe mode if any critical fault is detected
  if (status.tpsFault || status.mapFault || status.cltFault || 
      status.crankFault || status.fuelPressureFault || 
      status.rpmLimitReached || status.overvoltage || status.overtemperature) {
    status.safeMode = true;
  }
}

void SafetySystem::checkTPSFault(const SensorData& sensors) {
  if (!isInRange(sensors.tps, 0.0, 100.0)) {
    status.tpsFault = true;
  } else {
    status.tpsFault = false;
  }
}

void SafetySystem::checkMAPFault(const SensorData& sensors) {
  if (!isInRange(sensors.map, MAP_MIN_KPA, MAP_MAX_KPA)) {
    status.mapFault = true;
  } else {
    status.mapFault = false;
  }
}

void SafetySystem::checkIATFault(const SensorData& sensors) {
  if (!isInRange(sensors.iat, TEMP_MIN_C, TEMP_MAX_C)) {
    status.iatFault = true;
  } else {
    status.iatFault = false;
  }
}

void SafetySystem::checkCLTFault(const SensorData& sensors) {
  if (!isInRange(sensors.clt, TEMP_MIN_C, TEMP_MAX_C) || 
      sensors.clt > MAX_CLT_C) {
    status.cltFault = true;
  } else {
    status.cltFault = false;
  }
}

void SafetySystem::checkCrankFault(float rpm) {
  // Crank signal failure if RPM is 0 when engine should be running
  // or if RPM is unrealistic
  if (rpm > MAX_RPM || (rpm < MIN_RPM && rpm > 0)) {
    status.crankFault = true;
  } else {
    status.crankFault = false;
  }
}

void SafetySystem::checkWBO2Fault(const SensorData& sensors) {
  if (!isInRange(sensors.afr, AFR_MIN, AFR_MAX)) {
    status.wbo2Fault = true;
  } else {
    status.wbo2Fault = false;
  }
}

void SafetySystem::checkFuelPressureFault(const SensorData& sensors) {
  if (sensors.fuelPressure < MIN_FUEL_PRESSURE_KPA) {
    status.fuelPressureFault = true;
  } else {
    status.fuelPressureFault = false;
  }
}

void SafetySystem::checkOilPressureFault(const SensorData& sensors) {
  // Only check if engine is running (RPM > 0)
  static float lastRPM = 0.0;
  if (lastRPM > 500 && sensors.oilPressure < MIN_OIL_PRESSURE_KPA) {
    status.oilPressureFault = true;
  } else {
    status.oilPressureFault = false;
  }
  lastRPM = 0.0; // Would be updated from engine state
}

void SafetySystem::checkRPMLimit(float rpm) {
  if (rpm > MAX_RPM_LIMIT) {
    status.rpmLimitReached = true;
  } else {
    status.rpmLimitReached = false;
  }
}

void SafetySystem::checkOvervoltage() {
  // Check system voltage (simplified - would need voltage divider)
  // Assuming analogRead on a voltage divider connected to system voltage
  // For now, this is a placeholder
  float systemVoltage = (analogRead(A15) / 1024.0) * 5.0 * 4.0; // Assuming 4:1 divider
  
  if (systemVoltage > 16.0) { // Overvoltage threshold
    status.overvoltage = true;
  } else {
    status.overvoltage = false;
  }
}

void SafetySystem::checkOvertemperature(const SensorData& sensors) {
  if (sensors.clt > MAX_CLT_C || sensors.iat > MAX_IAT_C) {
    status.overtemperature = true;
  } else {
    status.overtemperature = false;
  }
}

bool SafetySystem::isInRange(float value, float min, float max) {
  return (value >= min && value <= max);
}

void SafetySystem::resetFaults() {
  memset(&status, 0, sizeof(SafetyStatus));
}

