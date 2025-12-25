#include "fuel_system.h"
#include <Arduino.h>
#include <math.h>

FuelSystem::FuelSystem() {
  lastPulseWidth = 0.0;
  pidOutput = 0.0;
  pidError = 0.0;
  pidIntegral = 0.0;
  pidLastError = 0.0;
  
  // Initialize VE table with default values
  // RPM bins (example values, should be tuned)
  for (int i = 0; i < RPM_TABLE_SIZE; i++) {
    veTable.rpmBins[i] = 500 + (i * 500); // 500-8000 RPM
  }
  
  // Load bins (0-100% MAP or TPS)
  for (int i = 0; i < LOAD_TABLE_SIZE; i++) {
    veTable.loadBins[i] = (i * 100.0) / (LOAD_TABLE_SIZE - 1);
  }
  
  // Initialize VE values (should be tuned per engine)
  for (int i = 0; i < VE_TABLE_SIZE; i++) {
    veTable.veValues[i] = 80.0; // Default 80% VE
  }
}

void FuelSystem::initialize() {
  // Configure injector pins as outputs
  pinMode(PIN_INJ_CYL1, OUTPUT);
  pinMode(PIN_INJ_CYL2, OUTPUT);
  pinMode(PIN_INJ_CYL3, OUTPUT);
  pinMode(PIN_INJ_CYL4, OUTPUT);
  if (NUM_CYLINDERS > 4) {
    pinMode(PIN_INJ_CYL5, OUTPUT);
    pinMode(PIN_INJ_CYL6, OUTPUT);
  }
  
  // Ensure injectors are off
  digitalWrite(PIN_INJ_CYL1, LOW);
  digitalWrite(PIN_INJ_CYL2, LOW);
  digitalWrite(PIN_INJ_CYL3, LOW);
  digitalWrite(PIN_INJ_CYL4, LOW);
  if (NUM_CYLINDERS > 4) {
    digitalWrite(PIN_INJ_CYL5, LOW);
    digitalWrite(PIN_INJ_CYL6, LOW);
  }
}

float FuelSystem::calculatePulseWidth(const SensorData& sensors, float rpm, const FuelConfig& config) {
  if (rpm < MIN_RPM) {
    return 0.0;
  }
  
  float basePulseWidth = 0.0;
  float load = 0.0;
  
  // Determine engine load based on algorithm
  switch (config.algorithm) {
    case FUEL_ALGORITHM_SPEED_DENSITY:
      load = sensors.map; // Use MAP as load
      basePulseWidth = calculateSpeedDensity(sensors.map, sensors.iat, rpm, lookupVE(rpm, load));
      break;
      
    case FUEL_ALGORITHM_ALPHA_N:
      load = sensors.tps; // Use TPS as load
      basePulseWidth = calculateAlphaN(sensors.tps, rpm, lookupVE(rpm, load));
      break;
      
    case FUEL_ALGORITHM_MAF:
      // For MAF, load would be MAF voltage/flow
      load = sensors.tps; // Fallback to TPS if MAF not available
      basePulseWidth = calculateMAF(sensors.tps * 5.0 / 100.0, sensors.iat, rpm); // Simplified
      break;
  }
  
  // Apply corrections
  basePulseWidth *= calculateCoolantCorrection(sensors.clt, config);
  basePulseWidth *= calculateIATCorrection(sensors.iat, config);
  basePulseWidth *= calculateTPSCorrection(sensors.tps, config);
  
  if (config.flexFuelEnabled) {
    basePulseWidth *= calculateFlexFuelCorrection(sensors.ethanolPercent, config);
  }
  
  // Closed-loop AFR correction (PID)
  if (sensors.afr > AFR_MIN && sensors.afr < AFR_MAX) {
    float targetAFR = config.targetAFR; // Could be from AFR table
    float dt = 0.01; // Approximate loop time in seconds
    float pidCorrection = calculatePID(targetAFR, sensors.afr, dt);
    basePulseWidth *= (1.0 + pidCorrection);
  }
  
  // Apply injector deadtime
  basePulseWidth = applyInjectorDeadtime(basePulseWidth, config);
  
  // Apply small pulse correction
  basePulseWidth = applySmallPulseCorrection(basePulseWidth, config);
  
  // Deceleration fuel cut
  if (config.decelFuelCut && sensors.tps < 5.0 && rpm > 2000) {
    basePulseWidth = 0.0;
  }
  
  // Minimum pulse width check
  if (basePulseWidth < (MIN_INJECTOR_PULSE_US / 1000.0)) {
    basePulseWidth = 0.0;
  }
  
  lastPulseWidth = basePulseWidth;
  return basePulseWidth;
}

float FuelSystem::calculateSpeedDensity(float map, float iat, float rpm, float ve) {
  // Speed-Density formula: Fuel = (MAP * VE * RPM) / (IAT * constant)
  // Simplified calculation - actual formula depends on injector size, engine displacement, etc.
  
  // Standard temperature and pressure correction
  float iatKelvin = iat + 273.15;
  float airDensity = (map * 100.0) / (287.05 * iatKelvin); // kg/m³
  
  // Fuel mass calculation (simplified)
  float airMassPerRev = (ve / 100.0) * airDensity * 0.001; // Simplified
  float fuelMassPerRev = airMassPerRev / 14.7; // Stoichiometric ratio
  
  // Convert to pulse width (simplified - needs injector flow rate)
  // Assuming ~12ms injector pulse at 100% duty cycle for typical injector
  float pulseWidth = (fuelMassPerRev * rpm * 12.0) / 6000.0;
  
  return pulseWidth;
}

float FuelSystem::calculateAlphaN(float tps, float rpm, float ve) {
  // Alpha-N: Fuel based on throttle position and RPM
  // Simplified calculation
  float loadFactor = tps / 100.0;
  float rpmFactor = rpm / 6000.0;
  float veFactor = ve / 100.0;
  
  float pulseWidth = (loadFactor * rpmFactor * veFactor * 12.0);
  return pulseWidth;
}

float FuelSystem::calculateMAF(float mafVoltage, float iat, float rpm) {
  // MAF-based calculation (simplified)
  // MAF voltage typically represents airflow
  float airFlow = mafVoltage * 100.0; // Simplified conversion
  float fuelFlow = airFlow / 14.7;
  float pulseWidth = (fuelFlow * rpm * 12.0) / 6000.0;
  return pulseWidth;
}

float FuelSystem::lookupVE(float rpm, float load) {
  // Bilinear interpolation in VE table
  // Find RPM bin
  int rpmIdx = 0;
  for (int i = 0; i < RPM_TABLE_SIZE - 1; i++) {
    if (rpm >= veTable.rpmBins[i] && rpm < veTable.rpmBins[i + 1]) {
      rpmIdx = i;
      break;
    }
  }
  if (rpm >= veTable.rpmBins[RPM_TABLE_SIZE - 1]) {
    rpmIdx = RPM_TABLE_SIZE - 2;
  }
  
  // Find load bin
  int loadIdx = 0;
  for (int i = 0; i < LOAD_TABLE_SIZE - 1; i++) {
    if (load >= veTable.loadBins[i] && load < veTable.loadBins[i + 1]) {
      loadIdx = i;
      break;
    }
  }
  if (load >= veTable.loadBins[LOAD_TABLE_SIZE - 1]) {
    loadIdx = LOAD_TABLE_SIZE - 2;
  }
  
  // Get four corner values
  float ve11 = veTable.veValues[rpmIdx * LOAD_TABLE_SIZE + loadIdx];
  float ve12 = veTable.veValues[rpmIdx * LOAD_TABLE_SIZE + (loadIdx + 1)];
  float ve21 = veTable.veValues[(rpmIdx + 1) * LOAD_TABLE_SIZE + loadIdx];
  float ve22 = veTable.veValues[(rpmIdx + 1) * LOAD_TABLE_SIZE + (loadIdx + 1)];
  
  // Interpolation factors
  float rpmRatio = (rpm - veTable.rpmBins[rpmIdx]) / 
                   (veTable.rpmBins[rpmIdx + 1] - veTable.rpmBins[rpmIdx]);
  float loadRatio = (load - veTable.loadBins[loadIdx]) / 
                    (veTable.loadBins[loadIdx + 1] - veTable.loadBins[loadIdx]);
  
  // Bilinear interpolation
  float ve1 = ve11 + (ve12 - ve11) * loadRatio;
  float ve2 = ve21 + (ve22 - ve21) * loadRatio;
  float ve = ve1 + (ve2 - ve1) * rpmRatio;
  
  return ve;
}

float FuelSystem::lookupAFR(float rpm, float load, const AFRTable& table) {
  // Similar to lookupVE but for AFR table
  // Implementation would be identical to lookupVE
  return 14.7; // Default stoichiometric
}

float FuelSystem::calculateCoolantCorrection(float clt, const FuelConfig& config) {
  // Cold engine enrichment
  if (clt < 70.0) {
    return 1.0 + ((70.0 - clt) / 70.0) * 0.5; // Up to 50% enrichment when cold
  }
  return config.coolantMultiplier;
}

float FuelSystem::calculateIATCorrection(float iat, const FuelConfig& config) {
  // Hot air is less dense, needs less fuel
  if (iat > 25.0) {
    return 1.0 - ((iat - 25.0) / 100.0) * 0.1; // Slight reduction for hot air
  }
  return config.iatMultiplier;
}

float FuelSystem::calculateTPSCorrection(float tps, const FuelConfig& config) {
  // TPS-based corrections (acceleration enrichment, etc.)
  return config.tpsMultiplier;
}

float FuelSystem::calculateFlexFuelCorrection(float ethanolPercent, const FuelConfig& config) {
  // Ethanol requires more fuel (stoichiometric AFR ~9.0 vs gasoline ~14.7)
  if (!config.flexFuelEnabled) {
    return 1.0;
  }
  
  // Linear interpolation: 0% ethanol = 1.0, 100% ethanol = ~1.6 (14.7/9.0)
  return 1.0 + (ethanolPercent / 100.0) * 0.6;
}

float FuelSystem::applyInjectorDeadtime(float basePulseWidth, const FuelConfig& config) {
  // Add deadtime to base pulse width
  float deadtimeMs = config.injectorDeadtime / 1000.0; // Convert us to ms
  return basePulseWidth + deadtimeMs;
}

float FuelSystem::applySmallPulseCorrection(float pulseWidth, const FuelConfig& config) {
  // Small pulses may not deliver fuel proportionally
  if (pulseWidth < 2.0) { // Less than 2ms
    return pulseWidth * config.smallPulseCorrection;
  }
  return pulseWidth;
}

float FuelSystem::calculatePID(float targetAFR, float actualAFR, float dt) {
  // Simple PID controller for closed-loop AFR control
  float Kp = 0.1; // Proportional gain
  float Ki = 0.01; // Integral gain
  float Kd = 0.05; // Derivative gain
  
  pidError = targetAFR - actualAFR;
  pidIntegral += pidError * dt;
  
  // Anti-windup
  if (pidIntegral > 0.5) pidIntegral = 0.5;
  if (pidIntegral < -0.5) pidIntegral = -0.5;
  
  float derivative = (pidError - pidLastError) / dt;
  pidLastError = pidError;
  
  pidOutput = (Kp * pidError) + (Ki * pidIntegral) + (Kd * derivative);
  
  // Limit output to ±20% correction
  if (pidOutput > 0.2) pidOutput = 0.2;
  if (pidOutput < -0.2) pidOutput = -0.2;
  
  return pidOutput;
}

void FuelSystem::update(const SensorData& sensors, float rpm, const FuelConfig& config) {
  float pulseWidth = calculatePulseWidth(sensors, rpm, config);
  lastPulseWidth = pulseWidth;
}

void FuelSystem::inject(uint8_t cylinder, float pulseWidthMs) {
  if (pulseWidthMs < (MIN_INJECTOR_PULSE_US / 1000.0)) {
    return;
  }
  
  uint8_t pin = 0;
  switch (cylinder) {
    case 1: pin = PIN_INJ_CYL1; break;
    case 2: pin = PIN_INJ_CYL2; break;
    case 3: pin = PIN_INJ_CYL3; break;
    case 4: pin = PIN_INJ_CYL4; break;
    case 5: pin = PIN_INJ_CYL5; break;
    case 6: pin = PIN_INJ_CYL6; break;
    default: return;
  }
  
  // Trigger injector (non-blocking implementation would use timers)
  digitalWrite(pin, HIGH);
  delayMicroseconds((uint32_t)(pulseWidthMs * 1000.0));
  digitalWrite(pin, LOW);
}

