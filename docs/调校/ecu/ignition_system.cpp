#include "ignition_system.h"
#include <Arduino.h>
#include <math.h>

IgnitionSystem::IgnitionSystem() {
  lastTimingAdvance = 0.0;
  
  // Initialize ignition table with default values
  // RPM bins
  for (int i = 0; i < RPM_TABLE_SIZE; i++) {
    ignTable.rpmBins[i] = 500 + (i * 500); // 500-8000 RPM
  }
  
  // Load bins (0-100% MAP or TPS)
  for (int i = 0; i < LOAD_TABLE_SIZE; i++) {
    ignTable.loadBins[i] = (i * 100.0) / (LOAD_TABLE_SIZE - 1);
  }
  
  // Initialize timing values (degrees BTDC)
  for (int i = 0; i < IGN_TABLE_SIZE; i++) {
    ignTable.timingValues[i] = 15.0; // Default 15 degrees BTDC
  }
}

void IgnitionSystem::initialize() {
  // Configure coil pins as outputs
  pinMode(PIN_COIL_CYL1, OUTPUT);
  pinMode(PIN_COIL_CYL2, OUTPUT);
  pinMode(PIN_COIL_CYL3, OUTPUT);
  pinMode(PIN_COIL_CYL4, OUTPUT);
  if (NUM_CYLINDERS > 4) {
    pinMode(PIN_COIL_CYL5, OUTPUT);
    pinMode(PIN_COIL_CYL6, OUTPUT);
  }
  
  // Ensure coils are off
  digitalWrite(PIN_COIL_CYL1, LOW);
  digitalWrite(PIN_COIL_CYL2, LOW);
  digitalWrite(PIN_COIL_CYL3, LOW);
  digitalWrite(PIN_COIL_CYL4, LOW);
  if (NUM_CYLINDERS > 4) {
    digitalWrite(PIN_COIL_CYL5, LOW);
    digitalWrite(PIN_COIL_CYL6, LOW);
  }
}

float IgnitionSystem::calculateTimingAdvance(float rpm, float load, const IgnitionConfig& config) {
  return calculateTimingAdvance(rpm, load, 20.0, 20.0, config); // Default temp values
}

float IgnitionSystem::calculateTimingAdvance(float rpm, float load, float clt, float iat, const IgnitionConfig& config) {
  if (rpm < MIN_RPM) {
    return 0.0;
  }
  
  float baseTiming = 0.0;
  
  if (config.mode == IGN_MODE_DYNAMIC) {
    baseTiming = lookupTiming(rpm, load);
  } else {
    baseTiming = 10.0; // Fixed timing mode
  }
  
  // Apply corrections
  baseTiming += calculateCoolantCorrection(clt, config);
  baseTiming += calculateIATAdder(iat, config);
  
  // Limit timing advance
  if (baseTiming > 45.0) baseTiming = 45.0;
  if (baseTiming < -10.0) baseTiming = -10.0;
  
  lastTimingAdvance = baseTiming;
  return baseTiming;
}

float IgnitionSystem::lookupTiming(float rpm, float load) {
  // Bilinear interpolation in ignition table (similar to VE lookup)
  int rpmIdx = 0;
  for (int i = 0; i < RPM_TABLE_SIZE - 1; i++) {
    if (rpm >= ignTable.rpmBins[i] && rpm < ignTable.rpmBins[i + 1]) {
      rpmIdx = i;
      break;
    }
  }
  if (rpm >= ignTable.rpmBins[RPM_TABLE_SIZE - 1]) {
    rpmIdx = RPM_TABLE_SIZE - 2;
  }
  
  int loadIdx = 0;
  for (int i = 0; i < LOAD_TABLE_SIZE - 1; i++) {
    if (load >= ignTable.loadBins[i] && load < ignTable.loadBins[i + 1]) {
      loadIdx = i;
      break;
    }
  }
  if (load >= ignTable.loadBins[LOAD_TABLE_SIZE - 1]) {
    loadIdx = LOAD_TABLE_SIZE - 2;
  }
  
  // Get four corner values
  float tim11 = ignTable.timingValues[rpmIdx * LOAD_TABLE_SIZE + loadIdx];
  float tim12 = ignTable.timingValues[rpmIdx * LOAD_TABLE_SIZE + (loadIdx + 1)];
  float tim21 = ignTable.timingValues[(rpmIdx + 1) * LOAD_TABLE_SIZE + loadIdx];
  float tim22 = ignTable.timingValues[(rpmIdx + 1) * LOAD_TABLE_SIZE + (loadIdx + 1)];
  
  // Interpolation factors
  float rpmRatio = (rpm - ignTable.rpmBins[rpmIdx]) / 
                   (ignTable.rpmBins[rpmIdx + 1] - ignTable.rpmBins[rpmIdx]);
  float loadRatio = (load - ignTable.loadBins[loadIdx]) / 
                    (ignTable.loadBins[loadIdx + 1] - ignTable.loadBins[loadIdx]);
  
  // Bilinear interpolation
  float tim1 = tim11 + (tim12 - tim11) * loadRatio;
  float tim2 = tim21 + (tim22 - tim21) * loadRatio;
  float timing = tim1 + (tim2 - tim1) * rpmRatio;
  
  return timing;
}

float IgnitionSystem::calculateCoolantCorrection(float clt, const IgnitionConfig& config) {
  // Coolant temperature ignition correction
  // Cold engines may need more advance, hot engines may need less
  // This is a placeholder - actual implementation needs CLT sensor data
  return config.coolantCorrection;
}

float IgnitionSystem::calculateIATAdder(float iat, const IgnitionConfig& config) {
  // Intake air temperature adder
  // Hot air may need less advance to prevent knock
  float adder = config.iatAdder;
  if (iat > 40.0) {
    adder -= (iat - 40.0) * 0.1; // Reduce advance for hot intake air
  }
  return adder;
}

float IgnitionSystem::calculatePerCylinderTrim(uint8_t cylinder, const IgnitionConfig& config) {
  if (cylinder > 0 && cylinder <= NUM_CYLINDERS) {
    return config.perCylinderTrim[cylinder - 1];
  }
  return 0.0;
}

float IgnitionSystem::degreesToMicroseconds(float degrees, float rpm) {
  // Convert degrees of crankshaft rotation to microseconds
  // At given RPM: degrees per second = (rpm / 60) * 360
  // microseconds per degree = 1,000,000 / (degrees per second)
  
  float degreesPerSecond = (rpm / 60.0) * 360.0;
  float microsecondsPerDegree = 1000000.0 / degreesPerSecond;
  return degrees * microsecondsPerDegree;
}

void IgnitionSystem::update(const SensorData& sensors, float rpm, const IgnitionConfig& config) {
  float load = (config.mode == IGN_MODE_DYNAMIC) ? sensors.map : sensors.tps;
  float timing = calculateTimingAdvance(rpm, load, sensors.clt, sensors.iat, config);
  lastTimingAdvance = timing;
}

void IgnitionSystem::fireCoil(uint8_t cylinder, float advanceDegrees, float rpm, const IgnitionConfig& config) {
  if (rpm < MIN_RPM) {
    return;
  }
  
  uint8_t pin = 0;
  switch (cylinder) {
    case 1: pin = PIN_COIL_CYL1; break;
    case 2: pin = PIN_COIL_CYL2; break;
    case 3: pin = PIN_COIL_CYL3; break;
    case 4: pin = PIN_COIL_CYL4; break;
    case 5: pin = PIN_COIL_CYL5; break;
    case 6: pin = PIN_COIL_CYL6; break;
    default: return;
  }
  
  // Calculate dwell time
  float dwellMs = config.dwellTime;
  if (dwellMs < MIN_DWELL_MS) dwellMs = MIN_DWELL_MS;
  if (dwellMs > MAX_DWELL_MS) dwellMs = MAX_DWELL_MS;
  
  // Start charging coil
  digitalWrite(pin, HIGH);
  delayMicroseconds((uint32_t)(dwellMs * 1000.0));
  
  // Fire coil (turn off to create spark)
  digitalWrite(pin, LOW);
  
  // Note: In a real implementation, timing would be calculated based on crank position
  // and scheduled using hardware timers for precise timing control
}

void IgnitionSystem::scheduleIgnition(uint8_t cylinder, uint32_t delayMicroseconds) {
  // This would use hardware timers for precise ignition timing
  // Implementation depends on specific Arduino model and timer availability
  // For now, this is a placeholder for future timer-based implementation
}

