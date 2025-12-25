/*
 * FOME ECU Implementation for Arduino
 * Main ECU Control Loop
 * 
 * This implementation provides:
 * - Sensor input processing
 * - Fuel injection control (Alpha-N, MAF, Speed Density)
 * - Ignition timing control
 * - Safety and protection logic
 * 
 * Hardware: Arduino Mega 2560, Due, or Teensy 32-bit
 */

#include "ecu_config.h"
#include "ecu_types.h"
#include "sensors.h"
#include "fuel_system.h"
#include "ignition_system.h"
#include "safety_system.h"

// Global ECU objects
SensorManager sensorManager;
FuelSystem fuelSystem;
IgnitionSystem ignitionSystem;
SafetySystem safetySystem;

// Engine state
EngineState engineState;
FuelConfig fuelConfig;
IgnitionConfig ignitionConfig;

// Timing variables
uint32_t lastLoopTime = 0;
const uint32_t LOOP_TIME_MS = 10; // 10ms = 100Hz update rate

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("FOME ECU Initializing...");
  
  // Initialize all subsystems
  sensorManager.initialize();
  fuelSystem.initialize();
  ignitionSystem.initialize();
  safetySystem.initialize();
  
  // Configure fuel system
  fuelConfig.algorithm = FUEL_ALGORITHM_SPEED_DENSITY;
  fuelConfig.targetAFR = 14.7;
  fuelConfig.coolantMultiplier = 1.0;
  fuelConfig.iatMultiplier = 1.0;
  fuelConfig.tpsMultiplier = 1.0;
  fuelConfig.injectorDeadtime = INJECTOR_DEADTIME_US;
  fuelConfig.smallPulseCorrection = 1.0;
  fuelConfig.flexFuelEnabled = false;
  fuelConfig.decelFuelCut = true;
  fuelConfig.injectionMode = 1; // Sequential injection
  for (int i = 0; i < NUM_CYLINDERS; i++) {
    fuelConfig.perCylinderTrim[i] = 0.0;
  }
  
  // Configure ignition system
  ignitionConfig.mode = IGN_MODE_DYNAMIC;
  ignitionConfig.dwellTime = DWELL_TIME_MS;
  ignitionConfig.iatAdder = 0.0;
  ignitionConfig.coolantCorrection = 0.0;
  // Default firing order: 1-3-4-2 (4-cylinder)
  ignitionConfig.firingOrder[0] = 1;
  ignitionConfig.firingOrder[1] = 3;
  ignitionConfig.firingOrder[2] = 4;
  ignitionConfig.firingOrder[3] = 2;
  for (int i = 0; i < NUM_CYLINDERS; i++) {
    ignitionConfig.perCylinderTrim[i] = 0.0;
  }
  
  // Initialize engine state
  engineState.rpm = 0.0;
  engineState.load = 0.0;
  engineState.timingAdvance = 0.0;
  engineState.fuelPulseWidth = 0.0;
  engineState.dwellTime = DWELL_TIME_MS;
  engineState.running = false;
  engineState.cranking = false;
  engineState.currentCylinder = 1;
  engineState.lastCrankEvent = 0;
  
  // Allow sensors to stabilize
  delay(500);
  
  Serial.println("FOME ECU Ready");
}

void loop() {
  // Control loop timing
  uint32_t currentTime = millis();
  if (currentTime - lastLoopTime < LOOP_TIME_MS) {
    delay(1);
    return;
  }
  float dt = (currentTime - lastLoopTime) / 1000.0; // Delta time in seconds
  lastLoopTime = currentTime;
  
  // Update sensors
  sensorManager.update();
  SensorData sensors = sensorManager.getData();
  
  // Update engine state
  engineState.rpm = sensors.rpm;
  engineState.load = (fuelConfig.algorithm == FUEL_ALGORITHM_SPEED_DENSITY) ? 
                      sensors.map : sensors.tps;
  
  // Determine engine state
  engineState.cranking = (engineState.rpm > 0 && engineState.rpm < 500);
  engineState.running = (engineState.rpm > 500);
  
  // Update safety system
  safetySystem.update(sensors, engineState.rpm);
  SafetyStatus safety = safetySystem.getStatus();
  
  // Check if safe to run
  if (!safetySystem.isSafeToRun() && engineState.running) {
    // Enter safe mode - cut fuel and ignition
    Serial.println("SAFE MODE ACTIVATED - Cutting fuel and ignition");
    engineState.running = false;
    // Turn off injectors and coils
    fuelSystem.inject(1, 0.0);
    fuelSystem.inject(2, 0.0);
    fuelSystem.inject(3, 0.0);
    fuelSystem.inject(4, 0.0);
    digitalWrite(PIN_COIL_CYL1, LOW);
    digitalWrite(PIN_COIL_CYL2, LOW);
    digitalWrite(PIN_COIL_CYL3, LOW);
    digitalWrite(PIN_COIL_CYL4, LOW);
    return;
  }
  
  // Calculate fuel injection
  if (engineState.running || engineState.cranking) {
    fuelSystem.update(sensors, engineState.rpm, fuelConfig);
    engineState.fuelPulseWidth = fuelSystem.calculatePulseWidth(sensors, engineState.rpm, fuelConfig);
    
    // Apply per-cylinder trim
    for (uint8_t cyl = 1; cyl <= NUM_CYLINDERS; cyl++) {
      float trimmedPulseWidth = engineState.fuelPulseWidth * 
                                (1.0 + fuelConfig.perCylinderTrim[cyl - 1] / 100.0);
      // Note: In sequential mode, inject at proper timing
      // For now, this is simplified
      // fuelSystem.inject(cyl, trimmedPulseWidth);
    }
  } else {
    engineState.fuelPulseWidth = 0.0;
  }
  
  // Calculate ignition timing
  if (engineState.running || engineState.cranking) {
    ignitionSystem.update(sensors, engineState.rpm, ignitionConfig);
    engineState.timingAdvance = ignitionSystem.calculateTimingAdvance(
      engineState.rpm, engineState.load, sensors.clt, sensors.iat, ignitionConfig);
    
    // Apply per-cylinder trim
    for (uint8_t cyl = 1; cyl <= NUM_CYLINDERS; cyl++) {
      float trimmedTiming = engineState.timingAdvance + 
                           ignitionConfig.perCylinderTrim[cyl - 1];
      // Note: In sequential mode, fire at proper timing
      // For now, this is simplified
      // ignitionSystem.fireCoil(cyl, trimmedTiming, engineState.rpm, ignitionConfig);
    }
  } else {
    engineState.timingAdvance = 0.0;
  }
  
  // Debug output (every 100ms = 10 loops)
  static uint8_t debugCounter = 0;
  if (++debugCounter >= 10) {
    debugCounter = 0;
    printDebugInfo(sensors, engineState, safety);
  }
}

void printDebugInfo(const SensorData& sensors, const EngineState& state, const SafetyStatus& safety) {
  Serial.print("RPM: ");
  Serial.print(state.rpm);
  Serial.print(" | TPS: ");
  Serial.print(sensors.tps, 1);
  Serial.print("% | MAP: ");
  Serial.print(sensors.map, 1);
  Serial.print(" kPa | IAT: ");
  Serial.print(sensors.iat, 1);
  Serial.print("C | CLT: ");
  Serial.print(sensors.clt, 1);
  Serial.print("C | AFR: ");
  Serial.print(sensors.afr, 2);
  Serial.print(" | Fuel PW: ");
  Serial.print(state.fuelPulseWidth, 2);
  Serial.print(" ms | Timing: ");
  Serial.print(state.timingAdvance, 1);
  Serial.print(" deg");
  
  if (safety.safeMode) {
    Serial.print(" | SAFE MODE");
  }
  
  Serial.println();
}

