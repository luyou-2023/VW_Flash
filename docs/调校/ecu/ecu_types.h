#ifndef ECU_TYPES_H
#define ECU_TYPES_H

#include <stdint.h>

// Sensor Data Structure
struct SensorData {
  float tps;              // Throttle Position (0-100%)
  float map;              // Manifold Absolute Pressure (kPa)
  float iat;              // Intake Air Temperature (C)
  float clt;              // Coolant Temperature (C)
  float baro;             // Barometric Pressure (kPa)
  float fuelPressure;     // Fuel Pressure (kPa)
  float fuelLevel;        // Fuel Level (0-100%)
  float oilPressure;      // Oil Pressure (kPa)
  float afr;              // Air-Fuel Ratio
  float ethanolPercent;   // Ethanol percentage (0-100%)
  float vss;              // Vehicle Speed (km/h)
  bool brakePedal;        // Brake pedal state
  bool clutchPedal;       // Clutch pedal state
};

// Engine State Structure
struct EngineState {
  float rpm;              // Engine RPM
  float load;             // Engine load (MAP or TPS based on algorithm)
  float timingAdvance;    // Ignition advance (degrees BTDC)
  float fuelPulseWidth;   // Fuel injector pulse width (ms)
  float dwellTime;        // Ignition coil dwell time (ms)
  bool running;           // Engine running state
  bool cranking;          // Engine cranking state
  uint8_t currentCylinder; // Current firing cylinder (1-NUM_CYLINDERS)
  uint32_t lastCrankEvent; // Microseconds since last crank event
};

// Fuel Algorithm Configuration
struct FuelConfig {
  uint8_t algorithm;      // FUEL_ALGORITHM_SPEED_DENSITY, ALPHA_N, or MAF
  float targetAFR;        // Target Air-Fuel Ratio
  float coolantMultiplier; // Coolant temperature fuel correction
  float iatMultiplier;    // Intake air temperature fuel correction
  float tpsMultiplier;    // Throttle position fuel correction
  float injectorDeadtime; // Injector dead time (ms)
  float smallPulseCorrection; // Small pulse width correction factor
  bool flexFuelEnabled;   // Flex fuel enabled
  bool decelFuelCut;      // Deceleration fuel cut enabled
  float perCylinderTrim[6]; // Per-cylinder fuel trim (-50% to +50%)
  uint8_t injectionMode;  // Single or sequential injection
};

// Ignition Configuration
struct IgnitionConfig {
  uint8_t mode;           // IGN_MODE_FIXED or IGN_MODE_DYNAMIC
  float dwellTime;        // Coil dwell time (ms)
  float iatAdder;         // IAT ignition timing adder (degrees)
  float coolantCorrection; // Coolant temperature ignition correction (degrees)
  float perCylinderTrim[6]; // Per-cylinder ignition trim (degrees)
  uint8_t firingOrder[6]; // Firing order (cylinder numbers)
};

// Safety Status
struct SafetyStatus {
  bool tpsFault;
  bool mapFault;
  bool iatFault;
  bool cltFault;
  bool crankFault;
  bool camFault;
  bool wbo2Fault;
  bool fuelPressureFault;
  bool oilPressureFault;
  bool overvoltage;
  bool overtemperature;
  bool rpmLimitReached;
  bool safeMode;          // Overall safe mode flag
};

// VE Table (Volumetric Efficiency) - 16x16 RPM vs Load
struct VETable {
  uint16_t rpmBins[RPM_TABLE_SIZE];
  float loadBins[LOAD_TABLE_SIZE];
  float veValues[VE_TABLE_SIZE]; // VE values (0-200%)
};

// Ignition Advance Table - 16x16 RPM vs Load
struct IgnitionTable {
  uint16_t rpmBins[RPM_TABLE_SIZE];
  float loadBins[LOAD_TABLE_SIZE];
  float timingValues[IGN_TABLE_SIZE]; // Timing advance (degrees BTDC)
};

// AFR Target Table - 16x16 RPM vs Load
struct AFRTable {
  uint16_t rpmBins[RPM_TABLE_SIZE];
  float loadBins[LOAD_TABLE_SIZE];
  float afrValues[AFR_TABLE_SIZE]; // Target AFR values
};

#endif // ECU_TYPES_H

