#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

// Hardware Configuration
#define ARDUINO_MEGA 1  // Change to 0 for Due/Teensy
#define USE_CAN_BUS 1

// Sensor Pin Definitions
// Analog Inputs
#define PIN_TPS_CH1 A0      // Throttle Position Sensor Channel 1
#define PIN_TPS_CH2 A1      // Throttle Position Sensor Channel 2 (redundant)
#define PIN_MAP A2          // Manifold Absolute Pressure
#define PIN_IAT A3          // Intake Air Temperature
#define PIN_CLT A4          // Coolant Temperature
#define PIN_FUEL_PRESSURE A5
#define PIN_FUEL_LEVEL A6
#define PIN_BARO A7         // Barometric Pressure
#define PIN_WBO2 A8         // Wideband O2 Sensor
#define PIN_FLEX_FUEL A9    // Flex Fuel Sensor

// Digital Inputs
#define PIN_CRANK_SENSOR 2      // Interrupt-capable pin for crankshaft sensor
#define PIN_CAM_SENSOR 3        // Interrupt-capable pin for camshaft sensor
#define PIN_BRAKE_PEDAL 4
#define PIN_CLUTCH_PEDAL 5
#define PIN_VSS 6               // Vehicle Speed Sensor
#define PIN_TURBO_SPEED 7

// Output Pins
// Injectors (PWM capable)
#define PIN_INJ_CYL1 9
#define PIN_INJ_CYL2 10
#define PIN_INJ_CYL3 11
#define PIN_INJ_CYL4 12
#define PIN_INJ_CYL5 44  // Arduino Mega
#define PIN_INJ_CYL6 45

// Ignition Coils
#define PIN_COIL_CYL1 22
#define PIN_COIL_CYL2 23
#define PIN_COIL_CYL3 24
#define PIN_COIL_CYL4 25
#define PIN_COIL_CYL5 26
#define PIN_COIL_CYL6 27

// CAN Bus (MCP2515 SPI pins if using module)
#define PIN_CAN_CS 53

// System Configuration
#define NUM_CYLINDERS 4         // Change based on engine
#define MAX_RPM 8000
#define MIN_RPM 100
#define CRANK_TEETH 60          // Number of teeth on crank trigger wheel
#define MISSING_TEETH 2         // Missing teeth pattern

// Timing Constants
#define MICROSECONDS_PER_MILLISECOND 1000UL
#define MILLISECONDS_PER_SECOND 1000UL
#define MICROSECONDS_PER_SECOND 1000000UL

// Sensor Limits and Ranges
#define TPS_MIN_VOLTS 0.5
#define TPS_MAX_VOLTS 4.5
#define MAP_MIN_KPA 0
#define MAP_MAX_KPA 300
#define TEMP_MIN_C -40
#define TEMP_MAX_C 200
#define AFR_MIN 10.0
#define AFR_MAX 20.0

// Safety Limits
#define MAX_CLT_C 120           // Maximum coolant temperature
#define MIN_CLT_C -10           // Minimum coolant temperature for start
#define MAX_IAT_C 80            // Maximum intake air temperature
#define MAX_RPM_LIMIT 7500      // Hard RPM limit
#define MIN_OIL_PRESSURE_KPA 50 // Minimum oil pressure
#define MIN_FUEL_PRESSURE_KPA 200 // Minimum fuel pressure

// Filtering
#define FILTER_ALPHA 0.1        // Low-pass filter coefficient (0.0-1.0)
#define RPM_FILTER_SAMPLES 4

// Fuel System
#define FUEL_ALGORITHM_SPEED_DENSITY 0
#define FUEL_ALGORITHM_ALPHA_N 1
#define FUEL_ALGORITHM_MAF 2

// Ignition Modes
#define IGN_MODE_FIXED 0
#define IGN_MODE_DYNAMIC 1

// Injector Configuration
#define INJECTOR_DEADTIME_US 1000  // Injector dead time in microseconds
#define MIN_INJECTOR_PULSE_US 500  // Minimum injector pulse width

// Ignition Configuration
#define DWELL_TIME_MS 3.0          // Coil dwell time in milliseconds
#define MIN_DWELL_MS 1.0
#define MAX_DWELL_MS 5.0

// Table Sizes
#define RPM_TABLE_SIZE 16
#define LOAD_TABLE_SIZE 16
#define VE_TABLE_SIZE (RPM_TABLE_SIZE * LOAD_TABLE_SIZE)
#define IGN_TABLE_SIZE (RPM_TABLE_SIZE * LOAD_TABLE_SIZE)
#define AFR_TABLE_SIZE (RPM_TABLE_SIZE * LOAD_TABLE_SIZE)

#endif // ECU_CONFIG_H

