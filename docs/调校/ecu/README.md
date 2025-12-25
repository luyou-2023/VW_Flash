# FOME ECU Implementation for Arduino

A comprehensive Engine Control Unit (ECU) implementation for Arduino boards, designed to replicate key features of FOME ECU systems. This implementation includes sensor processing, fuel injection algorithms, ignition control, and safety/protection logic.

## Hardware Requirements

### Supported Boards
- **Arduino Mega 2560** (Recommended for pin count)
- **Arduino Due** (32-bit, native CAN support)
- **Teensy 3.x/4.x** (32-bit, high performance)

### Required Components
- Arduino-compatible microcontroller
- Analog sensors (TPS, MAP, IAT, CLT, pressure sensors)
- Digital sensors (Crank, Cam, VSS)
- Wideband O2 sensor (optional for closed-loop control)
- Fuel injectors with drivers
- Ignition coils with drivers
- CAN bus module (MCP2515, optional)
- Various resistors and signal conditioning circuits

## Features

### 1. Sensor Processing
- **Throttle Position Sensor (TPS)**: Dual redundant channels with low-pass filtering
- **Manifold Absolute Pressure (MAP)**: Pressure sensor reading
- **Intake Air Temperature (IAT)**: Thermistor-based temperature sensing
- **Coolant Temperature (CLT)**: Engine temperature monitoring
- **Barometric Pressure**: Altitude compensation
- **Wideband O2 Sensor**: Air-fuel ratio measurement
- **Flex Fuel Sensor**: Ethanol content detection
- **Crank/Cam Sensors**: RPM calculation and engine position
- **Vehicle Speed Sensor (VSS)**: Speed measurement
- **Brake/Clutch Pedal Sensors**: Safety and control inputs

### 2. Fuel Injection Algorithms
- **Speed Density**: Fuel calculation based on MAP, IAT, RPM, and VE table
- **Alpha-N**: Fuel control based on throttle position and RPM
- **MAF (Mass Air Flow)**: Fuel control based on airflow sensor input

### 3. Fuel System Features
- Closed-loop AFR control with PID
- Coolant temperature fuel correction
- Intake air temperature correction
- TPS-based corrections (acceleration enrichment)
- Flex fuel support (ethanol percentage compensation)
- Per-cylinder fuel trim
- Deceleration fuel cut-off
- Injector deadtime compensation
- Small pulse width correction

### 4. Ignition Control
- Dynamic timing advance tables (RPM vs Load)
- Fixed timing mode
- Coolant temperature correction
- Intake air temperature adder
- Per-cylinder ignition trim
- Configurable dwell time
- Multiple firing orders supported

### 5. Safety and Protection
- Sensor fault detection (TPS, MAP, IAT, CLT)
- Crank/Cam signal validation
- Wideband O2 sensor fault detection
- Fuel pressure monitoring
- Oil pressure monitoring
- RPM limit protection
- Overvoltage protection
- Overtemperature protection
- Safe mode activation on faults

## Project Structure

```
ecu/
├── ecu_main.ino          # Main ECU control loop
├── ecu_config.h          # Hardware configuration and pin definitions
├── ecu_types.h           # Data structures and types
├── sensors.h/cpp         # Sensor reading and processing
├── fuel_system.h/cpp     # Fuel injection algorithms and control
├── ignition_system.h/cpp # Ignition timing and coil control
├── safety_system.h/cpp   # Safety and fault detection
├── utilities.h/cpp       # Helper functions (filtering, math, etc.)
└── README.md             # This file
```

## Configuration

### Pin Definitions
Edit `ecu_config.h` to match your hardware wiring:

```cpp
#define PIN_TPS_CH1 A0      // Throttle Position Sensor Channel 1
#define PIN_MAP A2          // Manifold Absolute Pressure
#define PIN_IAT A3          // Intake Air Temperature
#define PIN_CLT A4          // Coolant Temperature
#define PIN_CRANK_SENSOR 2  // Crankshaft sensor (interrupt-capable)
// ... etc
```

### Engine Configuration
Modify these constants in `ecu_config.h`:

```cpp
#define NUM_CYLINDERS 4         // Number of cylinders
#define MAX_RPM 8000            // Maximum RPM
#define CRANK_TEETH 60          // Crank trigger wheel teeth
#define MISSING_TEETH 2         // Missing teeth pattern
```

### Fuel Algorithm Selection
In `ecu_main.ino` setup():

```cpp
fuelConfig.algorithm = FUEL_ALGORITHM_SPEED_DENSITY;  // or ALPHA_N, MAF
```

## Installation

1. **Install Arduino IDE** (1.8.x or 2.x) or PlatformIO

2. **Clone/Download** this repository to your Arduino sketchbook or workspace

3. **Open** `ecu_main.ino` in Arduino IDE

4. **Select Board**: Tools → Board → Arduino Mega 2560 (or your board)

5. **Select Port**: Tools → Port → [Your Arduino port]

6. **Upload** the sketch

## Tuning

### VE Table (Volumetric Efficiency)
The VE table maps RPM and Load (MAP or TPS) to volumetric efficiency percentage. Edit the table initialization in `fuel_system.cpp`:

```cpp
for (int i = 0; i < VE_TABLE_SIZE; i++) {
  veTable.veValues[i] = 80.0; // Tune these values for your engine
}
```

### Ignition Timing Table
Edit the ignition advance table in `ignition_system.cpp`:

```cpp
for (int i = 0; i < IGN_TABLE_SIZE; i++) {
  ignTable.timingValues[i] = 15.0; // Degrees BTDC - tune for your engine
}
```

### Closed-Loop AFR Control
Adjust PID gains in `fuel_system.cpp`:

```cpp
float Kp = 0.1;  // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.05; // Derivative gain
```

### Safety Limits
Modify safety thresholds in `ecu_config.h`:

```cpp
#define MAX_CLT_C 120           // Maximum coolant temperature
#define MAX_RPM_LIMIT 7500      // Hard RPM limit
#define MIN_FUEL_PRESSURE_KPA 200 // Minimum fuel pressure
```

## Usage

### Basic Operation
1. Power on the Arduino
2. Connect all sensors according to pin definitions
3. Open Serial Monitor at 115200 baud
4. Monitor sensor readings and ECU calculations
5. Engine will enter safe mode if any critical fault is detected

### Debug Output
The ECU outputs debug information every 100ms via Serial:

```
RPM: 1250 | TPS: 25.5% | MAP: 85.2 kPa | IAT: 22.3C | CLT: 87.5C | AFR: 14.65 | Fuel PW: 3.42 ms | Timing: 18.5 deg
```

### Safe Mode
When safe mode is activated, the ECU will:
- Cut fuel injection
- Disable ignition
- Log fault conditions via Serial
- Prevent engine operation until faults are cleared

## Advanced Features

### Sequential Fuel Injection
For sequential injection, implement proper timing based on crank/cam position. Currently, the framework supports this but requires timer-based implementation for precise injection timing.

### Hardware Timer-Based Ignition
For precise ignition timing, implement hardware timer interrupts. See `ignition_system.cpp` → `scheduleIgnition()` for placeholder implementation.

### CAN Bus Communication
To enable CAN bus communication (for gauges, datalogging, etc.):
1. Install CAN library (e.g., `mcp_can` for MCP2515)
2. Uncomment CAN-related code
3. Configure CAN module pins

### EEPROM/SD Card Tuning Storage
Store tuning tables in EEPROM or SD card for runtime modification:
- Use Arduino EEPROM library for small tables
- Use SD card library for larger datasets
- Implement table read/write functions

## Sensor Calibration

### Temperature Sensors
Most temperature sensors use NTC thermistors. Adjust the Steinhart-Hart coefficients in `sensors.cpp` → `readTemperatureSensor()` based on your specific sensor:

```cpp
float A = 0.001129148;  // Adjust for your thermistor
float B = 0.000234125;
float C = 0.0000000876741;
```

### Pressure Sensors
Calibrate pressure sensor ranges in `ecu_config.h`:

```cpp
#define MAP_MIN_KPA 0
#define MAP_MAX_KPA 300  // Adjust for your MAP sensor range
```

### Wideband O2 Sensor
Adjust AFR calculation in `sensors.cpp` → `readWidebandO2()` based on your sensor's voltage-to-AFR mapping (e.g., Innovate LC-2, AEM UEGO, etc.)

## Limitations and Notes

1. **Timing Precision**: Current implementation uses `delayMicroseconds()` for timing. For production use, implement hardware timer-based timing for injectors and ignition.

2. **Interrupt Latency**: Ensure interrupt handlers are fast and non-blocking. Consider using ISR-safe flags and processing in main loop.

3. **Sensor Filtering**: Adjust filter coefficients (`FILTER_ALPHA`) based on your application's noise characteristics.

4. **Table Size**: Current tables are 16x16. Adjust `RPM_TABLE_SIZE` and `LOAD_TABLE_SIZE` if needed, but consider memory limitations.

5. **Sequential Injection**: Full sequential injection requires cam sensor position tracking and precise timing control.

6. **Hardware Drivers**: This code assumes injectors and coils are controlled via digital outputs. You may need external drivers (e.g., injector drivers, coil drivers) depending on your hardware.

## Safety Warning

⚠️ **WARNING**: This is a development ECU implementation. Always test thoroughly on a test bench before using on an actual vehicle. Engine damage or personal injury may result from improper tuning or hardware failures. Use at your own risk.

## Contributing

This is a reference implementation. Feel free to modify and extend for your specific needs:
- Add support for additional sensors
- Implement advanced timing strategies
- Add datalogging capabilities
- Integrate with tuning software
- Optimize for specific hardware platforms

## License

This code is provided as-is for educational and development purposes. Modify and use as needed for your project.

## References

- FOME ECU Documentation
- Arduino Documentation: https://www.arduino.cc/reference/
- Engine Management Systems: Speed Density vs Alpha-N
- Wideband O2 Sensor Integration Guides
- Ignition Timing Fundamentals

## Support

For issues and questions:
1. Check sensor wiring and pin definitions
2. Verify sensor calibration values
3. Monitor Serial output for fault messages
4. Review safety system status

---

**Version**: 1.0  
**Last Updated**: 2024  
**Compatible Arduino IDE**: 1.8.x or 2.x

