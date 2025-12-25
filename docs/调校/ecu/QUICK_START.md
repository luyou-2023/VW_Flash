# Quick Start Guide - FOME ECU Arduino Implementation

## Getting Started in 5 Minutes

### 1. Hardware Setup Checklist
- [ ] Arduino Mega 2560 (or Due/Teensy)
- [ ] USB cable for programming
- [ ] Sensors connected according to pin definitions in `ecu_config.h`
- [ ] Power supply (12V for vehicle use, 5V for bench testing)

### 2. Software Setup
1. Open `ecu_main.ino` in Arduino IDE
2. Select your board: **Tools → Board → Arduino Mega 2560**
3. Select port: **Tools → Port → [Your COM port]**
4. Click **Upload**

### 3. Verify Operation
1. Open **Serial Monitor** (115200 baud)
2. You should see: `"FOME ECU Initializing..."`
3. Followed by: `"FOME ECU Ready"`
4. Debug output will appear every 100ms

### 4. Basic Configuration

#### Change Number of Cylinders
Edit `ecu_config.h`:
```cpp
#define NUM_CYLINDERS 4  // Change to your engine
```

#### Select Fuel Algorithm
Edit `ecu_main.ino` → `setup()`:
```cpp
fuelConfig.algorithm = FUEL_ALGORITHM_SPEED_DENSITY;  // or ALPHA_N, MAF
```

#### Adjust Safety Limits
Edit `ecu_config.h`:
```cpp
#define MAX_CLT_C 120      // Max coolant temp
#define MAX_RPM_LIMIT 7500 // Max RPM
```

### 5. First Test
- Connect sensors (at minimum: TPS, MAP, IAT, CLT, Crank sensor)
- Monitor Serial output for sensor readings
- Verify no "SAFE MODE" messages appear
- Check that RPM is calculated correctly from crank sensor

### 6. Troubleshooting

**Problem**: No sensor readings
- **Solution**: Check pin definitions match your wiring
- **Solution**: Verify sensor power and ground connections
- **Solution**: Check Serial Monitor baud rate (115200)

**Problem**: "SAFE MODE ACTIVATED"
- **Solution**: Check which sensor fault triggered (see Serial output)
- **Solution**: Verify sensor is connected and working
- **Solution**: Check sensor calibration values

**Problem**: RPM reads 0
- **Solution**: Verify crank sensor is connected to interrupt pin (PIN_CRANK_SENSOR)
- **Solution**: Check crank sensor signal (should be 0-5V square wave)
- **Solution**: Verify CRANK_TEETH definition matches your trigger wheel

**Problem**: Compilation errors
- **Solution**: Ensure all `.h` and `.cpp` files are in the same directory as `.ino`
- **Solution**: Verify Arduino IDE version (1.8.x or 2.x)
- **Solution**: Check for missing includes

### 7. Next Steps
1. **Tune VE Table**: Adjust fuel delivery in `fuel_system.cpp`
2. **Tune Ignition Table**: Adjust timing advance in `ignition_system.cpp`
3. **Calibrate Sensors**: Adjust temperature sensor coefficients
4. **Implement Sequential Injection**: Add timer-based injection timing
5. **Add Hardware Timers**: Improve ignition timing precision

### 8. Important Notes
- ⚠️ This is a development implementation - test thoroughly before vehicle use
- ⚠️ Current timing uses `delayMicroseconds()` - not suitable for high RPM
- ⚠️ Sequential injection requires proper cam sensor implementation
- ✅ Safety system will prevent engine damage if sensors fail
- ✅ All sensor readings are filtered for noise reduction

### Support Files
- **README.md**: Full documentation
- **ecu_config.h**: All configuration constants
- **ecu_types.h**: Data structures
- **Serial Monitor**: Real-time debugging and telemetry

---

For detailed information, see **README.md**

