#ifndef SENSORS_H
#define SENSORS_H

#include "ecu_config.h"
#include "ecu_types.h"

class SensorManager {
public:
  SensorManager();
  void initialize();
  void update();
  SensorData getData() const { return sensorData; }
  bool isSensorHealthy() const;

private:
  SensorData sensorData;
  
  // Filtered values
  float filteredTPS;
  float filteredMAP;
  float filteredIAT;
  float filteredCLT;
  float filteredRPM;
  
  // RPM calculation
  volatile uint32_t crankPulseCount;
  volatile uint32_t lastCrankTime;
  volatile uint32_t crankPeriod;
  
  // Helper functions
  float readAnalogVoltage(uint8_t pin);
  float readAnalogPercentage(uint8_t pin, float minVolts, float maxVolts);
  float readTemperatureSensor(uint8_t pin); // Thermistor or analog temp sensor
  float readPressureSensor(uint8_t pin, float minKPa, float maxKPa);
  float readWidebandO2(uint8_t pin);
  float readFlexFuelSensor(uint8_t pin);
  float calculateRPM();
  void applyLowPassFilter(float &filtered, float newValue, float alpha);
  
  // Interrupt handlers (must be static or friend functions)
  static void crankSensorISR();
  static SensorManager* instance;
};

#endif // SENSORS_H

