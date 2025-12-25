#include "sensors.h"
#include <Arduino.h>
#include <string.h>

// Static instance pointer for interrupt handlers
SensorManager* SensorManager::instance = nullptr;

// Crank sensor interrupt handler
void SensorManager::crankSensorISR() {
  if (SensorManager::instance) {
    uint32_t currentTime = micros();
    uint32_t elapsed = currentTime - SensorManager::instance->lastCrankTime;
    
    // Ignore obviously wrong values (debounce)
    if (elapsed > 100 && elapsed < 100000) {
      SensorManager::instance->crankPeriod = elapsed;
      SensorManager::instance->lastCrankTime = currentTime;
      SensorManager::instance->crankPulseCount++;
    }
  }
}

SensorManager::SensorManager() {
  instance = this;
  filteredTPS = 0.0;
  filteredMAP = 0.0;
  filteredIAT = 0.0;
  filteredCLT = 0.0;
  filteredRPM = 0.0;
  crankPulseCount = 0;
  lastCrankTime = 0;
  crankPeriod = 0;
  
  // Initialize sensor data
  memset(&sensorData, 0, sizeof(SensorData));
}

void SensorManager::initialize() {
  // Configure analog inputs
  pinMode(PIN_TPS_CH1, INPUT);
  pinMode(PIN_TPS_CH2, INPUT);
  pinMode(PIN_MAP, INPUT);
  pinMode(PIN_IAT, INPUT);
  pinMode(PIN_CLT, INPUT);
  pinMode(PIN_FUEL_PRESSURE, INPUT);
  pinMode(PIN_FUEL_LEVEL, INPUT);
  pinMode(PIN_BARO, INPUT);
  pinMode(PIN_WBO2, INPUT);
  pinMode(PIN_FLEX_FUEL, INPUT);
  
  // Configure digital inputs
  pinMode(PIN_BRAKE_PEDAL, INPUT_PULLUP);
  pinMode(PIN_CLUTCH_PEDAL, INPUT_PULLUP);
  pinMode(PIN_VSS, INPUT);
  pinMode(PIN_TURBO_SPEED, INPUT);
  
  // Configure interrupt pins
  pinMode(PIN_CRANK_SENSOR, INPUT_PULLUP);
  pinMode(PIN_CAM_SENSOR, INPUT_PULLUP);
  
  // Attach interrupts for crank and cam sensors
  attachInterrupt(digitalPinToInterrupt(PIN_CRANK_SENSOR), crankSensorISR, RISING);
  
  // Allow ADC to stabilize
  delay(100);
  
  // Read initial barometric pressure
  sensorData.baro = readPressureSensor(PIN_BARO, 0, 150);
}

void SensorManager::update() {
  // Read TPS (use average of two channels for redundancy)
  float tps1 = readAnalogPercentage(PIN_TPS_CH1, TPS_MIN_VOLTS, TPS_MAX_VOLTS);
  float tps2 = readAnalogPercentage(PIN_TPS_CH2, TPS_MIN_VOLTS, TPS_MAX_VOLTS);
  float tpsRaw = (tps1 + tps2) / 2.0;
  applyLowPassFilter(filteredTPS, tpsRaw, FILTER_ALPHA);
  sensorData.tps = filteredTPS;
  
  // Read MAP sensor
  float mapRaw = readPressureSensor(PIN_MAP, MAP_MIN_KPA, MAP_MAX_KPA);
  applyLowPassFilter(filteredMAP, mapRaw, FILTER_ALPHA);
  sensorData.map = filteredMAP;
  
  // Read temperature sensors
  float iatRaw = readTemperatureSensor(PIN_IAT);
  applyLowPassFilter(filteredIAT, iatRaw, FILTER_ALPHA);
  sensorData.iat = filteredIAT;
  
  float cltRaw = readTemperatureSensor(PIN_CLT);
  applyLowPassFilter(filteredCLT, cltRaw, FILTER_ALPHA);
  sensorData.clt = filteredCLT;
  
  // Read barometric pressure (less frequent updates)
  static uint32_t lastBaroRead = 0;
  if (millis() - lastBaroRead > 1000) {
    sensorData.baro = readPressureSensor(PIN_BARO, 0, 150);
    lastBaroRead = millis();
  }
  
  // Read fuel sensors
  sensorData.fuelPressure = readPressureSensor(PIN_FUEL_PRESSURE, 0, 600);
  sensorData.fuelLevel = readAnalogPercentage(PIN_FUEL_LEVEL, 0.5, 4.5);
  
  // Read wideband O2 sensor
  sensorData.afr = readWidebandO2(PIN_WBO2);
  
  // Read flex fuel sensor
  sensorData.ethanolPercent = readFlexFuelSensor(PIN_FLEX_FUEL);
  
  // Read digital inputs
  sensorData.brakePedal = !digitalRead(PIN_BRAKE_PEDAL); // Active low with pullup
  sensorData.clutchPedal = !digitalRead(PIN_CLUTCH_PEDAL);
  
  // Calculate RPM from crank sensor
  float rpmRaw = calculateRPM();
  applyLowPassFilter(filteredRPM, rpmRaw, FILTER_ALPHA);
  sensorData.rpm = filteredRPM;
  
  // VSS calculation (pulse counting - simplified)
  // Would need interrupt handler for accurate VSS reading
  // sensorData.vss = calculateVSS();
}

float SensorManager::readAnalogVoltage(uint8_t pin) {
  int raw = analogRead(pin);
  return (raw / 1024.0) * 5.0; // Assuming 5V reference
}

float SensorManager::readAnalogPercentage(uint8_t pin, float minVolts, float maxVolts) {
  float voltage = readAnalogVoltage(pin);
  float percentage = ((voltage - minVolts) / (maxVolts - minVolts)) * 100.0;
  return constrain(percentage, 0.0, 100.0);
}

float SensorManager::readTemperatureSensor(uint8_t pin) {
  // Assuming 10k NTC thermistor with 10k pullup resistor
  // This is a simplified calculation - adjust based on actual sensor
  float voltage = readAnalogVoltage(pin);
  float resistance = (5.0 - voltage) * 10000.0 / voltage; // 10k pullup
  
  // Steinhart-Hart equation approximation for NTC thermistor
  // Constants for typical 10k NTC (adjust based on actual sensor)
  float A = 0.001129148;
  float B = 0.000234125;
  float C = 0.0000000876741;
  
  float logR = log(resistance);
  float temperature = 1.0 / (A + B * logR + C * logR * logR * logR);
  temperature = temperature - 273.15; // Convert Kelvin to Celsius
  
  return constrain(temperature, TEMP_MIN_C, TEMP_MAX_C);
}

float SensorManager::readPressureSensor(uint8_t pin, float minKPa, float maxKPa) {
  // Assuming 0-5V linear pressure sensor
  float voltage = readAnalogVoltage(pin);
  float pressure = minKPa + ((voltage / 5.0) * (maxKPa - minKPa));
  return constrain(pressure, minKPa, maxKPa);
}

float SensorManager::readWidebandO2(uint8_t pin) {
  // Wideband O2 sensors typically output 0-5V representing AFR 10-20
  // Adjust based on your specific sensor (e.g., Innovate LC-2, AEM UEGO, etc.)
  float voltage = readAnalogVoltage(pin);
  float afr = 10.0 + ((voltage / 5.0) * 10.0); // 10-20 AFR range
  return constrain(afr, AFR_MIN, AFR_MAX);
}

float SensorManager::readFlexFuelSensor(uint8_t pin) {
  // Flex fuel sensor typically outputs PWM or analog signal
  // PWM: 0-100% ethanol (0-5V = 0-100%)
  // This is a simplified implementation - adjust for your sensor
  float voltage = readAnalogVoltage(pin);
  float ethanolPercent = (voltage / 5.0) * 100.0;
  return constrain(ethanolPercent, 0.0, 100.0);
}

float SensorManager::calculateRPM() {
  // Calculate RPM from crank sensor period
  // Assuming 60 teeth on crank wheel with 2 missing teeth
  // Each pulse = 6 degrees (360 / 60)
  
  if (crankPeriod == 0 || crankPeriod > 100000) {
    return 0.0;
  }
  
  // Period is time for one tooth (degrees)
  // RPM = (60 seconds/minute) * (1,000,000 microseconds/second) / (period microseconds/tooth * teeth/revolution)
  // For 60-tooth wheel: 60 teeth per revolution
  float periodPerRevolution = crankPeriod * 60.0; // microseconds per revolution
  float rpm = (60.0 * 1000000.0) / periodPerRevolution;
  
  return constrain(rpm, 0.0, MAX_RPM);
}

void SensorManager::applyLowPassFilter(float &filtered, float newValue, float alpha) {
  filtered = (alpha * newValue) + ((1.0 - alpha) * filtered);
}

bool SensorManager::isSensorHealthy() const {
  // Basic sensor health checks
  if (sensorData.tps < 0 || sensorData.tps > 100) return false;
  if (sensorData.map < MAP_MIN_KPA || sensorData.map > MAP_MAX_KPA) return false;
  if (sensorData.iat < TEMP_MIN_C || sensorData.iat > TEMP_MAX_C) return false;
  if (sensorData.clt < TEMP_MIN_C || sensorData.clt > TEMP_MAX_C) return false;
  return true;
}

