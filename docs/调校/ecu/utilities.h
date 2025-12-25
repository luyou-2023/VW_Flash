#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>

// Utility functions for ECU operations

// Math utilities
float interpolate(float x, float x1, float x2, float y1, float y2);
float constrainFloat(float value, float min, float max);
float mapFloat(float value, float fromMin, float fromMax, float toMin, float toMax);

// Filter utilities
class LowPassFilter {
public:
  LowPassFilter(float alpha = 0.1);
  float update(float input);
  void reset(float initialValue = 0.0);
  
private:
  float alpha;
  float filteredValue;
};

class MovingAverage {
public:
  MovingAverage(uint8_t size = 4);
  ~MovingAverage();
  float update(float input);
  void reset();
  
private:
  float* buffer;
  uint8_t size;
  uint8_t index;
  bool filled;
};

// Timing utilities
uint32_t microsecondsToCrankDegrees(uint32_t microseconds, float rpm);
uint32_t crankDegreesToMicroseconds(float degrees, float rpm);

// Table lookup utilities
float tableLookup1D(const float* table, const float* bins, uint8_t size, float value);
float tableLookup2D(const float* table, const float* xBins, const float* yBins, 
                    uint8_t xSize, uint8_t ySize, float xValue, float yValue);

#endif // UTILITIES_H

