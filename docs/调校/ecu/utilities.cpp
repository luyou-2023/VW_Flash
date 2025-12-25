#include "utilities.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Linear interpolation
float interpolate(float x, float x1, float x2, float y1, float y2) {
  if (x2 == x1) return y1;
  return y1 + ((x - x1) * (y2 - y1) / (x2 - x1));
}

// Constrain float value
float constrainFloat(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// Map float value from one range to another
float mapFloat(float value, float fromMin, float fromMax, float toMin, float toMax) {
  float fromRange = fromMax - fromMin;
  if (fromRange == 0) return toMin;
  
  float toRange = toMax - toMin;
  float normalized = (value - fromMin) / fromRange;
  return toMin + (normalized * toRange);
}

// Low-pass filter implementation
LowPassFilter::LowPassFilter(float alpha) {
  this->alpha = constrainFloat(alpha, 0.0, 1.0);
  filteredValue = 0.0;
}

float LowPassFilter::update(float input) {
  filteredValue = (alpha * input) + ((1.0 - alpha) * filteredValue);
  return filteredValue;
}

void LowPassFilter::reset(float initialValue) {
  filteredValue = initialValue;
}

// Moving average filter implementation
MovingAverage::MovingAverage(uint8_t size) {
  this->size = size > 0 ? size : 1;
  buffer = (float*)malloc(this->size * sizeof(float));
  index = 0;
  filled = false;
  reset();
}

MovingAverage::~MovingAverage() {
  free(buffer);
}

float MovingAverage::update(float input) {
  buffer[index] = input;
  index = (index + 1) % size;
  
  if (!filled && index == 0) {
    filled = true;
  }
  
  float sum = 0.0;
  uint8_t count = filled ? size : index;
  for (uint8_t i = 0; i < count; i++) {
    sum += buffer[i];
  }
  
  return sum / count;
}

void MovingAverage::reset() {
  memset(buffer, 0, size * sizeof(float));
  index = 0;
  filled = false;
}

// Convert microseconds to crankshaft degrees
uint32_t microsecondsToCrankDegrees(uint32_t microseconds, float rpm) {
  if (rpm == 0) return 0;
  float degreesPerSecond = (rpm / 60.0) * 360.0;
  float degreesPerMicrosecond = degreesPerSecond / 1000000.0;
  return (uint32_t)(microseconds * degreesPerMicrosecond);
}

// Convert crankshaft degrees to microseconds
uint32_t crankDegreesToMicroseconds(float degrees, float rpm) {
  if (rpm == 0) return 0;
  float degreesPerSecond = (rpm / 60.0) * 360.0;
  float microsecondsPerDegree = 1000000.0 / degreesPerSecond;
  return (uint32_t)(degrees * microsecondsPerDegree);
}

// 1D table lookup with linear interpolation
float tableLookup1D(const float* table, const float* bins, uint8_t size, float value) {
  if (size == 0) return 0.0;
  if (size == 1) return table[0];
  
  // Clamp value to table range
  if (value <= bins[0]) return table[0];
  if (value >= bins[size - 1]) return table[size - 1];
  
  // Find bin
  for (uint8_t i = 0; i < size - 1; i++) {
    if (value >= bins[i] && value <= bins[i + 1]) {
      return interpolate(value, bins[i], bins[i + 1], table[i], table[i + 1]);
    }
  }
  
  return table[size - 1];
}

// 2D table lookup with bilinear interpolation
float tableLookup2D(const float* table, const float* xBins, const float* yBins, 
                    uint8_t xSize, uint8_t ySize, float xValue, float yValue) {
  if (xSize == 0 || ySize == 0) return 0.0;
  
  // Clamp values
  if (xValue <= xBins[0]) xValue = xBins[0];
  if (xValue >= xBins[xSize - 1]) xValue = xBins[xSize - 1];
  if (yValue <= yBins[0]) yValue = yBins[0];
  if (yValue >= yBins[ySize - 1]) yValue = yBins[ySize - 1];
  
  // Find X bin
  uint8_t xIdx = 0;
  for (uint8_t i = 0; i < xSize - 1; i++) {
    if (xValue >= xBins[i] && xValue <= xBins[i + 1]) {
      xIdx = i;
      break;
    }
  }
  
  // Find Y bin
  uint8_t yIdx = 0;
  for (uint8_t i = 0; i < ySize - 1; i++) {
    if (yValue >= yBins[i] && yValue <= yBins[i + 1]) {
      yIdx = i;
      break;
    }
  }
  
  // Get four corner values
  float z11 = table[xIdx * ySize + yIdx];
  float z12 = table[xIdx * ySize + (yIdx + 1)];
  float z21 = table[(xIdx + 1) * ySize + yIdx];
  float z22 = table[(xIdx + 1) * ySize + (yIdx + 1)];
  
  // Bilinear interpolation
  float xRatio = (xValue - xBins[xIdx]) / (xBins[xIdx + 1] - xBins[xIdx]);
  float yRatio = (yValue - yBins[yIdx]) / (yBins[yIdx + 1] - yBins[yIdx]);
  
  float z1 = z11 + (z12 - z11) * yRatio;
  float z2 = z21 + (z22 - z21) * yRatio;
  float z = z1 + (z2 - z1) * xRatio;
  
  return z;
}

