#pragma once
#include <stdint.h>

// In a header so Arduino's automatic function prototypes can see this type.
struct SectorResult {
  uint16_t median;
  uint16_t minimum;
  uint16_t maximum;
  uint8_t hits;
  uint8_t cells;
  uint8_t meanQuality;
};
