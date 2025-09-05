#include "sensor_interface.hpp"
// const int16_t led_bitmask[20] = {1, 2, 4, 8, 16, 32, 64, 73, 81, 84, 128, 138, 140, 146, 256, 266, 268, 273, 276, 292};

const uint8_t led_pinmap[9] = {1,2,3,4,5,6,7,8,9}; // B1 G1 R1 B2 G2 R2 B3 G3 R3

void initializeLED();
void setLED(int16_t led_indicator);
//void readVoltage();
//void checkBattery();
//void setLumen(int16_t value);

