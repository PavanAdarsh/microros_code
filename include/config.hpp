#ifndef CONFIG_HPP
#define CONFIG_HPP
#define NUMBER_OF_THRUSTERS 6
#define INIT_THRUSTER_PWM \
  { 1500, 1500 ,1500 ,1500 ,1500 ,1500}
#define THRUSTER_PINS \
{ 1,2,3,4,5,6}
#define THRUSTER_MIN_PWM 1200 - (1500 - 1472)
#define THRUSTER_MAX_PWM 1800 - (1500 - 1472)
#define NORMAL_MODE 0
#define CALIBRATION_MODE 0
#define CALIBRATION_UPDATE_MODE 2
#define G 9.80665
const uint8_t v_sense = 27;// const uint8_t m2 = PA5;
// const uint8_t mpwm = PA7;
#endif  // CONFIG_HPP