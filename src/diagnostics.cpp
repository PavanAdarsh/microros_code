#include "diagnostics.hpp"

//Servo LUMEN;
int led_status[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

void initializeLED()
{
  for (int pin_index = 0; pin_index < 9; pin_index++)
  {
    pinMode(led_pinmap[pin_index], OUTPUT);
    digitalWrite(led_pinmap[pin_index], LOW);

    //LUMEN.attach(PA0);
  }
}

void setLED(int16_t led_indicator)
{
  for (int pin_index = 0; pin_index < 9; pin_index++)
  {
    digitalWrite(led_pinmap[pin_index], HIGH);
  }

  // for (int j = 0; j < 20; j++)
  // {
  //   if (led_indicator == led_bitmask[j])
  //   {
  for (int pin_index = 0; pin_index < 9; pin_index++)
  {
    if (led_indicator & (1 << pin_index))
    {
      digitalWrite(led_pinmap[pin_index], LOW);
    }
  }
  return;
  //   }
  // }
}

// void readVoltage()
// {
//   int value = analogRead(PA4);
//   Serial.println(value);
//   float voltage = map (value, 0, 4096, 0, 3.3);
//   Serial.println(voltage);
//   delay(100);
// }

