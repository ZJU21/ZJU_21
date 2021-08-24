
#include "SunConfig.h"
A4950MotorShield motors;
void setup()
{
  motors.init();
  //motors.init(1, 0);
}

void loop()
{
  for (int i = 0; i < 256; i++)
  {
    motors.setM1Speed(i);
    delay(100);
  }

  motors.setM2Speed(150);
  delay(1000);
}