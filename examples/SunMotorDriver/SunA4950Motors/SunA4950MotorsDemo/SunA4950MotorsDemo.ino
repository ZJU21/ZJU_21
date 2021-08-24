/*
 * This example uses the SunA4950Motors library to drive each motor with the
 *  A4950 Dual Motor Driver Shield for Arduino forward, then backward. 
 * The yellow user LED is on when a motor is set to a positive speed and off when
 * a motor is set to a negative speed.
 */

#include "SunConfig.h"

A4950MotorShield motors;

void setup()
{
  motors.init();
  Serial.begin(BAUDRATE);
  Serial.println("A4950 Quad Motor Driver Shield for Arduino");
}

void loop()
{
  // run M1 motor with positive speed

  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  // run M1 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  // run M2 motor with positive speed



  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  // run M2 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  // run M3 motor with positive speed



  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  // run M3 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  // run M4 motor with positive speed


  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  // run M4 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  delay(500);
}