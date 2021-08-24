/**
 * @file A4950MotorShieldTest.ino
 * @author igcxl (igcxl@qq.com)
 * @brief A4950MotorShield库测试程序
 * @version 0.5
 * @date 2020-12-11
 *
 * @copyright Copyright © igcxl.com 2020 All right reserved.
 *
 */

#include "SunConfig.h"
/*
 * This example uses the A4950MotorShield library to drive each motor with the
 *  A4950 Dual Motor Driver Shield for Arduino forward, then backward.
 * The yellow user LED is on when a motor is set to a positive speed and off
 * when a motor is set to a negative speed.
 */

#define LED_STATE_PIN 22

A4950MotorShield motors;
int directionCorrection[4]={1,1,1,1};

void setup() {
  motors.init();  //电机初始化，初始化引脚模式和pwm频率和电机死区
  // motors.init(1,0);
  // //电机初始化，初始化引脚模式和pwm频率和电机死区,1高频，0默认，死区pwm
  // motors.init(0,30);
  // //电机初始化，初始化引脚模式和pwm频率和电机死区,1高频，0默认，死区pwm
  pinMode(LED_STATE_PIN, OUTPUT);
  Serial.begin(BAUDRATE);
  Serial.println("A4950 Quad Motor Driver Shield for Arduino");
}

void loop() {
  // run M1-M4 motor with positive speed

  digitalWrite(LED_STATE_PIN, HIGH);
  for (int i = 0; i < 256; i++) {
    motors.setSpeeds(100, 0, 0, 0);
    delay(2000);
    motors.setSpeeds(-100, 0, 0, 0);
    delay(1000);
    motors.setSpeeds(0, 0, 0, 0);
    delay(1000);

    motors.setSpeeds(0, 100, 0, 0);
    delay(2000);
    motors.setSpeeds(0, -100, 0, 0);
    delay(2000);
    motors.setSpeeds(0, 0, 0, 0);
    delay(1000);

    motors.setSpeeds(0, 0, 100, 0);
    delay(2000);
    motors.setSpeeds(0, 0, -100, 0);
    delay(2000);
    motors.setSpeeds(0, 0, 0, 0);
    delay(1000);

    motors.setSpeeds(0, 0, 0, 100);
    delay(2000);
    motors.setSpeeds(0, 0, 0, -100);
    delay(2000);
    motors.setSpeeds(0, 0, 0, 0);
    delay(1000);
    Serial.println(i);
  }
}