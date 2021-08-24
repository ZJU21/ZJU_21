#include "SunConfig.h"

SunWTServo16 servo(Serial3);

void setup() {

}
void loop() {
  servo.runActionGroup(1);  //运行1号动作组
  delay(5000);
  servo.moveServo(0,90,10); //0号舵机以10的速度移动至1500位置
  delay(2000);
  servo.moveServo(0,180,10); //0号舵机以10的速度移动至1500位置
  delay(2000);
  servo.moveServos(5,10,0,90,2,70,4,60,6,90,8,79); 
  //控制5个舵机，移动速度10，0号舵机至90位置，2号舵机至70位置，4号舵机至60位置，
  //6号舵机至90位置，8号舵机至79位置
  delay(2000);
}
