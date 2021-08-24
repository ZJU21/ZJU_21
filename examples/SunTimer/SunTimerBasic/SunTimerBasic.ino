
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * 使用SunEncoder库检测4路电机速度.
 */

#include <FlexiTimer2.h>  //定时中断

#include "SunConfig.h"       //包含配置库


void control() {
  static int print_Count;
  sei();                     //全局中断开启
  Serial.println(millis());  //显示
  //  if (++print_Count >= 10) //打印控制，控制周期100ms
  //  {
  // // Serial.println(millis());//显示
  //  Serial.print(Velocity_A);//显示
  //  Serial.print(",");
  // Serial.print(Velocity_B);//显示
  // Serial.print(",");
  //  Serial.print(Velocity_C);//显示
  //  Serial.print(",");
  //  Serial.println(Velocity_D);//显示
  //  //Serial.println(iConstrain);
  //    print_Count = 0;
  //  }
}

void setup() {
  FlexiTimer2::set(10, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();           //中断使能
  delay(100);                     //延时等待初始化完成
  Serial.begin(BAUDRATE);
  Serial.println("Sunnybot Timer  Test:");
}

void loop() {
  delay(2000);

  Serial.print("loop");
  Serial.println(millis());  //显示
}
