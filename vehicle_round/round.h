#ifndef _ROUND_H
#define _ROUND_H

#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 调试串口的配置
//#define DEBUG_SERIAL Serial
//#define DEBUG_SERIAL_BAUDRATE 115200

#define BAUDRATE 115200 // 波特率
#define SERVO1_ID 5 //舵机ID号
#define SERVO2_ID 6 //舵机ID号

FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo1(SERVO1_ID, &protocol); // 创建舵机
FSUS_Servo uservo2(SERVO2_ID, &protocol); // 创建舵机

class Round
{
  private:
  void test();
  public:
  void init();
  void run();
  void stop();
};

void Round::run()
{
  delay(3000);            // 等待2s
  uservo1.wheelRun(FSUS_CCW);
  uservo2.wheelRun(FSUS_CCW);
  delay(2000);            // 等待2s
}

void Round::stop()
{
  delay(3000);            // 等待2s
  uservo1.wheelStop();
  uservo2.wheelStop();
  delay(2000);            // 等待2s
}

void Round::test()
{
  run();
  stop();
}

void Round::init()
{
    protocol.init();        // 通信协议初始化
    uservo1.init();          //舵机角度初始化
    uservo2.init();          //舵机角度初始化
    uservo1.setSpeed(300);    // 设置转速为20°/s
    uservo2.setSpeed(300);    // 设置转速为20°/s
    test();
}

#endif
