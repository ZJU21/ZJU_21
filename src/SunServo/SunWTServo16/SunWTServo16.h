#ifndef SUN_WTSERVO16_H
#define SUN_WTSERVO16_H

#include <Arduino.h>
#include "SunConfig.h"
//发送部分的指令
#define FRAME_HEADER 0xFF          //帧头
#define CMD_SERVO_SPEED 0x01       //设置舵机速度
#define CMD_SERVO_POSITION 0x02    //设置舵机位置
#define CMD_ACTION_GROUP_RUN 0x09  //运行动作组
#define CMD_STOP_REFRESH 0x0b      //急停、恢复指令

class SunWTServo16 {
 public:
  SunWTServo16(HardwareSerial& uartPort);  //构造函数 串口
  SunWTServo16();  //默认构造函数

  void moveServo(uint8_t servoID, uint16_t Position, uint16_t Speed);//Position 输入的是角度  中间换算为脉冲宽度 按180度舵机转换
  void moveServos(uint8_t Num, uint16_t Speed, ...);
  void runActionGroup(uint8_t NumOfAction);
  void stopServo(void);
  void refreshServo(void);

 public:
  bool _isRunning;  //正在运行
  HardwareSerial* _uartPort;

  };
#endif