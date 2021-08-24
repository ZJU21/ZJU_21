/*
 * 舵机角度回读实验
 * 用手掰动舵机, 角度和公路回读并将角度和功率读数通过串口3传回
 * --------------------------

 * 更新时间: 2020/01/13
 **/
#include "SunConfig.h"  // 串口总线舵机通信协议 Fashion Star串口总线舵机 Fashion Star智能夹具

// 串口总线舵机配置
#define SERVO_ID 4              //爪子对应的舵机ID号
#define USERVO_BAUDRATE 115200  // 舵机控制板串口波特率
#define DEBUG_BAUDRATE 19200    //调式串口波特率

// 配置参数
#define DAMPING_POWER 800  // 阻尼模式下的功率(单位mW) 500,800,1000
// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial3, USERVO_BAUDRATE);  //构建对象，串口3自动初始化
// FSUS_Protocol protocol(&Serial3);
// //构建对象，串口3自动初始化,默认波特率115200
FSUS_Servo uservo(SERVO_ID, &protocol);  // 创建舵机

void setup() {
  Serial.begin(DEBUG_BAUDRATE);
  uservo.init();  //舵机角度初始化
  uservo.setDamping(DAMPING_POWER);
  Serial.println("Query Servo Angle");
}

void loop() {
  // 舵机角度查询 (更新角度)
  uservo.queryAngle();

  // 日志输出
  String message =
      "Status Code: " + String(uservo.protocol->responsePack.recv_status, DEC) +
      " servo #" + String(uservo.servoId, DEC) +
      " , Current Angle = " + String(uservo.curAngle, 1) + " deg" +
      " , Current Power = " + String(uservo.queryPower(), 1) + " mW";
  Serial.println(message);
}