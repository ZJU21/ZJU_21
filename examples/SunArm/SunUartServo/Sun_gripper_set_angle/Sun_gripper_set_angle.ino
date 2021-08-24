/**
 * 爪子角度控制指令测试(带功率约束)
 * --------------------------

 * 更新时间: 2020/11/14
 */

#include "SunConfig.h"  // 串口总线舵机通信协议 Fashion Star串口总线舵机 Fashion Star智能夹具

// 串口总线舵机配置
#define SERVO_ID 0              //爪子对应的舵机ID号
#define USERVO_BAUDRATE 115200  // 舵机控制板串口波特率
#define DEBUG_BAUDRATE 19200    //调式串口波特率

// 爪子的配置
#define SERVO_ANGLE_GRIPPER_OPEN 65.0   // 爪子张开时的角度
#define SERVO_ANGLE_GRIPPER_CLOSE -5.0  // 爪子闭合时的角度
#define GRIPPER_INTERVAL_MS 1000        // 爪子开启闭合的周期, ms
#define GRIPPER_MAX_POWER 200           // 爪子的最大功率

// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial3, USERVO_BAUDRATE);  //构建对象，串口3自动初始化
// FSUS_Protocol protocol(&Serial3);
// //构建对象，串口3自动初始化,默认波特率115200
FSUS_Servo uservo(SERVO_ID, &protocol);  // 创建舵机
// 创建智能机械爪实例
FSGP_Gripper gripper(&uservo, SERVO_ANGLE_GRIPPER_OPEN,
                     SERVO_ANGLE_GRIPPER_CLOSE);

void setup() {
  Serial.begin(DEBUG_BAUDRATE);
  uservo.init();                              // 串口总线舵机初始化
  Serial.println("Start To Test Gripper\n");  // 打印日志

  protocol.init();  // 舵机通信协议初始化
  gripper.init();   // 爪子初始化
}

void loop() {
  Serial.println("Gripper Close\n");  // 打印日志
  gripper.setAngle(SERVO_ANGLE_GRIPPER_CLOSE, GRIPPER_INTERVAL_MS,
                   GRIPPER_MAX_POWER);
  delay(3000);                       // 等待1s
  Serial.println("Gripper Open\n");  // 打印日志
  gripper.setAngle(SERVO_ANGLE_GRIPPER_OPEN, GRIPPER_INTERVAL_MS,
                   GRIPPER_MAX_POWER);
  delay(3000);
}
