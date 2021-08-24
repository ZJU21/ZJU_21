/**
 * 爪子开启关闭实验(带功率约束)
 * --------------------------

 * 更新时间: 2021/01/13
 * 串行总线舵机使用手册 https://wiki.fashionrobo.com/uartbasic/
 *  舵机需要独立供电
 * 舵机号不能冲突
 * 

 */

#include "SunConfig.h"  // 串口总线舵机通信协议 Fashion Star串口总线舵机 Fashion Star智能夹具
// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
    #include <SoftwareSerial.h>
    #define SOFT_SERIAL_RX 6
    #define SOFT_SERIAL_TX 7
    SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
    #define DEBUG_SERIAL softSerial
    #define DEBUG_SERIAL_BAUDRATE 4800
#elif defined(ARDUINO_AVR_MEGA2560)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
#elif defined(ARDUINO_ARCH_ESP32)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
#endif 
// 串口总线舵机配置
#define SERVO_ID 4              //爪子对应的舵机ID号，按实际舵机编号填写
#define USERVO_BAUDRATE 115200  // 舵机控制板串口波特率
#define DEBUG_BAUDRATE 115200    //调式串口波特率

// 爪子的配置
#define SERVO_ANGLE_GRIPPER_OPEN -102.0  // 爪子张开时的角度**根据爪子实际修改
#define SERVO_ANGLE_GRIPPER_CLOSE -12.0  // 爪子闭合时的角度

// 创建舵机的通信协议对象
//FSUS_Protocol protocol(&Serial3, USERVO_BAUDRATE);  //构建对象，串口3自动初始化
// FSUS_Protocol protocol(&Serial3);
FSUS_Protocol protocol(BAUDRATE);
// //构建对象,默认波特率115200
FSUS_Servo uservo(SERVO_ID, &protocol);  // 创建舵机
// 创建智能机械爪实例
FSGP_Gripper gripper(&uservo, SERVO_ANGLE_GRIPPER_OPEN,SERVO_ANGLE_GRIPPER_CLOSE);

void setup() {
   DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 软串口
    DEBUG_SERIAL.println("Start To Test Gripper\n"); // 打印日志
  protocol.init();    // 舵机通信协议初始化    
  uservo.init();                              // 串口总线舵机初始化
  Serial.println("Start To Test Gripper\n");  // 打印日志
  gripper.init();                             // 爪子初始化
  // 参数配置
  gripper.setMaxPower(400);  // 设置最大功率，单位mW
}

void loop() {
  Serial.println("Gripper Close\n");  // 打印日志
  gripper.close();
  Serial.print(uservo.queryAngle());     // 打印日志
  Serial.print(",");                     // 打印日志
  Serial.println(uservo.queryPower());   // 打印日志
  Serial.println(gripper.queryPower());  // 打印日志
  delay(5000);                           // 等待5s

  Serial.println("Gripper Open\n");  // 打印日志
  gripper.open();
  Serial.print(uservo.queryAngle());     // 打印日志
  Serial.print(",");                     // 打印日志
  Serial.println(uservo.queryPower());   // 打印日志
  Serial.println(gripper.queryPower());  // 打印日志
}