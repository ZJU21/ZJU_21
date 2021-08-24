/* 
 * 设置舵机的角度 适合mega2560 使用两个硬串口
 * Sunny 修改版
 * 让舵机在两个角度之间进行切换, 并动态的查询舵机的角度

 * --------------------------

 * 更新时间: 2021/03/14
 */

#include "SunConfig.h" // 串口总线舵机通信协议  Fashion Star串口总线舵机的依赖

// 配置参数
#define SERVO_ID 5 //舵机ID号
#define USERVO_BAUDRATE 115200 // 舵机控制板串口波特率
#define DEBUG_BAUDRATE 19200 //调式串口波特率

FSUS_Protocol protocol(&Serial3, USERVO_BAUDRATE); //构建对象，串口3自动初始化
//FSUS_Protocol protocol(&Serial3); //构建对象，串口3自动初始化,默认波特率115200
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机


void setup(){
    Serial.begin(DEBUG_BAUDRATE);
    uservo.init(); // 串口总线舵机初始化
    Serial.println("Set Servo Angle");
}

void loop(){
    Serial.println("Set Angle = -88°");
    uservo.setAngle(-88.0);  // 设置舵机的角度
    uservo.wait();          // 等待舵机旋转到目标角度
    uservo.queryAngle();    // 再次更新一下角度
    Serial.println("Real Angle = " + String(uservo.curAngle, 1) + " Target Angle = "+String(uservo.targetAngle, 1));
    
    //delay(2000); // 暂停2s
    
    Serial.println("Set Angle = 0°");
    uservo.setAngle(0);
    uservo.wait();          // 等待舵机旋转到目标角度
    uservo.queryAngle();    // 再次更新一下角度
    Serial.println("Real Angle = " + String(uservo.curAngle, 1) + " Target Angle = "+String(uservo.targetAngle, 1));

    //delay(2000); // 暂停2s
}