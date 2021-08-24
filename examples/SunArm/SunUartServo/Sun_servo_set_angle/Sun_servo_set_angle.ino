/* 
 * 设置舵机的角度 软串口 适合uno
 * Sunny 修改版
 * 让舵机在两个角度之间进行切换, 并动态的查询舵机的角度
 * 提示: 拓展板上电之后, 记得按下Arduino的RESET按键
 * --------------------------

 * 更新时间: 2021/03/07
 */
#include <SoftwareSerial.h>
#include "SunConfig.h" // 串口总线舵机通信协议  Fashion Star串口总线舵机的依赖
// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

// 配置参数
#define BAUDRATE 115200 // 波特率
#define SERVO_ID 0 //舵机ID号

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
    protocol.init(); // 通信协议初始化
    uservo.init(); //舵机角度初始化
    softSerial.println("Set Servo Angle");
}

void loop(){
    softSerial.println("Set Angle = 135°");
    uservo.setAngle(90.0);  // 设置舵机的角度
    uservo.wait();          // 等待舵机旋转到目标角度
    uservo.queryAngle();    // 再次更新一下角度
    softSerial.println("Real Angle = " + String(uservo.curAngle, 1) + " Target Angle = "+String(uservo.targetAngle, 1));
    
    delay(2000); // 暂停2s
    
    softSerial.println("Set Angle = -135°");
    uservo.setAngle(-90);
    uservo.wait();          // 等待舵机旋转到目标角度
    uservo.queryAngle();    // 再次更新一下角度
    softSerial.println("Real Angle = " + String(uservo.curAngle, 1) + " Target Angle = "+String(uservo.targetAngle, 1));

    delay(2000); // 暂停2s
}