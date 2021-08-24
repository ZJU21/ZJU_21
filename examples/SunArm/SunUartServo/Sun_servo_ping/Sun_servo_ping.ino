/*
 * 舵机通讯检测
 * Sunny 修改版
 * 通讯协议构建实例后，自动初始化不需要载protocol.init()；
 * 串口0打印 串口3与舵机控制板通讯
 * --------------------------
*/
#include "SunConfig.h" // 串口总线舵机通信协议  Fashion Star串口总线舵机的依赖



// 配置
#define SERVO_ID 0 //舵机ID号
#define USERVO_BAUDRATE 115200 // 舵机控制板串口波特率
#define DEBUG_BAUDRATE 19200 //调式串口波特率

FSUS_Protocol protocol(&Serial3, USERVO_BAUDRATE); //构建对象，串口3自动初始化
//FSUS_Protocol protocol(&Serial3); //构建对象，串口3自动初始化,默认波特率115200
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

void setup(){
    Serial.begin(DEBUG_BAUDRATE);
    uservo.init(); // 串口总线舵机初始化
    Serial.println("Start To Ping Servo\n"); // 打印日志
}

void loop(){
    bool isOnline = uservo.ping(); // 舵机通讯检测
    String message = "servo #"+String(uservo.servoId,DEC); // 日志输出
    if(isOnline){
        Serial.println(message+" is online.");
    }else{
        Serial.println(message+" is offline.");
    }
    // 等待1s
    delay(1000);
}
