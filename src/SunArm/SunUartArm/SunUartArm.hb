/* SunArm library 2021
* 修改自：https://github.com/RorschachUK/meArm_Adafruit

 * A simple control library for SunArm
 * Usage:
 *   SunArm arm;
 *   arm.begin(1, 10, 9, 6);
 *   arm.openGripper();
 *   arm.gotoPoint(-80, 100, 140);
 *   arm.closeGripper();
 *   arm.gotoPoint(70, 200, 10);
 *   arm.openGripper();
 */
#ifndef SUNUARTARM_H
#define SUNUARTARM_H

#include <Arduino.h>
#include "SunConfig.h"

const float pi=3.14159265359;

struct ServoInfo {
    int n_min, n_max;   // PWM 'soft' limits - should be just within range软限位
    float gain;         // PWM per radian pwm 每弧度
    float zero;         // Theoretical PWM for zero angle零度的理论值
};

class SunUartArm {
  public:
    //Full constructor uses calibration data, or can just give pins校准数据
    //构造函数 组合类构造函数
    SunUartArm(HardwareSerial& uartPort,FSUS_SERVO_ID_T base=0, FSUS_SERVO_ID_T shoulder=1, FSUS_SERVO_ID_T elbow=2,
     FSUS_SERVO_ID_T wrist=3, FSUS_SERVO_ID_T gripper=4);



    //required before running
    void begin(byte pinBase, byte pinShoulder, byte pinElbow, byte pinGripper);
    void end();
    //Travel smoothly from current point to another point
    void gotoPoint(float x, float y, float z);
    //Set servos to reach a certain point directly without caring how we get there 
    void goDirectlyTo(float x, float y, float z);

    //Same as above but for cylindrical polar coodrinates
    //援助坐标系
    void gotoPointCylinder(float theta, float r, float z);
    void goDirectlyToCylinder(float theta, float r, float z);

    //Grab something
    void openGripper();
    //Let go of something
    void closeGripper();
    //Check to see if possible
    bool isReachable(float x, float y, float z);
    //Current x, y and z
    float getX();
    float getY();
    float getZ();

    float getR();
    float getTheta();
  private:
    void polarToCartesian(float theta, float r, float& x, float& y);
    float _x, _y, _z;
    float _r, _t;
    ServoInfo _svoBase, _svoShoulder, _svoElbow, _svoGripper;
   
    HardwareSerial* _uartPort;
    FSUS_SERVO_ID_T _base,_shoulder,_elbow, _wrist,_gripper;
 
    FSUS_Protocol protocol;//通讯协议类成员
    FSUS_Servo baseServo,shouldeServo,elbowServo,wristServo;
    FSGP_Gripper gripperServo;//智能抓取舵机
   //void IKMapServo(float x,byte k,in b);
   //void IKMapServo(ik结算角度,斜率,截距);
   //k斜率1 或-1，转动正方向是否相同；b 截距 x=0时 实际关节的角度,角度制
    float IKMapServo(float x,byte k,int b);
    //角度转弧度
inline float ang2rad(float angle) { return angle / 180.0 * PI ; }
//弧度转角度
inline float rad2ang(float radian) { return radian* 180.0 / PI; }

};


#endif
