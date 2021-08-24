#ifndef SUN_A4950MOTORS_h //防止头文件被多次引用
#define SUN_A4950MOTORS_h


#include <Arduino.h>
#include "SunConfig.h"
//四路
class A4950MotorShield
{
public:
  //构造函数
  //函数名必须与类名相同
  //没有返回值
  A4950MotorShield();
  //用户自定义构造函数，函数重载
  A4950MotorShield(byte M1PWMA, byte M1PWMB, byte M2PWMA, byte M2PWMB, byte M3PWMA, byte M3PWMB, byte M4PWMA, byte M4PWMB);
  A4950MotorShield(byte M1PWMA, byte M1PWMB, byte M2PWMA, byte M2PWMB, byte M3PWMA, byte M3PWMB);
  //初始化函数1
  void init();
    //初始化函数2
  void init(bool HForLF,byte mtrDeadZone);
  //设置电机1速度
  void setM1Speed(int speed);
  //设置电机2速度
  void setM2Speed(int speed);
  //设置电机3速度
  void setM3Speed(int speed);
  //设置电机4速度
  void setM4Speed(int speed);
  //设置左右侧电机速度
  void setSpeeds(int LeftSpeed, int RightSpeed);
  //设置3个电机速度，来福轮
  void setSpeeds(int m1Speed, int m2Speed, int m3Speed);
  //设置4个电机速度，麦克纳木伦
  void setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed);
  //快速停Fast stop，慢速停slow stop setSpeeds(0,0)
  void motorsBrake();


//静态变量的特点
//1.静态变量无需生成对象就可被调用,可以使用类名和对象两种方法调用
//2.静态变量是全局变量
//3.任何一个对象修改静态变量的值，那么所有的该类的对象的静态成员变量的值都改变
  static byte motorDeadZone; ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试静态成员是可以独立访问的，也就是说，无须创建任何对象实例就可以访问
  //在类成员变量前加static，这个变量的作用域就不只局限于类成员变量中，这个变量就是一个静态全局变量，可以在引用该头文件的任意一个函数中使用，但是需要在前面加类名::，并且初始化不能在函数中初始化，需要在函数外初始化，在类中初始化也不行。
//高频噪声低，但是有高频损耗


private:
  byte _M1PWMA;
  byte _M1PWMB;
  byte _M2PWMA;
  byte _M2PWMB;
  byte _M3PWMA;
  byte _M3PWMB;
  byte _M4PWMA;
  byte _M4PWMB;
};
#endif