/**
 * @file SunConfig.h
 * @author igcxl (igcxl@qq.com)
 * @brief Sunnybot可重构模块化教育机器人配置文件
 * @version 0.4 调整了配置文件结构 ，将引脚定义按板分离出去
 * @date 2021-04-28
 * @copyright Copyright (c) 2020
 *
 */

#pragma once
/**
 * SunConfig.h
 *
 * Basic settings such as:
 *
 * - Type of electronics 主控型号
 *
 *
 */
#include <arduino.h>
#define CONFIGURATION_H_VERSION 000003

//===========================================================================
//============================= Getting Started =============================
//===========================================================================
//******************主控板规格***************************//
#ifndef MOTHERBOARD
//#define MOTHERBOARD BOARD_SG_MEGA_2560
// #define MOTHERBOARD BOARD_PHCZJ_MEGA_2560
#define MOTHERBOARD BOARD_QIYI_MEGA_2560
// #define MOTHERBOARD BOARD_SUNNYBOT_TEENSY_3
//#define MOTHERBOARD BOARD_SUNNYBOT_ESP32
#endif

#ifndef VOLT_THRESHOLD
#define VOLT_THRESHOLD 1110  ///<电压阈值1110v
#endif
//******************调试模式***************************//
#define DEBUG_INFO  ///开启调试模式
//#define DEBUG_TTS   ///开启语音调试模式
//******************串口0波特率***************************//
/**
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 115200

//*****************定时中断间隔***************************//
#define TIMER_PERIOD 10

/*
为了避免同一个头文件被包含（include）多次，C/C++中有两种宏实现方式：一种是#ifndef方式；另一种是#pragma
once方式。 示例代码如下：

 方式一：
 #ifndef  __SOMEFILE_H__
#define   __SOMEFILE_H__
 ... ... // 声明、定义语句
#endif
方式二：
#pragma once
 ... ... // 声明、定义语句
1.#include<>

1.在编译器设置的include路径内搜索；
2.如果是在命令行中编译，则在系统的INCLUDE环境变量内搜索。

2 #include”“
1.在包含当前include指令的文件所在的文件夹内搜索；
2.如果上一步找不到，则在之前已经使用include指令打开过的文件所在的文件夹内搜索，如果已经有多个被include的文件，则按照它们被打开的相反顺序去搜索；
3.如果上一步找不到，则在编译器设置的include路径内搜索；
4.如果上一步找不到，则在系统的INCLUDE环境变量内搜索。

<>引用方式，编译器仅在标准库头文件中进行匹配；

""引用方式，编译器先在工程目录下进行匹配，如果没有，再到标准库头文件中查找。
https://www.arduino.cc/reference/en/language/structure/further-syntax/include/
a.绝对路径引用，如#include"D:/test/hanshu.h"。但是使用这种方法，项目只能在本机上正常运行，换台机器就可能无法编译，不建议使用；

b.相对路径，如#include"../test/hanshu.h"。这种方法适用于代码中一个项目中；

c.库路径引用，需要将hanshu.h所在路径添加到项目的包含文件路径中，再通过<>引用，如#include<hanshu.h>。
*/

//#include "dir/test.h" 当前目录下dir文件下test.h

#ifndef GRAYSCALE_DEFAULT_PORT
#ifdef HAVE_HWSERIAL2
#define GRAYSCALE_DEFAULT_PORT Serial2  //循迹传感器默认串口
#else
#define GRAYSCALE_DEFAULT_PORT Serial  //循迹传感器默认串口
#endif
#endif
#define USE_GRAY_UART  //使用串口
#define USE_GRAY_IO    //使用IO口
#define USE_PS2        //使用PS2遥控手柄

//********************舵机控制板***************************//

#ifdef HAVE_HWSERIAL3
#define WTServo16_DEFAULT_PORT Serial3  //舵机控制板默认串口
#else
#define WTServo16_DEFAULT_PORT Serial  //舵机控制板器默认串口
#endif
//*******************机器人底盘电机参数***************************//
// define your robot' specs here
//电机参数
//#define MAX_RPM 500              // motor's maximum RPM
// 2019常用电机规格，请根据实际电机数据填写 #define MOTOR_TORQUE 1.5 // motor's
// torque  2019电机规格 kgf.cm，请根据实际电机数据填写

//根据实际使用电机选择宏定义
//#define USE_12V366RPM13PPR30_MOTOR //详细参数：https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-15726392041.15.1e936ab8Hc8eQf&id=45347924687
#define USE_12V333RPM11PPR30_MOTOR //详细参数：https://item.taobao.com/item.htm?spm=a1z0d.6639537.1997196601.1054.79cb7484Vto1GY&id=609636943444

#ifdef USE_12V366RPM13PPR30_MOTOR
#define MAX_RPM \
  366  // 电机额定转速  2020电机规格，请根据实际电机数据填写
#define MOTOR_TORQUE \
  1  // motor's torque  2020电机规格 1kg.cm，请根据实际电机数据填写
#define COUNTS_PER_REV 1560  // wheel encoder's num of ticks per rev 390*4=1560
#endif
#ifdef USE_12V333RPM11PPR30_MOTOR
#define MAX_RPM \
  333  // 电机额定转速  2020电机规格，请根据实际电机数据填写
#define MOTOR_TORQUE \
  1  // motor's torque  2020电机规格 1kg.cm，请根据实际电机数据填写
#define COUNTS_PER_REV 1320  // wheel encoder's num of ticks per rev 11*30*4=1320
#endif
 
//******************电机驱动规格***************************//
// uncomment the motor driver you're using
#define USE_A4950_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_L298_DRIVER
//**********************IMU规格***************************//
// #define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU6500_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

//********************机械臂参数（未使用）***************************//
const float L_UPPERARM = 148;  // unit:millimetres Shoulder to elbow length
const float L_FOREARM = 160;   // unit:millimetres Elbow to wrise length
const float L_OFFSET = 23;  // unit:millimetres Length from wrist to hand PLUS
                            // base centre to shoulder
//******************PID参数***************************//
#define K_P 10  // P constant
#define K_I 0   // I constant
#define K_D 0   // D constant

//******************轮子引脚定义***************************//
/**
 * 轮子定义Z字型定义 观察角度俯视 从上往下看
 * *               |               |
 *    MotorA (1)-->| +-- FRONT ---+|<-- MotorB (2)
 *       [5,6]     |       |       |     [8,7]
 *Encoder[2,51]    L |     |     | R     [3,53]
 *                 E |     |     | I
 *                 F |     |     | G
 *                 T |     |     | H
 *                   |     |     | T
 *                   |     |     |
 *                 |       |       |
 *    MotorC (3)-->| +-- BACK  ---+|<-- MotorD (4)
 *  [11/12,12/9]   |               |     [44,46]
 *Encoder[18,52]                           [19,50]
----------------------

ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
//******************TFT显示屏***************************//
#define USE_TFT_LCD
#ifdef USE_TFT_LCD

//#define LS18TFT  //绿深1.8 TFT 3.3V 8针
//#define JMD18TFT       //晶美达1.8 TFT 7针 3.3V  ST7735S GM177-02
#define YUROBOT18TFT       //YUROBOT深1.8 TFT 5V 8针

#include "pins/pins.h"  //根据主控板型号加载引脚定义
//#include "SunProtocol/DataScope/DataScope.h"//上位机通讯协议库

//#include "SunArm/SunUartArm/IKArm.h"      //串口舵机运动学逆解库 Sunny add
#include "SunController/SunPID/SunPID.h"  //PID控制器库
#include "SunRover/SunKinematics/SunKinematics.h"  //移动单元运动学库
#include "SunRover/SunOdometry/SunWheelOdometry.hpp"  //轮式里程计库
#include "SunRover/SunCar/SunCar.hpp"  //全局运动库
#include "SunSensors/SunBattery/SunBattery.h"      //电池电压检测库
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#include "SunSensors/SunEncoder/SunEncoder.h"      //编码器库
#include "SunSensors/SunGrayscale/SunGrayscale.h"  //灰度传感器库
#endif

//#include "SunSensors/SunUartMux/SunUartMux.h"//串口多路复用库

#include "SunMotorDriver/SunA4950Motors/SunA4950Motors.h"  //A4950驱动器库
#include "SunServo/SunWTServo16/SunWTServo16.h"  //16路PWM舵机控制板库


//******************机器人底盘构型***************************//

// uncomment the base you're building
//#define SUN_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define SUN_BASE SKID_STEER      // 4WD robot
// #define SUN_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define SUN_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
//#define SUN_BASE MECANUM_X         // Mecanum drive robot X Type


#define TYPE_BALL1 1
#define TYPE_BLANCE2 2
#define TYPE_DIFFERENTIAL_DRIVE3 3
#define TYPE_OMNI3 4
#define TYPE_ACKERMAN 5
#define TYPE_MECANUM_X 6
#define TYPE_SKID4WD 7
#define TYPE_MNI_X 8
#define TYPE_DEFAULT_2WD_4WD 9


#ifndef BASEPLATFORM
#define BASEPLATFORM MECANUM_X
#endif

#define BASETYPE TYPE_MECANUM_X

#define BT(T) (BASETYPE==TYPE_##T)

#if BT(MECANUM_X)
#define WHEEL_NUM 4
#define WHEEL_DIAMETER 0.065  // wheel's diameter in meters 0.06、0.065、0.08 必须保留小数点
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离 必须保留小数点
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m  0.165 共轴 0.19 必须保留小数点
#define CENTER_WHEELS_DISTANCE 0.1 //  三轮旋转中心到轮距离，单位m
#elif BT(OMNI3)
#define WHEEL_NUM 3           //主动轮数量
#define WHEEL_DIAMETER 0.075  // wheel's diameter in meters 0.06、0.065、0.08
#define CENTER_WHEELS_DISTANCE 0.10  // 三轮旋转中心到轮自距离，单位m

#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#elif BT(ACKERMAN)
#define WHEEL_NUM 2           //主动轮数量
#define WHEEL_DIAMETER 0.065  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#define MAX_STEERING_ANGLE \
  0.415  // max steering angle. This only applies to Ackermann steering

#elif BT(SKID4WD)
#define WHEEL_NUM 4
#define WHEEL_DIAMETER 0.060  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#elif BT(DEFAULT_2WD_4WD)
#define WHEEL_NUM 4
#define WHEEL_DIAMETER 0.060  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#else

#define WHEEL_NUM 4
#define WHEEL_DIAMETER 0.065  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m

#endif
/**
 * @brief
 *
 * #define PWM_BITS 8  // PWM Resolution of the microcontroller
//车体参数
#define MAX_STEERING_ANGLE \
  0.415  // max steering angle. This only applies to Ackermann steering
//差速
//#define LR_WHEELS_DISTANCE 0.17  // distance between left and right
// wheels轮距，单位m #define FR_WHEELS_DISTANCE 0.128 // distance between front
// and rear wheels. Ignore this if you're on 2WD/ACKERMANN 轴距，单位m 麦轮65mm
//底装 共轴
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距
        // moni中心到轮距离
#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m

         #define WHEEL_DIAMETER 0.065  // wheel's diameter in meters
0.06、0.065、0.08
         //麦轮80mm
//#define LR_WHEELS_DISTANCE 0.19  // distance between left and right
// wheels左右轮距，单位m 左右轮距 #define FR_WHEELS_DISTANCE 0.128 // distance
// between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
//前后轴距，单位m
 *
 */

#endif