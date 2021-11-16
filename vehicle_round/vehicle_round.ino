/************************************
 * 2021-08-26 by csr
 ************************************/

//#define TestMode1 // 关闭灰度传感器巡线和里程计纠正
//#define TestMode2 // 直接启动（单车调试）
//#define CloseBrake // 关闭急停

#define vehicle2

#include "vehicle.h"

// 指令清单
#ifdef vehicle1
const float start_x = 290, start_y = 150; // 设置初始时刻灰度传感器中点的位置
COMMAND commands[] = {
	{START, 0},
	{LEFTWARD, 0.45},
	{FORWARD, 1.6},
	//=====扫码=====
	{RIGHTWARD, 0.2},
	{PAUSE, 0},
	{SCANQR, 0},
	{PAUSE, 0},
	{LEFTWARD, 0.2},
	//=====拾取A===
	{GOX, 4.59},
	{PAUSE,1000},
	{GET1,0},
	{GOX,5.1},
	{PAUSE,1000},
	{GET1,0},
	{GOX,5.6},
	{PAUSE,1000},
	{GET1,0},
	{PAUSE, 0},
	//=====拾取B===
	{GOX,7.14},
	{PAUSE,0},
	{CHANGEDIR,0},
	{PAUSE,0},
	{GOY,3.4},
	{PAUSE,1000},
	{GET2,0},
	{GOY,4},
	{FINDLINEY,0},
	{PAUSE,1000},
	{GET2,0},
	{GOY,4.45},
	{PAUSE,1000},
	{GET2,0},
	{PAUSE,0},
	//=====装配=====
	{GOY,4},
	{GOX,5},
	{PAUSE, 3000}, // 装配
	{GOY,1.5},
	{GOX,1.2},
	{GOY,0.6},
	{END, 0}};
#endif
#ifdef vehicle2
const float start_x = 290, start_y = 2250; // 设置初始时刻灰度传感器中点的位置
COMMAND commands[] = {
	{START, 0},
	{RIGHTWARD,2.5},
	{PAUSE, 0},
	{BROADCAST,0}, // 播报扫码颜色
	{FINDLINEY, 0},
	{GOX, 3},
	{PAUSE, 0},
	{WORK,0},
	{LEFTWARD, 2},
	{FINDLINEY, 0},
	{BACKWARD, 2},
	{LEFTWARD, 0.4},
	{END, 0}};
#endif

Vehicle vehicle;

void control()
{
	sei(); //全局中断开启
	vehicle.get_motor_speed();
	vehicle.update_odometry();
	vehicle.set_motor_speed();
}

void setup()
{
	vehicle.start();
	vehicle.init(start_x, start_y, commands);
	FlexiTimer2::set(TIMER_PERIOD, control); //定时中断函数，TIMER_PERIOD为宏定义
	FlexiTimer2::start();
}

void loop()
{
	if (!vehicle.is_dmp_ready())
		return;
#ifndef TestMode1
	vehicle.redress_odometry(); // 灰度传感器修正里程计
#endif
	vehicle.get_current_status(); // 读取当前状态信息
	vehicle.run();				  // 执行命令
	vehicle.update_goal_speed();
}
