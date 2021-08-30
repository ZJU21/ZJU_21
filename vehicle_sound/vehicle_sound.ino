/************************************
 * 2021-08-30 by csr
 ************************************/
#define vehicle2	// 设置车辆编号（vehicle1/vehicle2）

// 多个TestMode可以同时开启
//#define TestMode1 // 关闭灰度传感器巡线和里程计纠正（用于架空调试）
//#define TestMode2 // 开机直接启动（用于单车调试）
//#define CloseBrake // 关闭急停

#include "vehicle.h"

const float start_x=10,start_y=150; // 设置初始位置，分别表示纠正触发点相对于起始点x和y方向位移

// 指令清单
// 第一个值为指令内容，第二个值为指令参数（浮点数）
// 必须以START开始，以END结尾。
// 可在vehicle.h的COMMAND_TYPE和run()中添加新的指令
//   LEFTWARD/RIGHTWARD/BACKWARD/FORWARD: 向左/右/后/前走若干格
//   PAUSE: 暂停若干秒
#ifdef vehicle1
COMMAND commands[]={
	{START,0},
	{LEFTWARD,0.45},
	{FORWARD,1.8},
	{PAUSE,3000}, // 扫码
	{FORWARD,2.3},
	{PAUSE,3000}, // 夹取物料A
	{FORWARD,1.3},
	{LEFTWARD,2.8},
  {FINDLINEY,0},
	{PAUSE,3000}, // 夹取物料B
	{BACKWARD,2.5},
	{PAUSE,3000}, // 装配
	{RIGHTWARD,2.7},
	{BACKWARD,2},
	{RIGHTWARD,0.4},
	{END,0}
};
#endif
#ifdef vehicle2
COMMAND commands[]={
  {START,0},
  {RIGHTWARD,3.2},
  {FINDLINEY,0},
  {FORWARD,2.3},
  {PAUSE,3000},
  {LEFTWARD,2.6},
  {FINDLINEY,0},
  {BACKWARD,2},
  {LEFTWARD,0.4},
  {END,0}
};
#endif

Vehicle vehicle;

void control()
{
	sei();  //全局中断开启
	vehicle.get_motor_speed();
	vehicle.update_odometry();
	vehicle.set_motor_speed();
}

void setup() 
{
#ifndef TestMode2
	vehicle.start();
#endif
	vehicle.init(start_x,start_y,commands);
	FlexiTimer2::set(TIMER_PERIOD, control);  //定时中断函数，TIMER_PERIOD为宏定义
	FlexiTimer2::start();
}

void loop() 
{
	if (!vehicle.is_dmp_ready()) return;
#ifndef TestMode1
	vehicle.redress_odometry();	// 灰度传感器修正里程计
#endif
	vehicle.get_current_status(); // 读取当前状态信息
	vehicle.run(); // 执行命令
	vehicle.update_goal_speed();
}
