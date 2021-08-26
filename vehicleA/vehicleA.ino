/************************************
 * 2021-08-26 by csr
 ************************************/
#include "vehicle.h"

const float start_x=10,start_y=150; // 设置初始位置，分别表示从起始点到纠正触发点x和y方向距离

// 指令清单
// 第一个值为指令内容，第二个值为指令参数（浮点数）
// 必须以START开始，以END结尾。
//   LEFTWARD/RIGHTWARD/BACKWARD/FORWARD: 向左/右/后/前走若干格
//   PAUSE: 暂停若干秒
COMMAND commands[]={
	{LEFTWARD,0.3},
	{FORWARD,1.8},
	{PAUSE,3000}, // 扫码
	{FORWARD,2.5},
	{PAUSE,3000}, // 夹取物料A
	{FORWARD,1.5},
	{LEFTWARD,3},
	{PAUSE,3000}, // 夹取物料B
	{BACKWARD,3},
	{PAUSE,3000}, // 装配
	{RIGHTWARD,2},
	{BACKWARD,2.9},
	{RIGHTWARD,1.4},
	{END,0}
};

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
	// vehicle.start_button();
	vehicle.init(start_x,start_y,commands);
	FlexiTimer2::set(TIMER_PERIOD, control);  //定时中断函数，TIMER_PERIOD为宏定义
	FlexiTimer2::start();
}

void loop() 
{
	if (!vehicle.is_dmp_ready()) return;
	vehicle.redress_odometry();	// 灰度传感器修正里程计
	vehicle.get_current_status(); // 读取当前状态信息
	vehicle.run(); // 执行命令
	vehicle.update_goal_speed();
}
