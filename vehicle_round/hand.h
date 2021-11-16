#ifndef _HAND_H
#define _HAND_H

#include "FashionStar_Arm5DoF.h"
#include "camera.h"

// 调试串口的配置
//#define DEBUG_SERIAL Serial
//#define DEBUG_SERIAL_BAUDRATE 115200

// 爪子的配置
#define SERVO_ANGLE_GRIPPER_OPEN 30 // 爪子张开时的角度
#define SERVO_ANGLE_GRIPPER_CLOSE -1.5 // 爪子闭合时的角度

class Hand
{
	FSARM_ARM5DoF arm; //机械臂对象
	Camera cam;	//新建相机实例
	FSARM_JOINTS_STATE_T thetas;    // 关节角度 
	FSARM_POINT3D_T toolPosi;       // 末端的位置  
	float pitch;                    // 末端倾角
	private:
	void move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num,float push_num);
	void print_theta(FSARM_JOINTS_STATE_T thetas);
	void print_toolPosi(FSARM_POINT3D_T toolPosi, float pitch);
	void run_action_group();
	void trail_move_action(FSARM_JOINTS_STATE_T *thetas,int command_num);
	void tiny_move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num);
	void Forward_and_backward_forward(float forward_push_num);
	void set_thetas(double a,double b,double c,double d);
	public:
	void scan1();
	void scan2();
	int scanQRcode();
	bool try_catchA();
	bool try_catchB();
	void init();
	void catchA1();
	void catchA2();  
	void catchB1();
	void catchB2();  
	void put1(int cnt);
	void put2(int cnt);
};

void Hand::scan1()
{
	arm.scan1();
}

void Hand::scan2()
{
	arm.scan2();
}
int Hand::scanQRcode()
{
	arm.scan1();
	FSARM_JOINTS_STATE_T vib_thetas;    // 关节角度
	arm.gripperOpen();
	arm.queryAngle(&vib_thetas);
	int x;
	for (x=cam.get_QRcode();x==9;x=cam.get_QRcode())
	{
		vib_thetas.theta1 = vib_thetas.theta1+3;    // 关节角度;
		arm.setAngle(vib_thetas);       // 设置舵机旋转到特定的角度
		arm.wait();
		vib_thetas.theta1 = vib_thetas.theta1-6;    // 关节角度;
		arm.setAngle(vib_thetas);       // 设置舵机旋转到特定的角度
		arm.wait();
		vib_thetas.theta1 = vib_thetas.theta1+3;    // 关节角度;
		arm.setAngle(vib_thetas);       // 设置舵机旋转到特定的角度
		arm.wait();

	}
	return x;
}

bool Hand::try_catchA()
{
	Position pos=cam.get_Aposition();
	return pos.size>500 && pos.x<50 && pos.x>-50;
}

bool Hand::try_catchB()
{
	Position pos=cam.get_Bposition();
	return pos.size>500 && pos.x<50 && pos.x>-50;
}

void Hand::move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num,float push_num)    //命令编号 1：前进 2：左移 3：右移 4：上移 5：下移 6：后退
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度

	/*输出，可注释*/
	FSARM_JOINTS_STATE_T res_thetas;    // 更新后关节角度
	FSARM_POINT3D_T res_toolPosi;       // 更新后末端的位置
	float res_pitch;                    // 更新后末端倾角

	switch (command_num)
	{
		case 1:
			toolPosi->x=toolPosi->x+push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***前进***");
#endif
			break;
		case 2: 
			toolPosi->y=toolPosi->y+push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向左***");
#endif
			break;
		case 3:
			toolPosi->y=toolPosi->y-push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向右***");
#endif
			break;
		case 4:
			toolPosi->z=toolPosi->z+push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向上***");
#endif
			break;
		case 5:
			toolPosi->z=toolPosi->z-push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向下***");
#endif
			break;
		case 6:
			toolPosi->x=toolPosi->x-push_num;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***后退***");
#endif
			break;
	}
	pitch=0;
	arm.inverseKinematics(*toolPosi, *pitch, &thetas);// 逆向运动学

	/*输出，可注释*/
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("move前进中");
#endif
	print_theta(thetas);
	print_toolPosi(*toolPosi,*pitch);
	thetas.theta4=-thetas.theta3-thetas.theta2;
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait(); 

	/*输出，可注释*/
	arm.queryAngle(&res_thetas);    // 查询舵机的角度
	arm.forwardKinematics(res_thetas, &res_toolPosi, &res_pitch); // 正向运动学    
	//前进前位置输出
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进后");
	print_theta(res_thetas);
	print_toolPosi(res_toolPosi,res_pitch);
#endif

	//thetas.theta1=2*thetas.theta1-res_thetas.theta1;
	//thetas.theta2=2*thetas.theta2-res_thetas.theta2;
	//thetas.theta3=2*thetas.theta3-res_thetas.theta3;
	thetas.theta4=-res_thetas.theta3-res_thetas.theta2;
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait(); 

	/*输出，可注释*/
	arm.queryAngle(&res_thetas);    // 查询舵机的角度
	arm.forwardKinematics(res_thetas, &res_toolPosi, &res_pitch); // 正向运动学    
	//前进前位置输出
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进后2");
	print_theta(res_thetas);
	print_toolPosi(res_toolPosi,res_pitch);
	DEBUG_SERIAL.println("******");
	DEBUG_SERIAL.println("******");
#endif
}

void Hand::init()
{

#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口
#endif

	arm.init(); //机械臂初始化
	cam.init();

	arm.home();
	arm.queryAngle(&thetas);    // 查询舵机的角度
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("home pos");
#endif
	print_theta(thetas);
	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果

	arm.scan1();
	arm.queryAngle(&thetas);    // 查询舵机的角度
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("ready pos");
#endif
	print_theta(thetas);
	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果

	//delay(1500);
	//Forward_and_backward_forward(1);

	//delay(1500);
	//Forward_and_backward_forward(3);

	//delay(1500);
	//Forward_and_backward_forward(10);
	// 运行动作组1次
	//move_action(&toolPosi,&pitch,4,6);
}

void Hand::set_thetas(double a,double b,double c,double d)
{
	FSARM_JOINTS_STATE_T thetas;
	thetas.theta1=a;
	thetas.theta2=b;
	thetas.theta3=c;
	thetas.theta4=d;
	arm.setAngle(thetas);
	arm.wait();
}

void Hand::catchA1()
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度
	set_thetas(0,-72.00,103,-28.00);
	arm.gripperOpen();
	arm.queryAngle(&thetas);    // 查询舵机的角度
	Position pos;
	for (;abs(pos.x)>10;pos=cam.get_Aposition())
		if (pos.size>100)
		{
			if (pos.x>0)
				trail_move_action(&thetas,2);
			if (pos.x<0)
				trail_move_action(&thetas,3);
		}
	arm.gripperClose();
	set_thetas(0,-80.00,90,-28.00);
	return;
}

void Hand::catchB1()
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度
	set_thetas(0,-72.00,103,-28.00);
	arm.gripperOpen();
	arm.queryAngle(&thetas);    // 查询舵机的角度
	Position pos;
	for (;abs(pos.x)>10;pos=cam.get_Bposition())
		if (pos.size>100)
		{
			if (pos.x>0)
				trail_move_action(&thetas,2);
			if (pos.x<0)
				trail_move_action(&thetas,3);
		}

	arm.gripperClose();
	set_thetas(0,-80.00,90,-28.00);
	return;
}

void Hand::catchA2()
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度
	set_thetas(90,-72.00,103,-28.00);
	arm.gripperOpen();
	arm.queryAngle(&thetas);    // 查询舵机的角度
	Position pos;
	for (;abs(pos.x)>10;pos=cam.get_Aposition())
		if (pos.size>100)
		{
			if (pos.x>0)
				trail_move_action(&thetas,2);
			if (pos.x<0)
				trail_move_action(&thetas,3);
		}

	arm.gripperClose();
	set_thetas(90,-80.00,90,-28.00);
	set_thetas(90,-86.90,-25.40,87.00);
	return;
}
void Hand::catchB2()
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度
	set_thetas(90,-72.00,103,-28.00);
	arm.gripperOpen();
	arm.queryAngle(&thetas);    // 查询舵机的角度
	Position pos;
	for (;abs(pos.x)>10;pos=cam.get_Bposition())
		if (pos.size>100)
		{
			if (pos.x>0)
				trail_move_action(&thetas,2);
			if (pos.x<0)
				trail_move_action(&thetas,3);
		}

	arm.gripperClose();
	set_thetas(90,-80.00,90,-28.00);
	set_thetas(90,-86.90,-25.40,87.00);
	return;
}

void Hand::put1(int cnt)
{
	if (cnt)
	{
		set_thetas(37.00,-113.20,44.10,32.90);
		set_thetas(45.60,-71.70,-35.70,-28.00);
		set_thetas(45.70,-65.30,-66.50,-55.30);
		set_thetas(45.70,-81.70,-47.60,-71.90);
		arm.gripperOpen();
		set_thetas(45.60,-82.00,25.60,0.60);
		set_thetas(19.80,-141.00,110.20,0.80);
	}
	else
	{
		set_thetas(90.00,-113.20,44.10,32.90);
		set_thetas(90.00,-71.70,-35.70,-28.00);
		set_thetas(90.00,-65.30,-66.50,-55.30);
		set_thetas(90.00,-81.70,-47.60,-71.90);
		arm.gripperOpen();
		set_thetas(90.00,-82.00,25.60,0.60);
		set_thetas(0,-141.00,110.20,0.80);
	}
}

void Hand::put2(int cnt)
{
	if (cnt) return;
	set_thetas(-44.60,-86.90,-25.40,87.00);
	set_thetas(-52.70,-114.90,54.80,67.70);
	set_thetas(-49.60,-116.90,80.40,51.20);
	arm.gripperOpen();
	set_thetas(-49.30,-117.60,27.40,51.10);
	set_thetas(89.60,-121.00,34.80,51.20);
}


//打印print_theta信息
void Hand::print_theta(FSARM_JOINTS_STATE_T thetas){
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("thetas: 0= " + String(thetas.theta1, 2) +\
			", 1= " + String(thetas.theta2, 2) + \
			", 2= " + String(thetas.theta3, 2) + \
			", 3= " + String(thetas.theta4, 2) );
#endif
}

//打印toolposi信息
void Hand::print_toolPosi(FSARM_POINT3D_T toolPosi, float pitch){
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("Tool Posi: X= " + String(toolPosi.x, 1) +\
			", Y= " + String(toolPosi.y, 1) + \
			", Z= " + String(toolPosi.z, 1) + \
			", Pitch: " + String(pitch, 2) + "deg");
#endif
}

void Hand::run_action_group(){    

	FSARM_JOINTS_STATE_T thetas;    // 关节角度 
	FSARM_POINT3D_T toolPosi;       // 末端的位置  
	float pitch;                    // 末端倾角 

	//action1    "Tool Posi: X= 13.5, Y= 0.0, Z= 6.4, Pitch: 20"
	thetas.theta1 = 0;
	thetas.theta2 = -115.0;
	thetas.theta3 = 90.0;
	thetas.theta4 = 25;

	// 机械臂运行
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait();                 // 等待舵机旋转到目标位置

	//信息打印
	arm.queryAngle(&thetas);    // 查询舵机的角度
	print_theta(thetas);

	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果


	//action2    "Tool Posi: X= 23.6, Y= 0.0, Z= 11.2, Pitch: -10.00deg"
	thetas.theta1 = 0;
	thetas.theta2 = -60;
	thetas.theta3 = 40;
	thetas.theta4 = 20;

	// 机械臂运行
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait();                 // 等待舵机旋转到目标位置

	//信息打印
	arm.queryAngle(&thetas);    // 查询舵机的角度
	print_theta(thetas);

	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果

	//action3     "Tool Posi: X= 27.9, Y= 0.5, Z= -3.9, Pitch: -0.20 deg"
	thetas.theta1 = 0;
	thetas.theta2 = -50;
	thetas.theta3 = 70;
	thetas.theta4 = -20;

	// 机械臂运行
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait();                 // 等待舵机旋转到目标位置

	//信息打印
	arm.queryAngle(&thetas);    // 查询舵机的角度
	print_theta(thetas);

	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果



	//action0     "Tool Posi: X= 1.6, Y= 0.0, Z= 29.2, Pitch: -86.1 deg"
	thetas.theta1 = 0;
	thetas.theta2 = -30;
	thetas.theta3 = 90;
	thetas.theta4 = -60;

	// 机械臂运行
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait();                 // 等待舵机旋转到目标位置

	//信息打印
	arm.queryAngle(&thetas);    // 查询舵机的角度
	print_theta(thetas);

	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果
	delay(3000);        // 等待1s
}

void Hand::trail_move_action(FSARM_JOINTS_STATE_T *thetas,int command_num)    //命令编号 1：前进 2：左移 3：右移 4：上移 5：下移 6：后退
{


	/*输出，可注释*/
	FSARM_POINT3D_T toolPosi;
	float pitch;
	FSARM_JOINTS_STATE_T res_thetas;    // 更新后关节角度
	FSARM_POINT3D_T res_toolPosi;       // 更新后末端的位置
	float res_pitch;                    // 更新后末端倾角

	switch (command_num)
	{
		case 1:
			thetas->theta2= thetas->theta2+3;
			thetas->theta3= thetas->theta3-3;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***前进***");
#endif
			break;
		case 2: 
			thetas->theta1= thetas->theta1+3;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向左***");
#endif
			break;
		case 3:
			thetas->theta1= thetas->theta1-3;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***向右***");
#endif
			break;
			/*
			   case 4:
			   thetas->theta1= thetas->theta2-3;
			   DEBUG_SERIAL.println("***向上***");
			   break;
			   case 5:
			   toolPosi->z=toolPosi->z-push_num;
			   DEBUG_SERIAL.println("***向下***");
			   break;
			   */
		case 6:
			thetas->theta2= thetas->theta2-3;
			thetas->theta3= thetas->theta3+3;
#ifdef DEBUG_SERIAL
			DEBUG_SERIAL.println("***后退***");
#endif
			break;
	}
	thetas->theta4=-thetas->theta3-thetas->theta2;
	arm.forwardKinematics(*thetas, &toolPosi, &pitch); // 正向运动学

#ifdef DEBUG_SERIAL
	/*输出，可注释*/
	DEBUG_SERIAL.println("move前进中");
	print_theta(*thetas);
	print_toolPosi(toolPosi,pitch);   
#endif 
	arm.setAngle(*thetas);       // 设置舵机旋转到特定的角度
	arm.wait(); 

	/*输出，可注释*/
	arm.queryAngle(&res_thetas);    // 查询舵机的角度
	arm.forwardKinematics(res_thetas, &res_toolPosi, &res_pitch); // 正向运动学    
	//前进前位置输出
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进后");
	print_theta(res_thetas);
	print_toolPosi(res_toolPosi,res_pitch);
#endif

	//thetas.theta1=2*thetas.theta1-res_thetas.theta1;
	//thetas.theta2=2*thetas.theta2-res_thetas.theta2;
	//thetas.theta3=2*thetas.theta3-res_thetas.theta3;
	thetas->theta4=-res_thetas.theta3-res_thetas.theta2;
	arm.setAngle(*thetas);       // 设置舵机旋转到特定的角度
	arm.wait(); 

	/*输出，可注释*/
	arm.queryAngle(&res_thetas);    // 查询舵机的角度
	arm.forwardKinematics(res_thetas, &res_toolPosi, &res_pitch); // 正向运动学    
	//前进前位置输出
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进后2");
	print_theta(res_thetas);
	print_toolPosi(res_toolPosi,res_pitch);
	DEBUG_SERIAL.println("******");
	DEBUG_SERIAL.println("******");
#endif
}

void Hand::tiny_move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num)    //命令编号 1：前进 2：左移 3：右移 4：上移 5：下移 6：后退
{
	FSARM_JOINTS_STATE_T thetas;    // 关节角度

	/*输出，可注释*/
	FSARM_JOINTS_STATE_T res_thetas;    // 更新后关节角度
	FSARM_POINT3D_T res_toolPosi;       // 更新后末端的位置
	float res_pitch;                    // 更新后末端倾角

	switch (command_num)
	{
		case 1:
			toolPosi->x=toolPosi->x+1;
			break;
		case 2:
			toolPosi->y=toolPosi->y+1;
			break;
		case 3:
			toolPosi->y=toolPosi->y-1;
			break;
		case 4:
			toolPosi->z=toolPosi->z+1;
			break;
		case 5:
			toolPosi->z=toolPosi->z-1;
			break;
		case 6:
			toolPosi->x=toolPosi->x-1;
			break;
	}
	arm.inverseKinematics(*toolPosi, *pitch, &thetas);// 逆向运动学

	/*输出，可注释*/
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进中");
	print_theta(thetas);
	print_toolPosi(*toolPosi,*pitch);
#endif

	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.setAngle(thetas);
	arm.wait(); 

	/*输出，可注释*/
	arm.queryAngle(&res_thetas);    // 查询舵机的角度

	arm.forwardKinematics(res_thetas, &res_toolPosi, &res_pitch); // 正向运动学    
	//前进前位置输出
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("前进后");
	print_theta(res_thetas);
	print_toolPosi(res_toolPosi,res_pitch);
#endif

}

void Hand::Forward_and_backward_forward(float forward_push_num){
	FSARM_JOINTS_STATE_T thetas;    // 关节角度
	FSARM_POINT3D_T toolPosi;       // 末端的位置  
	float pitch;                    // 末端倾角
	float i;
	arm.queryAngle(&thetas);    // 查询舵机的角度
	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学
	for (i=forward_push_num;i>0;i--)
	{
		if (i>=1)
			tiny_move_action(&toolPosi,&pitch,1);
		else 
			move_action(&toolPosi,&pitch,1,i);
	}

}
#endif
