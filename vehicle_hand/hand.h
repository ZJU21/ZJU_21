#ifndef _HAND_H
#define _HAND_H

#include "FashionStar_Arm5DoF.h"
#include "camera.h"

// 调试串口的配置

//#define DEBUG_SERIAL Serial
//#define DEBUG_SERIAL_BAUDRATE 115200

class Hand
{
	FSARM_ARM5DoF arm; //机械臂对象
	Camera cam;	//新建相机实例
	private:
	void move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num,float push_num);
	void print_theta(FSARM_JOINTS_STATE_T thetas);
	void print_toolPosi(FSARM_POINT3D_T toolPosi, float pitch);
	void run_action_group();
	void trail_move_action(FSARM_JOINTS_STATE_T *thetas,int command_num);
	void tiny_move_action(FSARM_POINT3D_T *toolPosi,float *pitch,int command_num);
	void Forward_and_backward_forward(float forward_push_num);
	public:
	int scanQRcode();
	bool try_catchA();
	bool try_catchB();
	void init();
	void catchA();
	void catchB();  // TODO
};

int Hand::scanQRcode()
{
	return cam.get_QRcode();
}

bool Hand::try_catchA()
{
	int x=cam.get_Aposition().size;
	Serial.print("size=");
	Serial.println(x);
	return x>1000;
}

bool Hand::try_catchB()
{
	return cam.get_Bposition().size>1000;
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
	FSARM_JOINTS_STATE_T thetas;    // 关节角度 
	FSARM_POINT3D_T toolPosi;       // 末端的位置  
	float pitch;                    // 末端倾角

#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口
#endif

	arm.init(); //机械臂初始化
	cam.init();

	//arm.home();
	arm.queryAngle(&thetas);    // 查询舵机的角度
#ifdef DEBUG_SERIAL
	DEBUG_SERIAL.println("home pos");
#endif
	print_theta(thetas);
	arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学           
	print_toolPosi(toolPosi,pitch);     // 打印正向运动学的结果

	//arm.ready();
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


void Hand::catchA()
{
	Serial.println("!!!!!");
	FSARM_JOINTS_STATE_T thetas;    // 关节角度 
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

}
/*
{
	move_action(&toolPosi,&pitch,1,6);
	move_action(&toolPosi,&pitch,5,7);
	delay(100);
	arm.gripperOpen();
	arm.queryAngle(&thetas);    // 查询舵机的角度
	Position pos;
	for (pos=cam.get_Aposition();pos.size<200;pos=cam.get_Aposition());
	for (;abs(pos.x)>10;pos=cam.get_Aposition())
		if (pos.size>100)
		{
			if (pos.x>0)
				trail_move_action(&thetas,2);
			if (pos.x<0)
				trail_move_action(&thetas,3);
			//          if (pos.size<1800)
			//              trail_move_action(&thetas,1);
			//          if (pos.size>2000)
			//              trail_move_action(&thetas,6);
			//if (pos.y>0)
			//    move_action(&toolPosi,&pitch,4,2);
			//if (pos.y<0)
			//    move_action(&toolPosi,&pitch,5,2);
		}

	move_action(&toolPosi,&pitch,1,9);
	arm.gripperClose();
	move_action(&toolPosi,&pitch,4,8);
	move_action(&toolPosi,&pitch,6,9);
	move_action(&toolPosi,&pitch,2,9);
	//arm.gripperOpen();
	//arm.gripperClose();
	//命令编号 1：前进 2：左移 3：右移 4：上移 5：下移 6：后退
	delay(9000);
	thetas.theta1 = 0;
	thetas.theta2 = -90;
	thetas.theta3 = 0;
	thetas.theta4 = 0;

	// 机械臂运行
	arm.setAngle(thetas);       // 设置舵机旋转到特定的角度
	arm.wait();                 // 等待舵机旋转到目标位置
}
*/


void Hand::catchB()
{
	//TODO
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
