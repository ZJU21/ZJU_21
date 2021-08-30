/************************************
 * 2021-08-30 by csr
 ************************************/

#ifndef _VEHICLE_H
#define _VEHICLE_H
#include "SunConfig.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <FlexiTimer2.h>  //定时中断
#include "message.h"
#include "sound.h"

const float grid_size=300;

typedef enum
{
	PAUSE, 
	LEFTWARD, 
	FORWARD, 
	RIGHTWARD, 
	BACKWARD, 
	FINDLINEY,
	START, 
	END
} COMMAND_TYPE;

struct COMMAND
{
	COMMAND_TYPE typ;
	float len;
};

class Vehicle
{
	//新建通信实例
	Message msg;
	//新建语音播报实例
	Sound sound;
	//新建小车陀螺仪实例
	MPU6050 mpu;
	//新建小车灰度传感器实例
	Grayscale GraySensors; 
	//新建小车底盘运动学实例
	Kinematics kinematics=Kinematics(MAX_RPM, WHEEL_DIAMETER, 
			FR_WHEELS_DISTANCE,
			LR_WHEELS_DISTANCE);
	//新建小车里程计实例
	WheelOdometry botOdometry=WheelOdometry(&kinematics);
	//新建小车电机实例
	A4950MotorShield motors;
	//新建小车编码器实例
	Encoder ENC[4] = {Encoder(ENCODER_A, DIRECTION_A), 
		Encoder(ENCODER_B, DIRECTION_B),
		Encoder(ENCODER_C, DIRECTION_C), 
		Encoder(ENCODER_D, DIRECTION_D)};
	float Kp = 10, Ki = 0.1, Kd = 0;
	double outPWM[4] = {0};
	long newPulses[4] = {0};
	float targetPulses[4] = {0};
	//新建小车速度PID实例
	PID VeloPID[4] = {PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), 
		PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
		PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), 
		PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

	//运行状态标记
	COMMAND *now_command;
	float start_x, previous_x = 0, current_x = 0;   // mm
	float start_y, previous_y = 0, current_y = 0;   // mm
	float linear_vel_x = 0;   // m/s
	float linear_vel_y = 0;   // m/s
	float angular_vel_z = 0;  // rad/s
	unsigned long previousMillis = 0, currentMillis = 0;  // ms

	// MPU control/status vars
	bool dmpReady;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	Quaternion q;   // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	//运动学输出量的结构体
	Kinematics::output pluses;
	//uart输出量的结构体
	Grayscale::strOutput GraySensorsUartIoOutput;
	public:
	bool is_dmp_ready();
	void update_odometry();
	void get_motor_speed();
	void set_motor_speed();
	void start();
	void init(float, float, COMMAND*);
	void get_current_status();
	void redress_odometry();
	void run();
	void update_goal_speed();
	private:
	void next_command();
};

void Vehicle::update_odometry()
{
	botOdometry.getPositon_mm(newPulses[0], newPulses[1], newPulses[2], newPulses[3], ypr[0]);
}

void Vehicle::get_motor_speed()
{
	targetPulses[0] = pluses.motor1;
	targetPulses[1] = pluses.motor2;
	targetPulses[2] = pluses.motor3;
	targetPulses[3] = pluses.motor4;
#ifdef PINS_REVERSE
	newPulses[0] = -ENC[0].read();  // A
	newPulses[1] = ENC[1].read();   // B
	newPulses[2] = -ENC[2].read();  // C
	newPulses[3] = ENC[3].read();   // D
#else
	newPulses[0] = ENC[0].read();   // A
	newPulses[1] = -ENC[1].read();  // B
	newPulses[2] = ENC[2].read();   // C
	newPulses[3] = -ENC[3].read();  // D
#endif
}

void Vehicle::set_motor_speed()
{
	for (int i = 0; i < WHEEL_NUM; i++) {
		outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
		ENC[i].write(0);  //复位
	}
	motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void Vehicle::start()
{
	msg.init();
#ifdef vehicle1
	msg.init_server();
	msg.start_server();
#endif
#ifdef vehicle2
	sound.init();
	msg.init_cilent1();
	sound.send(_WIFI);
	msg.init_cilent2();
	sound.send(_TCP);
	sound.send(_READY);
	msg.start_cilent();
	sound.send(_START);
#endif
}

void Vehicle::init(float x,float y,COMMAND *commands)
{
	motors.init();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	//IMU_Zero
	int TheOffsets[6] = {0};
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);

	uint8_t AOffsetRegister = (mpu.getDeviceID() < 0x38 )? 0x06: 0x77;
	if(AOffsetRegister == 0x06) I2Cdev::readWords(0x68, AOffsetRegister, 3, (int *)TheOffsets);
	else 
	{
		I2Cdev::readWords(0x68, AOffsetRegister, 1, (int *)TheOffsets);
		I2Cdev::readWords(0x68, AOffsetRegister+3, 1, (int *)TheOffsets+1);
		I2Cdev::readWords(0x68, AOffsetRegister+6, 1, (int *)TheOffsets+2);
	}
	//  A_OFFSET_H_READ_A_OFFS(Data);
	mpu.setXAccelOffset(TheOffsets[0]);
	mpu.setYAccelOffset(TheOffsets[1]);
	mpu.setZAccelOffset(TheOffsets[2]);
	I2Cdev::readWords(0x68, 0x13, 3, (int *)TheOffsets+3);
	//  XG_OFFSET_H_READ_OFFS_USR(Data);
	mpu.setXGyroOffset(TheOffsets[3]);
	mpu.setYGyroOffset(TheOffsets[4]);
	mpu.setZGyroOffset(TheOffsets[5]);

	if (devStatus == 0) 
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		// mpu.CalibrateAccel(6);
		// mpu.CalibrateGyro(6);
		// mpu.PrintActiveOffsets();
		mpu.setDMPEnabled(true);
		dmpReady = true;
		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}

	start_x=x;
	start_y=y;
	now_command=commands;
}

void Vehicle::get_current_status()
{
	currentMillis = millis();   // store the current time
	GraySensorsUartIoOutput = GraySensors.readUart();  //读取串口数字量数据
	current_x = botOdometry.botPosition.position_x;
	current_y = botOdometry.botPosition.position_y;
}

void Vehicle::redress_odometry()
{
	if (GraySensorsUartIoOutput.ioDigital == 127)
		botOdometry.botPosition.position_x = round((botOdometry.botPosition.position_x - start_x) / grid_size) * grid_size + start_x;
	if (GraySensorsUartIoOutput.ioDigital == 8)
		botOdometry.botPosition.position_y = round((botOdometry.botPosition.position_y - start_y) / grid_size) * grid_size + start_y;
}

void Vehicle::run()
{
	switch (now_command->typ)
	{
		case PAUSE:           //停止
#ifndef CloseBrake
			motors.motorsBrake();
#endif
			linear_vel_x = 0;   // m/s
			linear_vel_y = 0;   // m/s
			if (currentMillis - previousMillis >= now_command->len) 
				next_command();
			break;
		case LEFTWARD:             //左进
			linear_vel_x = 0;   // m/s
			linear_vel_y = 0.2;   // m/s
			if (current_y - previous_y >= now_command->len*grid_size) 
				next_command();
			break;
		case RIGHTWARD:             //右进
			linear_vel_x = 0;   // m/s
			linear_vel_y = -0.2;   // m/s
			if (current_y - previous_y <= -now_command->len*grid_size) 
				next_command();
			break;
		case FORWARD:          //前进
			linear_vel_x = 0.2;  // m/s
			if (GraySensorsUartIoOutput.ioCount) 
				linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;		// m/s
#ifdef TestMode1
			linear_vel_y=0;
#endif
			if (current_x - previous_x >= now_command->len*grid_size)
				next_command();
			break;
		case BACKWARD:          //后退
			linear_vel_x = -0.2;  // m/s
			if (GraySensorsUartIoOutput.ioCount) 
				linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;		// m/s
#ifdef TestMode1
			linear_vel_y=0;
#endif
			if (current_x - previous_x <= -now_command->len*grid_size)
				next_command();
			break;
		case START:             //开始
#ifndef CloseBrake
			motors.motorsBrake();
#endif
			linear_vel_x = 0;   // m/s
			linear_vel_y = 0;   // m/s
			angular_vel_z = 0;  // rad/s
			next_command();
			break; 
		case END:             //结束
#ifndef CloseBrake
			motors.motorsBrake();
#endif
			linear_vel_x = 0;   // m/s
			linear_vel_y = 0;   // m/s
			angular_vel_z = 0;  // rad/s
			break;
		case FINDLINEY:  // y方向和线对齐  
			linear_vel_x = 0;  // m/s
			if (GraySensorsUartIoOutput.ioCount) 
				linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;   // m/s
#ifdef TestMode1
			linear_vel_y=0;
#endif
			if (linear_vel_y == 0)
				next_command();
			break;
		default:              //停止
			linear_vel_x = 0;   // m/s
			linear_vel_y = 0;   // m/s
			angular_vel_z = 0;  // rad/s
			break;
	}
}

void Vehicle::next_command()
{
	now_command++;
	previous_x=current_x;
	previous_y=current_y;
	previousMillis=currentMillis;
}

void Vehicle::update_goal_speed()
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
	{ 
		// Get the Latest packet 
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		angular_vel_z=ypr[0]*5;
	}
	pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

bool Vehicle::is_dmp_ready()
{
	return dmpReady;
}

#endif

