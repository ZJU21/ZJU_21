//巡线程序 
//20210825

#include "SunConfig.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <FlexiTimer2.h>  //定时中断

//新建小车陀螺仪实例
MPU6050 mpu;
//新建小车灰度传感器实例
Grayscale GraySensors; 
//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, 
                        FR_WHEELS_DISTANCE,
                        LR_WHEELS_DISTANCE);
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

float linear_vel_x = 0;   // m/s
float linear_vel_y = 0;   // m/s
float angular_vel_z = 0;  // m/s
unsigned long previousMillis = 0;   // will store last time run
const long period = 5000;   // period at which to run in ms

//运行状态标记
enum CARDIRECTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD };
enum CARDIRECTION direction = PAUSE;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;   // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Kinematics::output pluses;
//uart输出量的结构体
Grayscale::strOutput GraySensorsUartIoOutput;

void control()
{
    sei();  //全局中断开启

    targetPulses[0] = pluses.motor1;
    targetPulses[1] = pluses.motor2;
    targetPulses[2] = pluses.motor3;
    targetPulses[3] = pluses.motor4;

    //获取电机速度
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

    for (int i = 0; i < WHEEL_NUM; i++) {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
    ENC[i].write(0);  //复位
    }
    //设置电机速度
    motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup() 
{
    Serial.begin(115200);
    motors.init();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    //先运行IMU_ZERO
    mpu.setXAccelOffset(-2140);
    mpu.setYAccelOffset(-774);
    mpu.setZAccelOffset(1248);
    mpu.setXGyroOffset(46);
    mpu.setYGyroOffset(28);
    mpu.setZGyroOffset(-18);

    if (devStatus == 0) 
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    FlexiTimer2::set(TIMER_PERIOD, control);  //定时中断函数，TIMER_PERIOD为宏定义
    FlexiTimer2::start();
}

void loop() 
{
    unsigned long currentMillis = millis();   // store the current time
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { 
        // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angular_vel_z=ypr[0]*5;
    }
    GraySensorsUartIoOutput = GraySensors.readUart();  //读取串口数字量数据
    //使用有限状态机方式
    // simulated required velocities
    // PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD
    switch (direction) 
    {
    case PAUSE:           //停止
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0;   // m/s
        //angular_vel_z = 0;  // rad/s
        //使用millis函数进行定时控制，代替delay函数
        if (currentMillis - previousMillis >= period) 
        {
            previousMillis = currentMillis;
            direction = FORWARD;
        }
        break;
    case FORWARD:          //前进
        linear_vel_x = 0.2;  // m/s
        if (GraySensorsUartIoOutput.ioCount) 
        {
            linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;
        }                   // m/s
        //angular_vel_z = 0;  // rad/s
        if (currentMillis - previousMillis >= (2 * period)) 
        {
            previousMillis = currentMillis;
            direction = BACKWARD;
        }
        break;
    case BACKWARD:          //后退
        linear_vel_x = -0.2;  // m/s
        if (GraySensorsUartIoOutput.ioCount) 
        {
            linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset; // m/s
        }                  
        //angular_vel_z = 0;  // rad/s
        if (currentMillis - previousMillis >= (2 * period)) 
        {
            previousMillis = currentMillis;
            direction = PAUSE;
        }
        break;
    default:              //停止
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0;   // m/s
        //angular_vel_z = 0;  // rad/s
        if (currentMillis - previousMillis >= period) 
        {
            previousMillis = currentMillis;
            direction = PAUSE;
        }
        break;
    }
    pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}
