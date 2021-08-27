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
//新建小车里程计实例
WheelOdometry botOdometry(&kinematics);
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
float angular_vel_z = 0;  // rad/s
unsigned long previousMillis = 0;   // will store last time run
const long period = 3000;   // period at which to run in ms

//运行状态标记
//enum CARDIRECTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD };
//enum CARDIRECTION direction = PAUSE;
int direction = 0;

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

//运动学输出量的结构体
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

    //里程计更新
    botOdometry.getPositon_mm(newPulses[0], newPulses[1], newPulses[2],
                            newPulses[3], ypr[0]);

    for (int i = 0; i < WHEEL_NUM; i++) {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
    ENC[i].write(0);  //复位
    }
    //设置电机速度
    motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup() 
{
    //一键启动
    pinMode(42, INPUT_PULLUP);
    while(digitalRead(42)){}

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

    FlexiTimer2::set(TIMER_PERIOD, control);  //定时中断函数，TIMER_PERIOD为宏定义
    FlexiTimer2::start();
}

void loop() 
{
    if (!dmpReady) return;
    unsigned long currentMillis = millis();   // store the current time
    GraySensorsUartIoOutput = GraySensors.readUart();  //读取串口数字量数据

    //灰度传感器修正里程计
    if(GraySensorsUartIoOutput.offset == 0)
        botOdometry.botPosition.position_y = round((botOdometry.botPosition.position_y - 150) / 300) * 300 + 150;
    if(GraySensorsUartIoOutput.ioCount == 7)
        botOdometry.botPosition.position_x = round((botOdometry.botPosition.position_x) / 300) * 300;
    
    //使用有限状态机方式
    switch (direction) 
    {
    case 0:           //停止
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0;   // m/s
        //angular_vel_z = 0;  // rad/s
        //使用millis函数进行定时控制，代替delay函数
        if (currentMillis - previousMillis >= period) 
        {
            previousMillis = currentMillis;
            direction++;
        }
        break;
    case 1:             //左进
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0.2;   // m/s
        if (botOdometry.botPosition.position_y >= 450) 
        {
            previousMillis = currentMillis;
            direction++;
        }
        break;
    case 2:          //前进
        linear_vel_x = 0.2;  // m/s
        if (GraySensorsUartIoOutput.ioCount) 
        {
            linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;
        }                   // m/s
        //angular_vel_z = 0;  // rad/s
        if (botOdometry.botPosition.position_x >= 600) 
        {
            previousMillis = currentMillis;
            direction++;
        }
        break;
    case 3:           //停止
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0;   // m/s
        //angular_vel_z = 0;  // rad/s
        //使用millis函数进行定时控制，代替delay函数
        if (currentMillis - previousMillis >= period) 
        {
            previousMillis = currentMillis;
            direction++;
        }
        break;
    case 4:          //前进
        linear_vel_x = 0.2;  // m/s
        if (GraySensorsUartIoOutput.ioCount) 
        {
            linear_vel_y = -0.005 * GraySensorsUartIoOutput.offset;
        }                   // m/s
        //angular_vel_z = 0;  // rad/s
        if (botOdometry.botPosition.position_x >= 1350) 
        {
            previousMillis = currentMillis;
            direction++;
        }
        break;
    default:              //停止
        linear_vel_x = 0;   // m/s
        linear_vel_y = 0;   // m/s
        //angular_vel_z = 0;  // rad/s
        if (currentMillis - previousMillis >= period) 
        {
            previousMillis = currentMillis;
        }
        break;
    }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { 
        // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angular_vel_z=ypr[0]*5;
    }

    pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

    Serial.print(botOdometry.botPosition.position_x);
    Serial.print(" ");
    Serial.println(botOdometry.botPosition.position_y);
}
