#include <FlexiTimer2.h>  //定时中断
// 请先安装FlexiTimer2库,https://playground.arduino.cc/Main/FlexiTimer2/

#include "SunConfig.h"       //包含配置库


//******************创建4个编码器对象***************************//
//良好性能，只有第一个引脚具备外部中断能力
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
long newPulses[4] = {0, 0, 0, 0};  //四个车轮的定时中断编码器四倍频速度

//*****************创建4个速度PID控制对象***************************//
/*PID(float min_val, float max_val, float kp, float ki, float kd)
 * float min_val = min output PID value
 * float max_val = max output PID value
 * float kp = PID - P constant PID控制的比例、积分、微分系数
 * float ki = PID - I constant
 * float di = PID - D constant
 * Input	(double)输入参数feedbackVel，待控制的量
 * Output	(double)输出参数outPWM，指经过PID控制系统的输出量
 * Setpoint	(double)目标值targetVel，希望达到的数值
 */
float targetPulses[4] = {50, 50, 50, 50};
float feedbackVel[4] = {0, 0, 0, 0};
float Kp = 10, Ki = 0.1, Kd = 0;
double outPWM[4] = {0, 0, 0, 0};
PID VeloPID[4] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
A4950MotorShield motors;

//*****************创建1个电池电压对象***************************//
Battery battery3S;

//*****************10ms定时中断函数，核心函数***************************//
void control() {
  sei();  //全局中断开启

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
  // pid控制器
  for (int i = 0; i < WHEEL_NUM; i++) {
    feedbackVel[i] = (float)newPulses[i];
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackVel[i]);
    ENC[i].write(0);  //复位电机速度为0
  }

  //电池电压正常的情况下启动电机
  if (battery3S.is_Volt_Low() == false) {
    motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
  }
  static int print_Count;
  if (++print_Count >= 100)  //打印控制，控制周期1000ms
  {
    Serial.print(millis());  //显示
    Serial.print(",");
    Serial.println(battery3S.read());  //显示
    for (int i = 0; i < WHEEL_NUM; i++) {
      Serial.print(i);  //显示
      Serial.print(":");
      Serial.print(targetPulses[i]);  //显示
      Serial.print(",");
      Serial.print(feedbackVel[i]);  //显示
      Serial.print(",");
      Serial.println(outPWM[i]);  //显示
      print_Count = 0;
    }
  }
}
void setup() {
  motors.init();
  delay(100);                               //延时等待初始化完成
  FlexiTimer2::set(TIMER_PERIOD, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();                     //中断使能
  delay(100);                               //延时等待初始化完成
  Serial.begin(BAUDRATE);
  Serial.println("Sunnybot PID Test:");
}

void loop() {
  Serial.println(battery3S.read());
  delay(1000);
}