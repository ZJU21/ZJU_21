//任务要求1：从出发区出发线前出发，巡线一圈，停在返回区
//任务要求2：从返回区出发，巡线一圈，停在出发区
//******************电机驱动规格***************************//
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，三路循迹传感器
#define LeftMotorAIN1 6  // A电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2 5  // A电机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1  8  // B电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2  7  // B电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚
//********************循迹传感器***************************//
#define LeftGrayscale A1  //左循迹传感器引脚，根据实际接线修改引脚
#define MiddleGrayscale A2  //中循迹传感器引脚，根据实际接线修改引脚
#define RightGrayscale A3  //右循迹传感器引脚，根据实际接线修改引脚
//********************颜色定义***************************//
#define BLACK HIGH  //循迹传感器检测到黑色为“HIGH”,灯灭（黑）
#define WHITE LOW  //循迹传感器检测到白色为“LOW”，灯亮（白）

//=======================智能小车的基本动作=========================
void moveForward()  // 前进
{
  // 左电机前进，PWM比例，0~255调速，左右轮差异略增减，电机空载死区0-30
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机前进，PWM比例，0~255调速，左右轮差异略增减
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void moveBackward()  //后退
{
  //左轮后退，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);
  //右轮后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);
}

void turnLeft()  //左转(左轮不动，右轮前进)
{
  //右电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 0);
  //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void spinLeft()  //原地左转(左轮后退，右轮前进)
{
  //左轮后退PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);
  //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void turnRight()  //右转(右轮不动，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 0);
}

void spinRight()  //原地右转(右轮后退，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);
}

void brake()  //急刹车，停车
{
  digitalWrite(RightMotorBIN1, HIGH);
  digitalWrite(RightMotorBIN2, HIGH);
  digitalWrite(LeftMotorAIN1, HIGH);
  digitalWrite(LeftMotorAIN2, HIGH);
}
void stop()  //普通刹车，停车，惯性滑行
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

unsigned long lastTime;//时间比较值

//======================配置程序====================================

void setup() {
  pinMode(LeftMotorAIN1, OUTPUT);  //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);
  pinMode(LeftGrayscale, INPUT);    //定义左循迹传感器为输入
  pinMode(MiddleGrayscale, INPUT);  //定义中循迹传感器为输入
  pinMode(RightGrayscale, INPUT);   //定义右循迹传感器为输入
  // Serial.begin(9600);
  // moveForward();
  // delay(500);
  lastTime = millis();  //记录程序开始时间，单位ms
}

void loop() {
  bool LeftGrayscaleValue = digitalRead(LeftGrayscale);
  bool MiddleGrayscaleValue = digitalRead(MiddleGrayscale);
  bool RightGrayscaleValue = digitalRead(RightGrayscale);
  // Serial.print("LeftGrayscaleValue:");
  // Serial.print(LeftGrayscaleValue);
  // Serial.print(",");
  // Serial.print("MiddleGrayscale:");
  // Serial.println(MiddleGrayscaleValue);
  // Serial.print(",");
  // Serial.print("RightGrayscaleValue:");
  // Serial.println(RightGrayscaleValue);
  if (millis() - lastTime < 25000) {
    if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == BLACK &&
        RightGrayscaleValue == BLACK)  // 111都是黑色，十字路口，前进
    {
      moveForward();
    }
    if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == BLACK &&
        RightGrayscaleValue == WHITE)  // 010 正常巡线 前进
    {
      moveForward();
      
    }
    if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == BLACK &&
        RightGrayscaleValue == WHITE)  // 110偏右，左转纠正？
    {
      turnLeft();
    }
    if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == WHITE &&
        RightGrayscaleValue == WHITE)  // 100偏右，左转纠正？
    {
       //do something 
    }
    if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == BLACK &&
        RightGrayscaleValue == BLACK)  // 011偏左，右转纠正？
    {
       //do something 
    }
    if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == WHITE &&
        RightGrayscaleValue == BLACK)  // 001偏左，右转纠正？
    {
       //do something 
    }
    if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == WHITE &&
        RightGrayscaleValue == WHITE)  // 000，后退？
    {
      //do something 
    }
    if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == WHITE &&
        RightGrayscaleValue == BLACK)  // 101，后退？
    {
      moveBackward();
    }
  }
  else brake();
}
