#include "SunA4950Motors.h"

byte A4950MotorShield::motorDeadZone = 0;

A4950MotorShield::A4950MotorShield() {  //默认引脚
  _M1PWMA = AIN1;                       // AIN1
  _M1PWMB = AIN2;
  _M2PWMA = BIN1;  // B
  _M2PWMB = BIN2;
  _M3PWMA = CIN1;  // C
  _M3PWMB = CIN2;
  _M4PWMA = DIN1;  // D
  _M4PWMB = DIN2;
}

A4950MotorShield::A4950MotorShield(byte M1PWMA, byte M1PWMB, byte M2PWMA,
                                   byte M2PWMB, byte M3PWMA, byte M3PWMB,
                                   byte M4PWMA, byte M4PWMB) {
  _M1PWMA = M1PWMA;
  _M1PWMB = M1PWMB;
  _M2PWMA = M2PWMA;
  _M2PWMB = M2PWMB;
  _M3PWMA = M3PWMA;
  _M3PWMB = M3PWMB;
  _M4PWMA = M4PWMA;
  _M4PWMB = M4PWMB;
}
//三电机委托构造函数
A4950MotorShield::A4950MotorShield(byte M1PWMA, byte M1PWMB, byte M2PWMA,
                                   byte M2PWMB, byte M3PWMA, byte M3PWMB)
    : A4950MotorShield(M1PWMA, M1PWMB, M2PWMA, M2PWMB, M3PWMA, M3PWMB, DIN1,
                       DIN2) {}
void A4950MotorShield::init() {
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  pinMode(_M1PWMA, OUTPUT);
  pinMode(_M1PWMB, OUTPUT);
  pinMode(_M2PWMA, OUTPUT);
  pinMode(_M2PWMB, OUTPUT);
  pinMode(_M3PWMA, OUTPUT);
  pinMode(_M3PWMB, OUTPUT);
  pinMode(_M4PWMA, OUTPUT);
  pinMode(_M4PWMB, OUTPUT);
  digitalWrite(_M1PWMA, LOW);
  digitalWrite(_M1PWMB, LOW);
  digitalWrite(_M2PWMA, LOW);
  digitalWrite(_M2PWMB, LOW);
  digitalWrite(_M3PWMA, LOW);
  digitalWrite(_M3PWMB, LOW);
  digitalWrite(_M4PWMA, LOW);
  digitalWrite(_M4PWMB, LOW);
  A4950MotorShield::motorDeadZone = 0;
}

void A4950MotorShield::init(bool HForLF, byte mtrDeadZone) {
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  pinMode(_M1PWMA, OUTPUT);
  pinMode(_M1PWMB, OUTPUT);
  pinMode(_M2PWMA, OUTPUT);
  pinMode(_M2PWMB, OUTPUT);
  pinMode(_M3PWMA, OUTPUT);
  pinMode(_M3PWMB, OUTPUT);
  pinMode(_M4PWMA, OUTPUT);
  pinMode(_M4PWMB, OUTPUT);
  digitalWrite(_M1PWMA, LOW);
  digitalWrite(_M1PWMB, LOW);
  digitalWrite(_M2PWMA, LOW);
  digitalWrite(_M2PWMB, LOW);
  digitalWrite(_M3PWMA, LOW);
  digitalWrite(_M3PWMB, LOW);
  digitalWrite(_M4PWMA, LOW);
  digitalWrite(_M4PWMB, LOW);
  A4950MotorShield::motorDeadZone = mtrDeadZone;

  if (HForLF) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

    int fff = 1;
    TCCR1B = (TCCR1B & 0xF8) |
             fff;  //调整计数器分频，频率调高至31.374KHZ 0xF8 11111000
    TCCR3B = (TCCR3B & 0xF8) | fff;  //调整计数器分频，频率调高至31.374KHZ
    TCCR4B = (TCCR4B & 0xF8) | fff;  //调整计数器分频，频率调高至31.374KHZ
    TCCR5B = (TCCR5B & 0xF8) | fff;  //调整计数器分频，频率调高至31.374KHZ

#endif
  }
}

// speed should be a number between -255 and 255
void A4950MotorShield::setM1Speed(int speed) {
  speed = constrain(speed, -255 + motorDeadZone, 255 - motorDeadZone);

  if (speed > 0)
    analogWrite(_M1PWMB, speed + motorDeadZone),
        analogWrite(_M1PWMA,
                    0);  ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (speed == 0)
    analogWrite(_M1PWMB, 0), analogWrite(_M1PWMA, 0);
  else if (speed < 0)
    analogWrite(_M1PWMA, -speed + motorDeadZone),
        analogWrite(
            _M1PWMB,
            0);  ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

// speed should be a number between -255 and 255
void A4950MotorShield::setM2Speed(int speed) {
  speed = constrain(speed, -255 + motorDeadZone, 255 - motorDeadZone);

  if (speed > 0)
    analogWrite(_M2PWMA, speed + motorDeadZone),
        analogWrite(_M2PWMB,
                    0);  ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (speed == 0)
    analogWrite(_M2PWMB, 0), analogWrite(_M2PWMA, 0);
  else if (speed < 0)
    analogWrite(_M2PWMB, -speed + motorDeadZone),
        analogWrite(
            _M2PWMA,
            0);  ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

// speed should be a number between -255 and 255
void A4950MotorShield::setM3Speed(int speed) {
  speed = constrain(speed, -255 + motorDeadZone, 255 - motorDeadZone);

  if (speed > 0)
    analogWrite(_M3PWMB, speed + motorDeadZone),
        analogWrite(_M3PWMA,
                    0);  ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (speed == 0)
    analogWrite(_M3PWMA, 0), analogWrite(_M3PWMB, 0);
  else if (speed < 0)
    analogWrite(_M3PWMA, -speed + motorDeadZone),
        analogWrite(
            _M3PWMB,
            0);  ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

// speed should be a number between -255 and 255
void A4950MotorShield::setM4Speed(int speed) {
  speed = constrain(speed, -255 + motorDeadZone, 255 - motorDeadZone);

  if (speed > 0)
    analogWrite(_M4PWMA, speed + motorDeadZone),
        analogWrite(_M4PWMB,
                    0);  ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (speed == 0)
    analogWrite(_M4PWMA, 0), analogWrite(_M4PWMB, 0);
  else if (speed < 0)
    analogWrite(_M4PWMB, -speed + motorDeadZone),
        analogWrite(
            _M4PWMA,
            0);  ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

// set speed for both motors
// speed should be a number between -255 and 255
void A4950MotorShield::setSpeeds(int LeftSpeed, int RightSpeed) {
  setM1Speed(LeftSpeed);
  setM2Speed(LeftSpeed);
  setM3Speed(RightSpeed);
  setM4Speed(RightSpeed);
}

void A4950MotorShield::setSpeeds(int m1Speed, int m2Speed, int m3Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
}

void A4950MotorShield::setSpeeds(int m1Speed, int m2Speed, int m3Speed,
                                 int m4Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);
}

void A4950MotorShield::motorsBrake() {
  digitalWrite(_M1PWMA, HIGH);
  digitalWrite(_M1PWMB, HIGH);
  digitalWrite(_M2PWMA, HIGH);
  digitalWrite(_M2PWMB, HIGH);
  digitalWrite(_M3PWMA, HIGH);
  digitalWrite(_M3PWMB, HIGH);
  digitalWrite(_M4PWMA, HIGH);
  digitalWrite(_M4PWMB, HIGH);
}