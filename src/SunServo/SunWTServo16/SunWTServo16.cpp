#include "SunWTServo16.h"


SunWTServo16::SunWTServo16(HardwareSerial& uartPort) {
  //初始化运行中标识为真
  _isRunning = true;
  _uartPort = &uartPort;
  (*_uartPort).begin(9600);  //串口初始化
}

SunWTServo16::SunWTServo16() :_uartPort(&WTServo16_DEFAULT_PORT),_isRunning(true){
  //初始化运行中标识为真
  (*_uartPort).begin(9600);  //串口初始化
}

/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Speed:转动速度
                    舵机ID取值:0<=舵机ID<=15,Speed取值: Speed > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void SunWTServo16::moveServo(uint8_t servoID, uint16_t Position,
                             uint16_t Speed) {
								 //数据格式 0xff CMD CH DataL DataH
  uint8_t buf[15];
  if (servoID > 15 || !(Speed > 0)) {
    return;
  }
  //y=kx+b
  //y= (x - 0     ) * (500      - 2500)   / (180    - 0)      + 500;
//  y= (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  Position = Position * 100 / 9 + 500;//转为脉冲宽度500us~2500us
  buf[0] = FRAME_HEADER;
  buf[1] = CMD_SERVO_SPEED;
  buf[2] = servoID;
  buf[3] = lowByte(Speed);//9°/s
  buf[4] = highByte(Speed);
  buf[5] = FRAME_HEADER;
  buf[6] = CMD_SERVO_POSITION;
  buf[7] = servoID;
  buf[8] = lowByte(Position);//舵机的控制脉宽是500us~2500us，对应的角度0~180°。
  buf[9] = highByte(Position);

  (*_uartPort).write(buf, 10);
}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机转动
 * Parameters:   Num:舵机个数,Speed:转动时间,...:舵机ID,转动角，舵机ID,转动角度
 *如此类推 Return:       无返回 Others:
 **********************************************************************************/
void SunWTServo16::moveServos(uint8_t Num, uint16_t Speed, ...) {
  uint8_t buf[15];
  va_list arg_ptr;
  va_start(arg_ptr, Speed);  //取得可变参数首地址
  if (Num < 1 || Num > 16 || (!(Speed > 0)) || arg_ptr == NULL) {
    return;  //舵机数不能为零和大与16，时间不能为零，可变参数不能为空
  }
  for (uint8_t i = 0; i < Num;
       i++) {  //从可变参数中取得并循环填充舵机ID和对应目标位置
    uint8_t index = 0;
    buf[index++] = FRAME_HEADER;
    buf[index++] = CMD_SERVO_SPEED;
    uint16_t tmp = va_arg(arg_ptr, uint16_t);  //可参数中取得舵机ID
    buf[index++] =
        lowByte(tmp);  //貌似avrgcc中可变参数整形都是十六位，再取其低八位
    buf[index++] = lowByte(Speed);   //取得速度的低八位
    buf[index++] = highByte(Speed);  //取得速度的高八位
    buf[index++] = FRAME_HEADER;
    buf[index++] = CMD_SERVO_POSITION;
    buf[index++] = lowByte(tmp);
    uint16_t pos = va_arg(arg_ptr, uint16_t);  //可变参数中取得对应目标位置
    pos = pos * 100 / 9 + 500;
    buf[index++] = lowByte(pos);   //填充目标位置低八位
    buf[index++] = highByte(pos);  //填充目标位置高八位
     (*_uartPort).write(buf, 10);            //发送帧
  }
  va_end(arg_ptr);  //置空arg_ptr
}

/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号
 * Return:       无返回
 **********************************************************************************/
void SunWTServo16::runActionGroup(uint8_t NumOfAction) {
  uint8_t buf[10];
  buf[0] = FRAME_HEADER;  //填充帧头
  buf[1] = CMD_ACTION_GROUP_RUN;
  buf[2] = 0x00;
  buf[3] = lowByte(NumOfAction);   //取得动作组号的低八位
  buf[4] = highByte(NumOfAction);  //取得动作组号的高八位
  (*_uartPort).write(buf, 5);               //发送数据帧
}

/*********************************************************************************
 * Function:  stopServo
 * Description： 急停
 * Parameters:   无
 * Return:       无返回
 **********************************************************************************/
void SunWTServo16::stopServo(void) {
  if (!_isRunning) {
    return;
  }
  _isRunning = !_isRunning;
  uint8_t buf[10];
  buf[0] = FRAME_HEADER;  //填充帧头
  buf[1] = CMD_STOP_REFRESH;
  buf[2] = 0x00;
  buf[3] = 0x01;
  buf[4] = 0x00;
   (*_uartPort).write(buf, 5);  //发送数据帧
}

/*********************************************************************************
 * Function:  refreshServo
 * Description： 恢复
 * Parameters:   无
 * Return:       无返回
 **********************************************************************************/
void SunWTServo16::refreshServo(void) {
  if (_isRunning) {
    return;
  }
  _isRunning = !_isRunning;
  uint8_t buf[10];
  buf[0] = FRAME_HEADER;  //填充帧头
  buf[1] = CMD_STOP_REFRESH;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
   (*_uartPort).write(buf, 5);  //发送数据帧
}
