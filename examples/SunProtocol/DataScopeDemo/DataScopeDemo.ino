
#include "SunConfig.h"      //这是PC端上位机的库文件
DATASCOPE data;
unsigned char Send_Count;  //上位机相关变量
float a;
int show1,show2,show3,show4;
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(show1, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(show2, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(show3, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(show4, 4);//显示第四个数据，电池电压，单位V
  Send_Count = data.DataScope_Data_Generate(4); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}
/**************************************************************************
函数功能：初始化 相
入口参数：无
返回  值：无
**************************************************************************/
void setup() {

  Serial.begin(128000);       //开启串口，设置波特率为 128000
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() {
      a+=0.1;
    if(a>3.14)  a=-3.14; 
    show1=500*sin(a);
    show2=500* tan(a);
    show3=500*cos(a);
    show4=100*a;
    DataScope(); //使用上位机时，波特率是128000
}

