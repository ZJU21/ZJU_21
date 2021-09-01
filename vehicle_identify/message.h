/************************************
 * 2021-08-27 by csr
 * 目前的调试信息：带有>>>的是运行信息，其他是DataSerial的信息。所有的DataSerial信息（不论发送还是返回）都会在DebugSerial同步显示。
 ************************************/
#ifndef _MESSAGE_H
#define _MESSAGE_H

#include <SoftwareSerial.h>

#define rxPin A9	//软串口rx
#define txPin A10	//软串口tx
SoftwareSerial MsgSerial(rxPin, txPin);

const char serverIP[]="192.168.4.1",cilentIP[]="192.168.4.2";
const int cilentID=0;
// cilentID其实应该是在start成功的时候从返回值里读取…… 
// 但是先这样写着，就是每次要先开1号车电源，再开2号车电源

class Message
{
	private:
		bool try_read(char str[]);
		void write_once(char command[]);
		void write(char command[],char msg[],bool mode);
		void clear();
	public:
		void init();
		void server_send(char text[]);
		void init_server();
		void init_cilent_WIFI();
		void init_cilent_TCP();
		void start_server();
		void start_cilent();
		int read_color();
		void send_color(int ch);
};

// 清空DataSerial的未读信息
void Message::clear()
{
	while (MsgSerial.read()>=0) {};
}

// 尝试从DataSerial读取str，成功返回true，失败返回false
bool Message::try_read(char str[])
{
	delay(500);
	int f[20],n=strlen(str);
	f[0]=-1;
	for (int i=1,j=-1;i<n;i++)
	{
		while (j!=-1 && str[i]!=str[j+1]) j=f[j];
		if (str[i]==str[j+1]) j++;
		f[i]=j;
	}
	int p=-1;
	for (char ch=MsgSerial.read();ch>=0;ch=MsgSerial.read())
	{
		while (p!=-1 && ch!=str[p+1]) p=f[p];
		if (str[p+1]==ch) p++;
		if (p==n-1) return true;
	}
	return false;
}

// 向DataSerial输出一次command
void Message::write_once(char command[])
{
	MsgSerial.println(command);
}

// 向DataSerial发送command并暂停程序，直到返回msg后继续运行
// mode=0: 失败后不重复发送
// mode=1: 失败后重复发送
void Message::write(char command[],char msg[],bool mode)
{
	clear();write_once(command);
	for (;!try_read(msg);)
		if (!mode)
			write_once(command);
}

// server向cilent发送text
void Message::server_send(char text[])
{
	char msg_text[20];
	sprintf(msg_text,"AT+CIPSEND=%d,%d",cilentID,strlen(text));
	write_once(msg_text);
	write(text,"SEND OK",0);
}

// Serial初始化
void Message::init()
{
	//MsgSerial.begin(115200);
	MsgSerial.begin(9600);
}

// server初始化、连接
void Message::init_server()
{
	//write("AT+RST","OK",1);
	write("AT+CWMODE_DEF=2","OK",1);
	write("AT+CWSAP_DEF=\"ZJU21\",\"12345678\",5,3","OK",1);
	write("AT+RST","OK",1);
	//for (clear();!try_read("CONNECTED"););
	write("AT+CIPMUX=1","OK",1);
	write("AT+CIPSERVER=1,333","OK",1);
	write("AT+CIPSTO=0","OK",1);
	for (;!try_read("CONNECT"););
}

void Message::start_server()
{
	pinMode(42, INPUT_PULLUP);
	while(digitalRead(42)){}	
}

// cilent初始化、连接，并启动vehicleB
void Message::init_cilent_WIFI()
{
	//write("AT+RST","OK",1);
	write("AT+CWMODE_DEF=1","OK",1);
	//write("AT+CWAUTOCONN=0","OK",1);
	//write("AT+RST","OK",1);
	write("AT+CWQAP","OK",1);
	write("AT+CWJAP_CUR=\"ZJU21\",\"12345678\"","CONNECTED",1);
}
void Message::init_cilent_TCP()
{
	write("AT+CIPSTART=\"TCP\",\"192.168.4.1\",333,3600","CONNECTED",0);
}

void Message::start_cilent()
{
	for (clear();!try_read("+IPD,5:START"););
}

int Message::read_color()
{
	for (;!try_read("+IPD,7:color="););
	return MsgSerial.read()-'0';
}
void Message::send_color(int ch)
{
	char msg_text[20];
	sprintf(msg_text,"AT+CIPSEND=%d,7",cilentID);
	MsgSerial.println(msg_text);
	sprintf(msg_text,"color=%d",ch);
	MsgSerial.println(msg_text);
}

#endif
