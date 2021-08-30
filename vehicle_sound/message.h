/************************************
 * 2021-08-27 by csr
 * 目前的调试信息：带有>>>的是运行信息，其他是MsgSerial的信息。所有的MsgSerial信息（不论发送还是返回）都会在DebugSerial同步显示。
 ************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <SoftwareSerial.h>

//#define MsgSerial Serial3
#define msg_rxPin A9	//软串口rx
#define msg_txPin A10	//软串口tx
SoftwareSerial MsgSerial(msg_rxPin, msg_txPin);
#define DebugSerial Serial

const char serverIP[]="192.168.4.1",cilentIP[]="192.168.4.2";
const int cilentID=0;
// cilentID其实应该是在start成功的时候从返回值里读取…… 
// 但是先这样写着，就是每次要先开1号车电源，再开2号车电源

class Message
{
	private:
		void clear();
		bool try_read(char str[]);
		void write_once(char command[]);
		void write(char command[],char msg[],bool mode);
		void server_send(char text[]);
		char* cilent_read();
	public:
		void init();
		// TODO: char* cilent_receive();
		void init_server();
		void init_cilent();
		void start_server();
		void start_cilent();
};

// 清空MsgSerial的未读信息
void Message::clear()
{
	while (MsgSerial.read()>=0) {};
}

// 尝试从MsgSerial读取str，成功返回true，失败返回false
bool Message::try_read(char str[])
{
	delay(1000);
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
		DebugSerial.print(ch);
		while (p!=-1 && ch!=str[p+1]) p=f[p];
		if (str[p+1]==ch) p++;
		if (p==n-1) return true;
	}
	return false;
}

// 向MsgSerial输出一次command
void Message::write_once(char command[])
{
	DebugSerial.print(">>> try send: ");
	DebugSerial.println(command);
	MsgSerial.println(command);
}

// 向MsgSerial发送command并暂停程序，直到返回msg后继续运行
// mode=0: 失败后重复发送
// mode=1: 失败后不重复发送
void Message::write(char command[],char msg[],bool mode)
{
	write_once(command);
	for (clear();!try_read(msg);)
		if (!mode)
			write_once(command);
}

// server向cilent发送text
void Message::server_send(char text[])
{
	char msg_text[100];
	sprintf(msg_text,"AT+CIPSEND=%d,%d",cilentID,strlen(text));
	DebugSerial.print(">>> ");
	DebugSerial.println(msg_text);
	write_once(msg_text);
	write(text,"SEND OK",0);
}

// Serial初始化
void Message::init()
{
	//MsgSerial.begin(115200);
	MsgSerial.begin(9600);
	DebugSerial.begin(115200);
}

// server初始化、连接
void Message::init_server()
{
	//write("AT+RST","OK",1);
	write("AT+CWMODE_DEF=2","OK",1);
	write("AT+CWSAP_DEF=\"ZJU21\",\"12345678\",5,3","OK",1);
	write("AT+RST","ready",1);
	//for (clear();!try_read("CONNECTED"););
	write("AT+CIPMUX=1","OK",1);
	write("AT+CIPSERVER=1,333","OK",1);
	write("AT+CIPSTO=0","OK",1);
	for (;!try_read("CONNECT"););
	DebugSerial.println(">>> ready to move");
}

// cilent初始化、连接，并启动vehicleB
void Message::init_cilent()
{
	//write("AT+RST","OK",1);
	write("AT+CWMODE_DEF=1","OK",1);
	//write("AT+CWAUTOCONN=0","OK",1);
	DebugSerial.println(">>> CWMODE set");
	//write("AT+RST","OK",1);
	write("AT+CWQAP","OK",1);
	write("AT+CWJAP_CUR=\"ZJU21\",\"12345678\"","CONNECTED",1);
	DebugSerial.println(">>> WiFi connected");
	write("AT+CIPSTART=\"TCP\",\"192.168.4.1\",333,3600","CONNECTED",0);
	DebugSerial.println(">>> TCP connected");
}

void Message::start_server()
{
	pinMode(42, INPUT_PULLUP);
	while(digitalRead(42)){}
	server_send("START");
}

char msg_content[100];
char* Message::cilent_read()
{
	for (clear();!try_read("+IPD,");)
		delay(100);
	char *p=msg_content;
	for (*p=Serial.read();*p>=0;p++,*p=MsgSerial.read());
	*p='\0';
	return msg_content;
}

void Message::start_cilent()
{
	//cilent_read();
	for (clear();!try_read("+IPD,5:START");)
		delay(100);
	DebugSerial.println(">>> start moving");
}

#endif
