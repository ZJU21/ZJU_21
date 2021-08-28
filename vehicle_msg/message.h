/************************************
 * 2021-08-27 by csr
 * 目前的调试信息：带有>>>的是运行信息，其他是DataSerial的信息。所有的DataSerial信息（不论发送还是返回）都会在DebugSerial同步显示。
 ************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <SoftwareSerial.h>

//#define DataSerial Serial3
#define rxPin xxx	//软串口rx
#define txPin xxx	//软串口tx
SoftwareSerial DataSerial(rxPin, txPin)
#define DebugSerial Serial

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
	// TODO: char* cilent_receive();
	void server_send(char text[]);
	void start_server();
	void start_cilent();
};

// 清空DataSerial的未读信息
void Message::clear()
{
	while (DataSerial.read()>=0) {};
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
	for (char ch=DataSerial.read();ch>=0;ch=DataSerial.read())
	{
		DebugSerial.print(ch);
		while (p!=-1 && ch!=str[p+1]) p=f[p];
		if (str[p+1]==ch) p++;
		if (p==n-1) return true;
	}
	return false;
}

// 向DataSerial输出一次command
void Message::write_once(char command[])
{
	DebugSerial.print(">>> try send: ");
	DebugSerial.println(command);
	DataSerial.println(command);
}

// 向DataSerial发送command并暂停程序，直到返回msg后继续运行
// mode=0: 失败后不重复发送
// mode=1: 失败后重复发送
void Message::write(char command[],char msg[],bool mode)
{
	write_once(command);
	for (clear();!try_read(msg);)
		if (!mode)
			write_once(command);
}

// server向cilent发送text
char msg_text[100];
void Message::server_send(char text[])
{
	sprintf(msg_text,"AT+CIPSEND=%d,%d\r\n%s",cilentID,strlen(text),text);
	DebugSerial.print(">>> ");
	DebugSerial.println(msg_text);
	write(msg_text,"SEND OK",1);
}

// Serial初始化
void Message::init()
{
	DataSerial.begin(115200);
	DebugSerial.begin(115200);
}

// server初始化、连接
void Message::start_server()
{
	write("AT+CWMODE_DEF=2","OK",1);
	write("AT+CWSAP_DEF=\"ZJU21\",\"12345678\",5,3","OK",1);
	write("AT+RST","OK",1);
	//for (clear();!try_read("CONNECTED"););
	write("AT+CIPMUX=1","OK",1);
	write("AT+CIPSERVER=1,333","OK",1);
	for (;!try_read("CONNECT"););
	DebugSerial.println(">>> ready to move");
}

// cilent初始化、连接，并启动vehicleB
void Message::start_cilent()
{
	write("AT+CWMODE_DEF=1","OK",1);
	write("AT+CWAUTOCONN=0","OK",1);
	DebugSerial.println(">>> CWMODE set");
	write("AT+CWJAP_DEF=\"ZJU21\",\"12345678\"","CONNECTED",1);
	DebugSerial.println(">>> WiFi connected");
	write("AT+CIPSTART=\"TCP\",\"192.168.4.1\",333","CONNECTED",0);
	DebugSerial.println(">>> TCP connected");
	for (clear();!try_read("+IPD,5:START"););
	DebugSerial.println(">>> start moving");
}

#endif
