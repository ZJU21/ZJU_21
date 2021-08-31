/************************************
 * 2021-08-30 by csr
 ************************************/
#ifndef _SOUND_H
#define _SOUND_H

#include <SoftwareSerial.h>

#define SoundSerial Serial3

typedef enum
{
	_TEST,
	_WIFI,
	_TCP,
	_READY,
	_START,
} SOUND_TYPE;

const char sounds[][100]={
	"<G>欢迎使用MR628语音合成模块", 
	"<G>网络已连接",
	"<G>TCP已连接",
	"<G>准备出发",
	"<G>出发"
};

class Sound
{
	public:
		void init();
		void send(SOUND_TYPE text);
		void broadcast(int color);
};

void Sound::init()
{
	SoundSerial.begin(9600);
}

void Sound::send(SOUND_TYPE text)
{
	SoundSerial.print(sounds[text]);
	delay(200);
}

const char colorstr[][30]={
	"红色红色",
	"红色绿色",
	"红色蓝色",
	"绿色红色",
	"绿色绿色",
	"绿色蓝色",
	"蓝色红色",
	"蓝色绿色",
	"蓝色蓝色",
};
void Sound::broadcast(int color)
{
	char msg_text[1000];
	SoundSerial.print("<G>识别到的颜色是");
	SoundSerial.print(colorstr[color]);
	delay(200);
}

#endif
