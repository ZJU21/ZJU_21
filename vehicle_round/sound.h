/************************************
 * 2021-08-30 by csr
 ************************************/
#ifndef _SOUND_H
#define _SOUND_H

//#include <SoftwareSerial.h>
#define SoundSerial Serial3
//SoftwareSerial SoundSerial(0, 30);
//#define DebugSerial Serial

typedef enum
{
	_TEST,
	_WIFI,
	_TCP,
	_READY,
	_START,
} SOUND_TYPE;

const char sounds[][100]={
	"<G>语音模块正常", 
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
 // DebugSerial.begin(115200);
	SoundSerial.begin(9600);
}

void Sound::send(SOUND_TYPE text)
{
  	delay(200);
	SoundSerial.print(sounds[text]);
}
const char colorstr[][300]={
	"<G>红色",
	"<G>绿色",
	"<G>蓝色"
};
void Sound::broadcast(int color)
{
  	delay(200);
	SoundSerial.print(colorstr[color/3]);
  	delay(200);
	SoundSerial.print(colorstr[color%3]);
  	delay(200);
}

#endif
