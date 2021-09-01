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
	"<G>��ӭʹ��MR628�����ϳ�ģ��", 
	"<G>����������",
	"<G>TCP������",
	"<G>׼������",
	"<G>����"
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
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
	"��ɫ��ɫ",
};
void Sound::broadcast(int color)
{
	char msg_text[1000];
	SoundSerial.print("<G>ʶ�𵽵���ɫ��");
	SoundSerial.print(colorstr[color]);
	delay(200);
}

#endif
