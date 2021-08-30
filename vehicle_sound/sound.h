/************************************
 * 2021-08-30 by csr
 ************************************/
#ifndef _SOUND_H_
#define _SOUND_H

#include <SoftwareSerial.h>

#define sound_rxPin 100
#define sound_txPin 42
SoftwareSerial SoundSerial(sound_rxPin, sound_txPin);

typedef enum
{
	_TEST,
	_READY,
	_START,
} SOUND_TYPE;

const char sounds[][100]={
	"<G>��ӭʹ��MR628�����ϳ�ģ��",
	"<G>׼������",
	"<G>����"
};

class Sound
{
	public:
		void init();
		void send(SOUND_TYPE text);
};

void Sound::init()
{
	SoundSerial.begin(9600);
}

void Sound::send(SOUND_TYPE text)
{
	SoundSerial.print(sounds[text]);
}

#endif
