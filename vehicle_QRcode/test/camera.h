/************************************
 * 2021-08-30 by csr
 ************************************/

#ifndef _CAMERA_H
#define _CAMERA_H

#define DebugSerial Serial
#define ScanSerial Serial2

class Camera
{
	public:
		void init();
		int get_QRcode();
};

void Camera::init()
{
	ScanSerial.begin(115200);
	DebugSerial.begin(115200);
	DebugSerial.println("test");
}

int Camera::get_QRcode()
{
	char ch;
	for (ch=ScanSerial.read();ch!='c';ch=ScanSerial.read())
	for (ch=ScanSerial.read();ch<0;ch=ScanSerial.read());
	return ch-'0';
}

#endif
