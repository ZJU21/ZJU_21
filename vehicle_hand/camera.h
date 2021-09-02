/************************************
 * 2021-08-30 by csr
 ************************************/

#ifndef _CAMERA_H
#define _CAMERA_H

#define ScanSerial Serial2

struct Position
{
	int x,y,size;
};

class Camera
{
	int w=160,h=120;
	private:
		int readint();
		void clear();
	public:
		void init();
		int get_QRcode();
		Position get_Aposition();
		Position get_Bposition();
};

int Camera::readint()
{
	char ch;int x;
	for (ch=ScanSerial.read();ch<'0' || ch>'9';ch=ScanSerial.read());
	for (x=0;ch!='.';ch=ScanSerial.read())
		if (ch>='0' && ch<='9')
			x=x*10+ch-'0';
	return x;
}

void Camera::init()
{
	ScanSerial.begin(115200);
	ScanSerial.println("start test");
}

int Camera::get_QRcode()
{
	char ch;
	while (ScanSerial.read()!='C');
	for (ch=ScanSerial.read();ch<0;ch=ScanSerial.read());
	return ch-'0';
}

void Camera::clear()
{
	while (ScanSerial.read()>=0);
}

Position Camera::get_Aposition()
{
	clear();
	Position ans;
	while (ScanSerial.read()!='A');
	ans.x=readint()-w/2;ans.y=readint()-h/2;ans.size=readint();
	return ans;
}

Position Camera::get_Bposition()
{
	clear();
	Position ans;
	while (ScanSerial.read()!='B');
	ans.x=readint()-w/2;ans.y=readint()-h/2;ans.size=readint();
	return ans;
}

#endif
