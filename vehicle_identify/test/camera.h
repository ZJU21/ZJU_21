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
		void readint(int &x);
	public:
		void init();
		int get_QRcode();
		Position get_Aposition();
		Position get_Bposition();
};


void Camera::readint(int &x)
{
	char ch;
	for (ch=ScanSerial.read();ch<'0' || ch>'9';ch=ScanSerial.read());
	for (x=0;ch!='.';ch=ScanSerial.read())
		if (ch>='0' && ch<='9')
			x=x*10+ch-'0';
}

void Camera::init()
{
	ScanSerial.begin(115200);
}

int Camera::get_QRcode()
{
	ScanSerial.println("Q");
	char ch;
	for (ch=ScanSerial.read();ch!='C';ch=ScanSerial.read());
	for (ch=ScanSerial.read();ch<0;ch=ScanSerial.read());
	return ch-'0';
}

Position Camera::get_Aposition()
{
	ScanSerial.println("A");
	Position ans;
	readint(ans.x);readint(ans.y);readint(ans.size);
	return ans;
}

Position Camera::get_Bposition()
{
	ScanSerial.println("B");
	Position ans;
	readint(ans.x);readint(ans.y);readint(ans.size);
	return ans;
}

#endif
