/************************************
 * 2021-08-30 by csr
 ************************************/

#ifndef _CAMERA_H
#define _CAMERA_H

#define ScanSerial Serial

class CAMERA
{
	public:
		void init();
		int get_QRcode();
};

void CAMERA::init()
{
	ScanSerial.begin(115200);
}

int CAMERA::get_QRcode()
{
	for (char ch=ScanSerial.read();ch!='c';);
	return ScanSerial.read()-'0';
}

#endif
