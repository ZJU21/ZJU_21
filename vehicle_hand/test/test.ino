/************************************
 * 2021-08-26 by csr
 ************************************/
#include "camera.h"

Camera cam;

void setup() 
{
	cam.init();
	Serial.begin(115200);
	Serial.println("scan QRcode");
	int x=cam.get_QRcode();
	Serial.print(">>>color:");
	Serial.println(x);
	return;
	for (;;)
	{
		Position a=cam.get_Aposition();
		Serial.print(">>>A position:");
		Serial.print("  x=");
		Serial.print(a.x);
		Serial.print("  y=");
		Serial.print(a.y);
		Serial.print("  size=");
		Serial.println(a.size);
	}
}

void loop() 
{
}
