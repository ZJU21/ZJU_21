/************************************
 * 2021-08-26 by csr
 ************************************/
#include "camera.h"
#include "sound.h"

Camera cam;
Sound sound;
void setup() 
{
	sound.init();
	cam.init();
	//sound.send(_START);
}

void loop() 
{
	int color=cam.get_QRcode();
	sound.broadcast(color);
	DebugSerial.println(color);
}
