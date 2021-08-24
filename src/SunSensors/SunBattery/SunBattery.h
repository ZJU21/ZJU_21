#ifndef SUN_BATTERY_h
#define SUN_BATTERY_h


#include "Arduino.h"
#include "SunConfig.h"


class Battery
{
public:
	Battery();
	Battery(byte battery_Pin, unsigned int threshold = VOLT_THRESHOLD);
	unsigned int read(void);
	bool is_Volt_Low(void);

private:
	byte _Battery_Pin;
	unsigned int _Threshold;
};

#endif
