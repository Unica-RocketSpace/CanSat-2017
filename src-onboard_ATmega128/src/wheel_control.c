/*
 * wheel_control.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */

#include <avr/io.h>

#define DRV_PORT PORTC
#define DRV_DDR DDRC
#define DRV_SHIM_PORT PORTB
#define DRV_SHIM_DDR DDRB

#define WH1_1 0
#define WH1_2 1
#define WH2_1 2
#define WH2_2 3
#define WH3_1 4
#define WH3_2 5
#define SHIM1 1
#define SHIM2 2
#define SHIM3 3



void wheel_init()
{
	DRV_DDR = 1;
	DRV_SHIM_DDR = 1;
}

void setSHIMfreq(int SHIMn, float speed)
{

}

void wheel_run(int driver_number, float speed)
{
	int WHCn, WHDn, SHIMn;

	switch (driver_number)
	{
		case 1:
			WHCn = WH1_1;
			WHDn = WH1_2;
			SHIMn = SHIM1;
			break;
		case 2:
			WHCn = WH2_1;
			WHDn = WH2_2;
			SHIMn = SHIM2;
			break;
		case 3:
			WHCn = WH3_1;
			WHDn = WH3_2;
			SHIMn = SHIM3;
			break;
	}

	if (speed > 0) // TURN RIGHT
	{
		DRV_PORT |= (1 << WHCn);
		DRV_PORT |= (0 << WHDn);
	}
	if (speed < 0) // TURN LEFT
	{
		DRV_PORT |= (0 << WHCn);
		DRV_PORT |= (1 << WHDn);
	}

	DRV_SHIM_PORT |= (1 << SHIMn);
}

/*float get_wheel_speed()
{

	return globalWheelSpeed[3];
}*/
