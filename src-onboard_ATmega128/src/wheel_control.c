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

#define WH1_C 0
#define WH1_D 1
#define WH2_C 2
#define WH2_D 3
#define WH3_C 4
#define WH3_D 5
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

void wheel_run(float speed1, float speed2, float speed3)
{


	if (speed1 > 0) // TURN RIGHT
	{
		DRV_PORT |= (1 << WH1_C);
		DRV_PORT |= (0 << WH1_D);
	}
	else		    // TURN LEFT
	{
		DRV_PORT |= (0 << WH1_C);
		DRV_PORT |= (1 << WH1_D);
	}

	if (speed2 > 0) // TURN RIGHT
	{
		DRV_PORT |= (1 << WH2_C);
		DRV_PORT |= (0 << WH2_D);
	}
	else		    // TURN LEFT
	{
		DRV_PORT |= (0 << WH2_C);
		DRV_PORT |= (1 << WH2_D);
	}
	if (speed3 > 0) // TURN RIGHT
	{
		DRV_PORT |= (1 << WH3_C);
		DRV_PORT |= (0 << WH3_D);
	}
	else		    // TURN LEFT
	{
		DRV_PORT |= (0 << WH3_C);
		DRV_PORT |= (1 << WH3_D);
	}

	OCR1A = (int)(speed1 * 65535);
	OCR1B = (int)(speed2 * 65535);
	OCR1C = (int)(speed3 * 65535);

}

/*float get_wheel_speed()
{

	return globalWheelSpeed[3];
}*/




