/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */
#include <rscs/uart.h>
#include <rscs/i2c.h>
#include <rscs/spi.h>
#include <rscs/onewire.h>
#include <rscs/bmp280.h>
#include <rscs/ds18b20.h>
#include <rscs/timeservice.h>
#include <rscs/stdext/stdio.h>

#include <avr/io.h>
#include <util/delay.h>

#include "timer.h"
#include "radio_transmitter.h"
#include "kinematic_unit.h"
#include "dynamic_unit.h"
#include "MPU9255.h"


void blink_led()
{
		PORTG ^= (1 << 3);
		_delay_ms(500);

}


int main()
{

	_delay_ms(2000);
	DDRG = (1);
	hardwareInit();
	rscs_e error = rscs_ds18b20_start_conversion(ds18b20);
	printf("ds_start_error: %d\n", error);
	_delay_ms(1500);
	bool b = rscs_ds18b20_check_ready();
	printf("ds_ready: %d", b);

	while (1)
	{
		set_zero_pressure();
		blink_led();
		/*int16_t dummy1, dummy2;
		pressure_read_recon(&dummy1, &dummy2, &
				STATE.pressure);
		//pull_recon_data();
		printf("pressure: %f\n", STATE.pressure);*/

		pull_recon_data();
		//printf("ax: %f, ay: %f, az: %f\n", STATE.aRelatedXYZ[0], STATE.aRelatedXYZ[1], STATE.aRelatedXYZ[2]);
		//printf("height : %f\n", STATE.height);
		printf("temperature bmp280 : %f\n", STATE.temp_bmp280);
		printf("temperature ds18b20: %f\n\n", STATE.temp_ds18b20);


	}

	/*
	hardwareInit();
	set_ISC_offset();
	set_magn_dir();


	while(1)
	{
		pull_recon_data();
		construct_trajectory();

		if (STATE.state && 0b00000001)
		{
			recalc_ISC();
		}

		send_package();
	}*/
	_delay_ms(100);

	return 0;
}
