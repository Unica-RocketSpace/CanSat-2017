/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */

#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <rscs/uart.h>
#include <rscs/i2c.h>
#include <rscs/spi.h>
#include <rscs/onewire.h>
#include <rscs/bmp280.h>
#include <rscs/ds18b20.h>
#include <rscs/timeservice.h>
#include <rscs/stdext/stdio.h>

#include "timer.h"
#include "radio_transmitter.h"
#include "kinematic_unit.h"
#include "dynamic_unit.h"
#include "MPU9255.h"


void blink_led()
{
	DDRG |= (1 << 3);
	PORTG ^= (1 << 3);
	_delay_ms(100);
}

void send_calibration_values()
{
	const rscs_bmp280_calibration_values_t * bmp280_cal_values = rscs_bmp280_get_calibration_values(bmp280);
	uint8_t i = 10, mark = 0xFF;
	while(i--)
	{
		rscs_uart_write(uart0, &mark, 1);
		rscs_uart_write(uart0, &bmp280_cal_values, sizeof(bmp280_cal_values));
	}
}

int main()
{
	_delay_ms(1000);

	hardwareInit();

	send_calibration_values();

	//set_zero_pressure();	//устанавливаем нулевое давление

	//set_ISC_offset();
	//FIXME: Внутри функции set_ISC_offset мигалка работает до самого конца,
	//а сразу после выхода из функции не работает
	//while (1) {DDRG |= (1 << 3);PORTG ^= (1 << 3);_delay_ms(100);}
	//set_magn_dir();

	int p_number = 0;

	while(1)
	{
		blink_led();


		pull_recon_data();
		construct_trajectory();

		p_number++;

		printf("TIME: %f  [s]\n", (float)STATE.Time / 1000);
		printf("=============PACKAGE NUMBER [ %d ]====\n", p_number);
		printf("-----MAIN MISSION-----\n");
		printf("Accelerations (ADXL345): %f, %f, %f  [m/s^2]\n", G_VECT*STATE.aALT_XYZ[0]/1.08, G_VECT*STATE.aALT_XYZ[1]/1.08, G_VECT*STATE.aALT_XYZ[2]/1.17);
		printf("Pressure      (BMP280) : %f  [Pa]\n", STATE.pressure);
		printf("Temperature   (BMP280) : %f  [oC]\n", STATE.temp_bmp280);
		printf("Temperature   (DS18B20): %f  [oC]\n", STATE.temp_ds18b20);

		printf("-----ADDITIONAL MISSION-----\n");
		printf("Accelerations    (MPU9255): %f, %f, %f  [m/s^2]\n", STATE.aRelatedXYZ[0], STATE.aRelatedXYZ[1], STATE.aRelatedXYZ[2]);
		printf("Angle velocities (MPU9255): %f, %f, %f  [1/s]\n", STATE.gRelatedXYZ[0], STATE.gRelatedXYZ[1], STATE.gRelatedXYZ[2]);
		printf("Magnetic vector  (MPU9255): %f, %f, %f  [-]\n", STATE.cRelatedXYZ[0], STATE.cRelatedXYZ[1], STATE.cRelatedXYZ[2]);
		printf("=============END OF PACKAGE=============\n\n");


		if (STATE.state & (1 << 1))
		{
			recalc_ISC();
		}
		send_package();
		_delay_ms(500);
	}

	return 0;
}


//TODO: ОТКАЛИБРОВАТЬ ДАТЧИКИ
//TODO: УЧЕСТЬ, ЧТО КАМЕРА ПОВЕРНУТА НА 45* ОТНОСИТЕЛЬНО ССК


