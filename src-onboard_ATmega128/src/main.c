/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */
#include <rscs/uart.h>
#include <rscs/i2c.h>
#include <rscs/spi.h>
#include <rscs/timeservice.h>
#include <rscs/stdext/stdio.h>

#include <avr/io.h>
#include <util/delay.h>

#include "timer.h"
#include "radio_transmitter.h"
#include "kinematic_unit.h"
#include "dynamic_unit.h"
#include "MPU9255.h"


void init_hw(void)
{

/*	// настраиваем printf на уарт0
	FILE * uart_std = rscs_make_uart_stream(uart0);
	stdout = uart_std;
*/

	// настраиваем i2c, spi и прочее
	rscs_i2c_init();
	rscs_i2c_set_scl_rate(400);

	MPU9255_init();

	rscs_spi_init();
	rscs_time_init();

	transmition_init();
}

int main()
{

	DDRB = (1 << 5);
	init_hw();
	set_ISC();

	while(1)
	{
		//rscs_e e;
		//uint8_t data1[12] = {0};
		//float data2[6] = {0};
		int16_t kompas_data[3] = {0};

		/*e = MPU9255_accel_gyro_data(data1, data2);
		printf("\nAcceleromer (error: %d)\n", e);
		for (int i = 0; i < 3; i++)
		{
			printf("data[%d]: %ld\n",i, (long)data2[i]);
		}
		printf("Gyroscope (error: %d)\n", e);
		for (int i = 3; i < 6; i++)
		{
			printf("data[%d]: %ld\n",i, (long)data2[i]);
		}*/

		/*e = MPU9255_kompas_data(kompas_data);
		printf("Kompas (error: %d)\n", e);*/
		for (int i = 0; i < 3/*sizeof(data1)*/; i++)
		{
			//printf("data[%d]: %ld\n",i, (long)data2[i]);
			printf("data[%d]: %d\n",i, kompas_data[i]);
		}

		_delay_ms(500);
	}

	return 0;
}
