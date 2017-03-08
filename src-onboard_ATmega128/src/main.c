/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */
#include <rscs/uart.h>
#include <rscs/i2c.h>
#include <rscs/spi.h>

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
	// настраиваем printf на уарт0
	rscs_uart_bus_t * uart0 = rscs_uart_init(RSCS_UART_ID_UART0,
			RSCS_UART_FLAG_ENABLE_TX /*| RSCS_UART_FLAG_BUFFER_TX*/);
	rscs_uart_set_baudrate(uart0, 9600);
	rscs_uart_set_character_size(uart0, 8);
	rscs_uart_set_parity(uart0, RSCS_UART_PARITY_NONE);
	rscs_uart_set_stop_bits(uart0, RSCS_UART_STOP_BITS_ONE);

	FILE * uart_std = rscs_make_uart_stream(uart0);
	stdout = uart_std;

	// настраиваем i2c, spi и прочее
	rscs_i2c_init();
	rscs_i2c_set_scl_rate(400);

	MPU9255_init();

	//rscs_spi_init();
}

int main()
{

	DDRB = (1 << 5);
	init_hw();

	while(1)
	{
		rscs_e e;
		uint8_t data1[12] = {0};
		float data2[6] = {0};

		/*e = MPU9255_accel_gyro_data(data1, data2);
		printf("\nAcceleromer (error: %d)\n", e);
		for (int i = 0; i < 3; i++)
		{
			printf("data[%d]: %d\n",i, (long)data2[i]);
		}
		printf("Gyroscope (error: %d)\n", e);
		for (int i = 3; i < 6; i++)
		{
			printf("data[%d]: %d\n",i, (long)data2[i]);
		}*/

		e = MPU9255_kompas_data(data1, data2);
		printf("Kompas (error: %d)\n", e);
		for (int i = 0; i < 3/*sizeof(data1)*/; i++)
		{
			printf("data[%d]: %d\n",i, (long)data2[i]);
		}

		_delay_ms(2000);
	}

	while(1)
	{
		uint8_t data1[12] = {0};

		rscs_e e;
		e = MPU9255_data_read(GYRO_AND_ACCEL, data1);

		for (int i = 0; i < sizeof(data1); i++)
		{
			printf("data[%d]: %d, error: %d\n",i, data1[i], e);
		}

		PORTB ^= (1 << 5);
		_delay_ms(2000);

	uint8_t data2[12] = {0};
		MPU9255_data_read(GYRO_AND_ACCEL, data2);

		for (int i = 0; i < sizeof(data2); i++)
		{
			printf("data[%d]: %d\n",i, data2[i]);
		}

		PORTB ^= (1 << 5);
		_delay_ms(2000);
	}
	return 0;
}
