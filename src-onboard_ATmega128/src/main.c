/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */
#include <rscs/uart.h>
#include <rscs/stdext/stdio.h>
#include <avr/io.h>
#include <math.h>

#include "timer.h"
#include "globals.h"
#include "radio_transmitter.h"
#include "sensors_poll.h"
#include "ADXL345.h"

#include "wheel_control.h"
#include "servo_control.h"


int main()
{
	/*rscs_uart_bus_t uart0_ = {
			&UDR0, &UCSR0A, &UCSR0B, &UCSR0C,
			&UBRR0L, &UBRR0H
	};

	rscs_uart_bus_t * uart0 = &uart0_;

	rscs_uart_init(
		uart0,
		RSCS_UART_FLAG_ENABLE_TX | RSCS_UART_FLAG_ENABLE_RX);
	rscs_uart_set_baudrate(uart0, 9600);
	rscs_uart_set_character_size(uart0, 8);
	rscs_uart_set_parity(uart0, RSCS_UART_PARITY_NONE);
	rscs_uart_set_stop_bits(uart0, 1);

	FILE * uart0stream = rscs_make_uart_stream(uart0);
	fprintf(uart0stream, "%s", "fdgdfg");
	stdout = uart0stream;
	printf("%s", "fdgdfg");*/

	timer0_init();
	timer1_init();
	wheel_init();

	rscs_i2c_set_scl_rate(i2c, 400);
	rscs_isc_set_timeout(i2c, 200);
	rscs_i2c_reset(i2c);


	return 0;
}

ISR(TIMER0_COMP_vect)
{
	ADXL345_GetGXYZ(&packet.accelX, &packet.accelY, &packet.accelZ, &STATE.aRawX, &STATE.aRawY, &STATE.aRawZ);

	OCR1B = setServoCosAngle() * 65025 / M_PI;




	setServoCosAngle();
	return 0;
}

