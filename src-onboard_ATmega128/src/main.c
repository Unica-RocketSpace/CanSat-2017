/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */
#include <rscs/uart.h>
#include <rscs/stdext/stdio.h>
#include <avr/io.h>

#include "timer.h"
#include "wheel_control.h"


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

	timer_init();
	wheel_init();



	return 0;
}
