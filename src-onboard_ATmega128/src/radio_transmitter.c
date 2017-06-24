/*
 * radio_transmitter.c
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: MIHAIL
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <rscs/bmp280.h>
#include <rscs/ds18b20.h>

#include "../librscs/rscs/uart.h"

#include "radio_transmitter.h"
#include "kinematic_unit.h"
#include "rscs/timeservice.h"
#include "rscs/stdext/stdio.h"


rscs_uart_bus_t * uart0;
package PACKAGE;

void transmition_init()
{

	uart0 = rscs_uart_init(RSCS_UART_ID_UART0,
				RSCS_UART_FLAG_ENABLE_TX /*| RSCS_UART_FLAG_BUFFER_TX*/);
	rscs_uart_set_baudrate(uart0, 9150);
	rscs_uart_set_character_size(uart0, 8);
	rscs_uart_set_parity(uart0, RSCS_UART_PARITY_NONE);
	rscs_uart_set_stop_bits(uart0, RSCS_UART_STOP_BITS_ONE);

//FIXME: УБРАТЬ!!!!!!!
	// настраиваем printf на уарт0
	stdout = rscs_make_uart_stream(uart0);

	package PACKAGE_ = { 0 };
	PACKAGE = PACKAGE_;

}

void full_package()
{
	PACKAGE.marker = 0xFFFF;

	PACKAGE.pressure = TRANSMIT_DATA.pressure;

	PACKAGE.temp_ds18b20 = TRANSMIT_DATA.temp_ds18b20;	//температура с bmp280
	PACKAGE.temp_bmp280 = TRANSMIT_DATA.temp_bmp280;	//температура с ds18b20

	//запись ускорений
	PACKAGE.aXYZ[0] = TRANSMIT_DATA.aTransmitXYZ[0];
	PACKAGE.aXYZ[1] = TRANSMIT_DATA.aTransmitXYZ[1];
	PACKAGE.aXYZ[2] = TRANSMIT_DATA.aTransmitXYZ[2];

	//запись угловых скоростей
	PACKAGE.gXYZ[0] = TRANSMIT_DATA.gTransmitXYZ[0];
	PACKAGE.gXYZ[1] = TRANSMIT_DATA.gTransmitXYZ[1];
	PACKAGE.gXYZ[2] = TRANSMIT_DATA.gTransmitXYZ[2];

	//запись показаний компаса
	PACKAGE.cXYZ[0] = TRANSMIT_DATA.cTransmitXYZ[0];
	PACKAGE.cXYZ[1] = TRANSMIT_DATA.cTransmitXYZ[1];
	PACKAGE.cXYZ[2] = TRANSMIT_DATA.cTransmitXYZ[2];

	//запись текущего времени
	PACKAGE.time = STATE.Time;

	//запись состояния (во внешнем прерывании)

}


void count_CS()
{
	uint8_t * first_PACKAGE = (uint8_t*)&PACKAGE;

	PACKAGE.CS = 0;

	for (int i = 0; i < sizeof(PACKAGE) - 2; i++)
	{
		uint32_t tCS = PACKAGE.CS + *(first_PACKAGE + i);
		PACKAGE.CS = (uint16_t)(tCS & 0x0000FFFF);
	}

}


void send_package()
{
	full_package();
	count_CS();

	//запись пакета в UART
	rscs_uart_write(uart0, &PACKAGE, sizeof(PACKAGE));
	PACKAGE.number++;

}
