/*
 * radio_transmitter.c
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: MIHAIL
 */

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "radio_transmitter.h"

uint8_t packet_size = sizeof(packet);
uint8_t * packet_address = (uint8_t *)&packet;

uint16_t count_CS()
{
	uint16_t CS = 0;

	for (int i = 0; i < packet_size - 2; i++)
	{
		uint32_t HCS = CS + *(packet_address + i);
		CS = (uint16_t)(HCS & 0x0000FFFF);
	}

	return CS;
}
