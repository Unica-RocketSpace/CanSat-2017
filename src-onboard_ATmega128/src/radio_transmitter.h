/*
 * radio_transmitter.h
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#ifndef RADIO_TRANSMITTER_H_
#define RADIO_TRANSMITTER_H_

#include <rscs/uart.h>

extern rscs_uart_bus_t * uart0;

//#pragma pack(push,1)
typedef struct
{
	uint16_t marker;	//marker

	uint16_t number;	//the number of packet

	uint16_t pressure;

	uint16_t temp_ds18b20;
	uint16_t temp_bmp280;

	uint16_t aXYZ[3];	//accelerometer raw data
	uint16_t gXYZ[3];	//gyro raw data
	uint16_t cXYZ[3];	//compass raw data

	uint32_t time;		//sending time

	uint8_t state;		//condition of the device
	uint16_t CS;		//control summ (CS) for checking packages on the Earth

}package;
//#pragma pack(pop)

extern package PACKAGE;				//name of package



//ИНИЦИАЛИЗИРУЕТ ПРОЦЕСС ПЕРЕДАЧИ ДАННЫХ

void transmition_init();

//ЗАПОЛНЯЕТ ПАКЕТ
void full_package();

//СЧИТАЕТ КОНТРОЛЬНУЮ СУММУ
void count_CS();

//ОТПРАВЛЯЕТ ПАКЕТ ДАННЫХ ТЕЛЕМЕТРИИ

void send_package();



#endif /* RADIO_TRANSMITTER_H_ */
