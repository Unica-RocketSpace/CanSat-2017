/*
 * radio_transmitter.h
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#ifndef RADIO_TRANSMITTER_H_
#define RADIO_TRANSMITTER_H_

extern uint8_t packet_size;

#pragma pack(push,1)
typedef struct
{
	uint16_t number;	//the number of packet
	uint16_t pressure1;	//pressure data 1
	uint16_t pressure2;	//pressure data 2
	uint16_t temp1;		//temperature data 1
	uint16_t temp2;		//temperature data 2
	uint16_t timeH;		//sending time (high register, uses TimeServise)
	uint16_t timeL;		//sending time (low register, uses TimeServise)
	uint16_t accelX;	//accelerometer raw data (X-axis)
	uint16_t accelY;	//accelerometer raw data (Y-axis)
	uint16_t accelZ;	//accelerometer raw data (Z-axis)
	//uint16_t gyro		//gyro data ?????????????????????
	uint8_t state;		//condition of the device
	uint16_t CS;		//control summ (CS) for checking packages on the Earth

}package;
#pragma pack(pop)

extern package packet;			//name of package
extern uint8_t* packet_address;	//address of package

#endif /* RADIO_TRANSMITTER_H_ */
