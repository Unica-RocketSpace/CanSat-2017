/*
 * package_struct.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#ifndef PACKAGE_STRUCT_H_
#define PACKAGE_STRUCT_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>


#pragma pack(push,1)
typedef struct
{
	uint16_t marker;	//marker

	uint16_t number;	//the number of packet

	uint16_t pressure;	//pressure data

	uint16_t temp;		//temperature data 1

	uint16_t aXYZ[3];	//accelerometer raw data
	uint16_t gXYZ[3];	//gyro raw data
	uint16_t cXYZ[3];	//compass raw data

	uint32_t time;		//sending time

	uint8_t state;		//condition of the device
	uint16_t CS;		//control summ (CS) for checking packages on the Earth

}package;
#pragma pack(pop)

extern package PACKAGE;				//name of package


//extern package PACKAGE;
//extern package * PACKAGE_ADDR;


#endif /* PACKAGE_STRUCT_H_ */
