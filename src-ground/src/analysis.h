/*
 * analysis.h
 *
 *  Created on: 23 июня 2017 г.
 *      Author: developer
 */

#ifndef ANALYSIS_H_
#define ANALYSIS_H_
#include "package_struct.h"

#define NUMBER 		(1 << 0)
#define P_BMP280 	(1 << 1)
#define T_BMP280 	(1 << 2)
#define T_DS18B20 	(1 << 3)
#define ACCEL 		(1 << 4)
#define GYRO 		(1 << 5)
#define COMPASS 	(1 << 6)
#define STATE 		(1 << 7)
#define TIME 		(1 << 8)
#define CSV			(1 << 9)
#define ALL			NUMBER | P_BMP280 | T_BMP280 | T_DS18B20 | ACCEL | GYRO | COMPASS | STATE | TIME


typedef struct
{
	float pressure;
	float temp_ds18b20;
	float temp_bmp280;
	float accel_XYZ[3];
	float gyro_XYZ[3];
	float compass_XYZ[3];
	float time;

}global_data;

void content_FPrint(FILE * file_, package * pack, global_data * data, uint16_t flag);

#endif /* ANALYSIS_H_ */
