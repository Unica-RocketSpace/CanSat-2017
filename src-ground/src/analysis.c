/*
 * analysis.c
 *
 *  Created on: 23 июня 2017 г.
 *      Author: developer
 */

#include <stdio.h>

#include "analysis.h"
#include "package_struct.h"
#include "recalculation.h"

void content_PrintRecalc(package * pack, package_content content)
{
	float accel_XYZ[3] = {0};	float gyro_XYZ[3] = {0};	float compass_XYZ[3] = {0};
	float time_;

	switch (content){
		case number:		printf("============================\n");
							printf("ПАКЕТ %d \n\n", pack->number);			break;
		case pressure:		printf("ДАВЛЕНИЕ: %d Па\n", pack->pressure);	break;
		case temperature:	printf("ТЕМПЕРАТУРА: %d C \n", pack->temp);	break;

		case accelerometer_data:	recalc_accel((int16_t*)(pack->aXYZ), accel_XYZ);
									printf("Ускорение по оси X: %f м/с^2\n", accel_XYZ[0]);
									printf("Ускорение по оси Y: %f м/с^2\n", accel_XYZ[1]);
									printf("Ускорение по оси Z: %f м/с^2\n", accel_XYZ[2]);
									printf("\n");			break;

		case gyroscope_data:		recalc_gyro((int16_t*)(pack->gXYZ), gyro_XYZ);
									printf("Угловая скорость по оси X: %f рад/с\n", gyro_XYZ[0]);
									printf("Угловая скорость по оси Y: %f рад/с\n", gyro_XYZ[1]);
									printf("Угловая скорость по оси Z: %f рад/с\n", gyro_XYZ[2]);
									printf("\n");			break;

		case compass_data:			recalc_compass((int16_t*)(pack->cXYZ), compass_XYZ);
									printf("cos угла (B,OX): %f\n", compass_XYZ[0]);
									printf("cos угла (B,OY): %f\n", compass_XYZ[1]);
									printf("cos угла (B,OZ): %f\n", compass_XYZ[2]);
									printf("\n");			break;

		case state:			printf("Состояние: %d\n", pack->state);			break;
		case time:			time_ = pack->time / 1000;
							printf("ВРЕМЯ ПЕРЕДАЧИ: %f c\n", time_);			break;

	}
}
