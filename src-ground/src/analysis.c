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


void content_FPrint(FILE * file_, package * pack, global_data * data, uint16_t flag)
{
	if (flag & NUMBER)
	{
		if (flag & CSV)
			fprintf(file_, "%d,", pack->number);
		else
		{
			fprintf(file_, "============================\n");
			fprintf(file_, "ПАКЕТ %d \n\n", pack->number);
		}
	}
	if (flag & P_BMP280)
	{
		if (flag & CSV)
			fprintf(file_, "%f,", data->pressure);
		else
			fprintf(file_, "ДАВЛЕНИЕ: %f Па\n", data->pressure);
	}
	if (flag & T_BMP280)
	{
		if (flag & CSV)
			fprintf(file_, "%f,", data->temp_ds18b20);
		else
			fprintf(file_, "ТЕМПЕРАТУРА: %f C \n", data->temp_ds18b20);
	}
	if (flag & T_DS18B20)
	{
		if (flag & CSV)
			fprintf(file_, "%f,", data->temp_bmp280);
		else
			fprintf(file_, "ТЕМПЕРАТУРА: %f C \n", data->temp_bmp280);
	}
	if (flag & ACCEL)
	{
		if (flag & CSV)
			fprintf(file_, "%f,%f,%f,", data->accel_XYZ[0], data->accel_XYZ[1], data->accel_XYZ[2]);
		else
			fprintf(file_, "Ускорения (XYZ): %f м/с^2\t%f м/с^2\t%f м/с^2\n\n", data->accel_XYZ[0], data->accel_XYZ[1], data->accel_XYZ[2]);
	}
	if (flag & GYRO)
	{
		if (flag & CSV)
			fprintf(file_, "%f,%f,%f,",  data->gyro_XYZ[0], data->gyro_XYZ[1], data->gyro_XYZ[2]);
		else
			fprintf(file_, "Угловые скорости (XYZ): %f рад/с\t%f рад/с\t%f рад/с\n\n",  data->gyro_XYZ[0], data->gyro_XYZ[1], data->gyro_XYZ[2]);
	}
	if (flag & COMPASS)
	{
		if (flag & CSV)
			fprintf(file_, "%f,%f,%f,",  data->compass_XYZ[0], data->compass_XYZ[1], data->compass_XYZ[2]);
		else
			fprintf(file_, "Cos (B,OXYZ): %f\t%f\t%f\n\n",  data->compass_XYZ[0], data->compass_XYZ[1], data->compass_XYZ[2]);
	}
	if (flag & STATE)
	{
		if (flag & CSV)
			fprintf(file_, "%d,", pack->state);
		else
			fprintf(file_, "Состояние: %d\n", pack->state);
	}
	if (flag & ACCEL)
	{
		if (flag & CSV)
			fprintf(file_, "%f\n", data->time);
		else
		{
			fprintf(file_, "ВРЕМЯ ПЕРЕДАЧИ: %f c\n", data->time);
			fprintf(file_, "============================\n");
		}

	}
}





