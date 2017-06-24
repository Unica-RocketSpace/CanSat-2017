/*
 * recalculation.c
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "package_struct.h"
#include "recalculation.h"

static const rscs_bmp280_calibration_values_t calvals =
{
		.T1 = 0,
		.T2 = 0,
		.T3 = 0,
		.P1 = 0,
		.P2 = 0,
		.P3 = 0,
		.P4 = 0,
		.P5 = 0,
		.P6 = 0,
		.P7 = 0,
		.P8 = 0,
		.P9 = 0
};

void recalc_accel(int16_t * raw_accel_XYZ, float * accel_XYZ)
{
	int16_t *first_raw_accel_XYZ = (int16_t*)raw_accel_XYZ;
	float *first_accel_XYZ = (float*)accel_XYZ;

	for (int i = 0; i < 3; i++)
	{
		first_accel_XYZ[i] = (float)(first_raw_accel_XYZ[i]) * ACCEL_SCALE_FACTOR * ACCEL_RANGE;
	}

}


void recalc_gyro(int16_t * raw_gyro_XYZ, float * gyro_XYZ)
{
	int16_t *first_raw_gyro_XYZ = (int16_t*)raw_gyro_XYZ;
	float *first_gyro_XYZ = (float*)gyro_XYZ;

	for (int i = 0; i < 3; i++)
		{
			first_gyro_XYZ[i] = (float)(first_raw_gyro_XYZ[i]) * ACCEL_SCALE_FACTOR * ACCEL_RANGE;
		}

}


void recalc_compass(int16_t * raw_compass_XYZ, float * compass_XYZ)
{
	float x = 0, y = 0, z = 0;
	int16_t *first_raw_compass_XYZ = (int16_t*)raw_compass_XYZ;
	float *first_compass_XYZ = (float*)compass_XYZ;


	float length = sqrt(pow(*(first_raw_compass_XYZ + 0), 2) + pow(*(first_raw_compass_XYZ + 1), 2) + pow(*(first_raw_compass_XYZ + 2), 2));

	if (length > 0)
	{
		x = ((float)*(first_raw_compass_XYZ + 1) / length);
		y = ((float)*(first_raw_compass_XYZ + 0)/ length);
		z = - ((float)*(first_raw_compass_XYZ + 2)/ length);
	}
	else {
		x = 0; y = 0; z = 0;
	}


	first_compass_XYZ[0] = x;
	first_compass_XYZ[1] = y;
	first_compass_XYZ[2] = z;
}

float recalc_ds18b20Temp(uint16_t raw_temp)
{
	return raw_temp / 16.0f;
}

float recalc_bmp280Temp(int32_t rawtemp)
{
	int32_t temp_p;
	int32_t t_fine;

	int32_t var1, var2;
	var1 = ((((rawtemp >> 3) - (((int32_t)calvals.T1) << 1))) * ((int32_t)calvals.T2)) >> 11;
	var2 = (((((rawtemp >> 4) - ((int32_t)calvals.T1)) * ((rawtemp>>4) - ((int32_t)calvals.T1))) >> 12) * ((int32_t)calvals.T3)) >> 14;
	t_fine = var1 + var2;
	temp_p = (t_fine * 5 + 128) >> 8;

	return (float)temp_p;
}

float recalc_bmp280Pressure(int32_t rawpress, int32_t rawtemp)
{
	int32_t press_p;
	int32_t t_fine;

		int32_t var1, var2;
		var1 = ((((rawtemp >> 3) - (((int32_t)calvals.T1) << 1))) * ((int32_t)calvals.T2)) >> 11;
		var2 = (((((rawtemp >> 4) - ((int32_t)calvals.T1)) * ((rawtemp>>4) - ((int32_t)calvals.T1))) >> 12) * ((int32_t)calvals.T3)) >> 14;
		t_fine = var1 + var2;

		int32_t var1_p, var2_p;
		uint32_t p;
		var1_p = (((int32_t)t_fine)>>1) - (int32_t)64000;
		var2_p = (((var1_p>>2) * (var1_p>>2)) >> 11 ) * ((int32_t)calvals.P6);
		var2_p = var2_p + ((var1_p*((int32_t)calvals.P5))<<1);
		var2_p = (var2_p>>2)+(((int32_t)calvals.P4)<<16);
		var1_p = (((calvals.P3 * (((var1_p>>2) * (var1_p>>2)) >> 13 )) >> 3) + ((((int32_t)calvals.P2) * var1_p)>>1))>>18;
		var1_p =((((32768+var1_p))*((int32_t)calvals.P1))>>15);
		if (var1_p == 0)
		{
		return -8; // чтобы не делить на ноль
		}
		p = (((uint32_t)(((int32_t)1048576)- rawpress)-(var2_p>>12)))*3125;
		if (p < 0x80000000)
		{
		p = (p << 1) / ((uint32_t)var1_p);
		}
		else
		{
		p = (p / (uint32_t)var1_p) * 2;
		}
		var1_p = (((int32_t)calvals.P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
		var2_p = (((int32_t)(p>>2)) * ((int32_t)calvals.P8))>>13;
		p = (uint32_t)((int32_t)p + ((var1_p + var2_p + calvals.P7) >> 4));
		press_p = p;


	return (float)press_p;
}

