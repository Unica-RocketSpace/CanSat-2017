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

