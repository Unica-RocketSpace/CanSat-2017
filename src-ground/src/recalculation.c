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
#include <sofa.h>

#include "package_struct.h"
#include "recalculation.h"

static const char calvals_value[] = {0x61,0x6c, 0xde, 0x65, 0x18, 0xfc, 0xb7, 0x97, 0xfb, 0xd5, 0xd0, 0x0b,
		0x17, 0x0e, 0x50, 0x01, 0xf9, 0xff, 0x8c, 0x3c, 0xf8, 0xc6, 0x70, 0x17};

//static const rscs_bmp280_calibration_values_t * calvals = (const rscs_bmp280_calibration_values_t * )calvals_value;

static const rscs_bmp280_calibration_values_t calvals =
{
		.P1 = 0x6c61,
		.P2 = 0x65de,
		.P3 = 0xfc18,
		.P4 = 0x97fb,
		.P5 = 0xd5fb,
		.P6 = 0x0bd0,
		.P7 = 0x0e17,
		.P8 = 0x0150,
		.P9 = 0xfff9,
		.T1 = 0x3c8c,
		.T2 = 0xc6f8,
		.T3 = 0x1770
};

void print_calibration_values()
{
	printf("1: %X\n", calvals.P1);
	printf("2: %X\n", calvals.P2);
	printf("3: %X\n", calvals.P3);
	printf("4: %X\n", calvals.P4);
	printf("5: %X\n", calvals.P5);
	printf("6: %X\n", calvals.P6);
	printf("7: %X\n", calvals.P7);
	printf("8: %X\n", calvals.P8);
	printf("9: %X\n", calvals.P9);
	printf("10: %X\n", calvals.T1);
	printf("11: %X\n", calvals.T2);
	printf("12: %X\n", calvals.T3);
}

void recalc_accel(const int16_t * raw_accel_XYZ, float * accel_XYZ)
{
	accel_XYZ[0] = (float)(raw_accel_XYZ[0]) * MPU9255_ACCEL_SCALE_FACTOR * pow(2, ACCEL_RANGE) * X_ACCEL_KOEFF;
	accel_XYZ[1] = (float)(raw_accel_XYZ[1]) * MPU9255_ACCEL_SCALE_FACTOR * pow(2, ACCEL_RANGE) * Y_ACCEL_KOEFF;
	accel_XYZ[2] = (float)(raw_accel_XYZ[2]) * MPU9255_ACCEL_SCALE_FACTOR * pow(2, ACCEL_RANGE) * Z_ACCEL_KOEFF;
}


void recalc_gyro(const int16_t * raw_gyro_XYZ, float * gyro_XYZ)
{
	for (int i = 0; i < 3; i++)
		gyro_XYZ[i] = (float)(raw_gyro_XYZ[i]) * MPU9255_GYRO_SCALE_FACTOR * pow(2, GYRO_RANGE) * Z_GYRO_KOEFF;
}

void recalc_compass(const int16_t * raw_compass_XYZ, float * compass_XYZ)
{
	/*float x, y, z;

	float length = sqrt(pow(raw_compass_XYZ[0], 2) + pow(raw_compass_XYZ[1], 2) + pow(raw_compass_XYZ[2], 2));

	x = (float)raw_compass_XYZ[1] / length;
	y = (float)raw_compass_XYZ[0] / length;
	z = - (float)raw_compass_XYZ[2] / length;


	compass_XYZ[0] = x;
	compass_XYZ[1] = y;
	compass_XYZ[2] = z;*/
	float raw_data[3] = {(float)raw_compass_XYZ[0], (float)raw_compass_XYZ[1], (float)raw_compass_XYZ[2]};
	float offset_vector[3] = {X_COMPAS_OFFSET, Y_COMPAS_OFFSET, Z_COMPAS_OFFSET};
	float transform_matrix[3][3] =	{	{XX_COMPAS_TRANSFORM_MATIX, XY_COMPAS_TRANSFORM_MATIX, XZ_COMPAS_TRANSFORM_MATIX},
										{XY_COMPAS_TRANSFORM_MATIX, YY_COMPAS_TRANSFORM_MATIX, YZ_COMPAS_TRANSFORM_MATIX},
										{XZ_COMPAS_TRANSFORM_MATIX, YZ_COMPAS_TRANSFORM_MATIX, ZZ_COMPAS_TRANSFORM_MATIX}};

	//printf("raw_compass_XYZ = %d, %d, %d\n", raw_compass_XYZ[0], raw_compass_XYZ[1], raw_compass_XYZ[2]);
	//printf("raw_compass_XYZ = %f, %f, %f\n", (float)raw_compass_XYZ[0], (float)raw_compass_XYZ[1], (float)raw_compass_XYZ[2]);
	iauPmp(raw_data, offset_vector, compass_XYZ);
	//printf("offset_compass_XYZ = %f, %f, %f\n", compass_XYZ[0], compass_XYZ[1], compass_XYZ[2]);
	iauRxp(transform_matrix, compass_XYZ, compass_XYZ);
	//printf("transform_compass_XYZ = %f, %f, %f\n\n", compass_XYZ[0], compass_XYZ[1], compass_XYZ[2]);
}

float recalc_ds18b20Temp(uint16_t raw_temp)
{
	return raw_temp / 16.0f;
}

float recalc_bmp280Temp(int32_t rawtemp)
{
	return (float)rawtemp / 100;
}


float recalc_bmp280Pressure(int32_t rawpress)
{
	return (float)rawpress;
}

void recalc_adxl345(int16_t * raw_adxl345, float * adxl345)
{
	adxl345[0] = raw_adxl345[0] * 0.004 * 9.81;
	adxl345[1] = raw_adxl345[1] * 0.004 * 9.81;
	adxl345[2] = raw_adxl345[2] * 0.004 * 9.81;
}

/*float recalc_bmp280Temp(int32_t rawtemp)
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
 * float recalc_bmp280Pressure(int32_t rawpress, int32_t rawtemp)
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
}*/

