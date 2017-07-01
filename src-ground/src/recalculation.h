/*
 * recalculation.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#ifndef RECALCULATION_H_
#define RECALCULATION_H_


#define MPU9255_GYRO_SCALE_FACTOR	3.14159265358979323846 / (180 * 131)
#define MPU9255_ACCEL_SCALE_FACTOR	0.00055217

#define X_ACCEL_OFFSET		0x003F//( 0x7FC1 )
#define Y_ACCEL_OFFSET		0x003E//( 0x7FC2 )
#define Z_ACCEL_OFFSET		0x0067//( 0x7F99 )
#define X_ACCEL_KOEFF		1.0796414
#define Y_ACCEL_KOEFF		1.0833494
#define Z_ACCEL_KOEFF		1.0717212

#define X_GYRO_KOEFF		1
#define Y_GYRO_KOEFF		1
#define Z_GYRO_KOEFF		1

#define X_COMPAS_OFFSET		479.1609
#define Y_COMPAS_OFFSET		42.0570
#define Z_COMPAS_OFFSET		191.5415
#define XX_COMPAS_TRANSFORM_MATIX	0.004041
#define YY_COMPAS_TRANSFORM_MATIX	0.004061
#define ZZ_COMPAS_TRANSFORM_MATIX	0.003486
#define XY_COMPAS_TRANSFORM_MATIX	-0.000029
#define XZ_COMPAS_TRANSFORM_MATIX	-0.000166
#define YZ_COMPAS_TRANSFORM_MATIX	-0.000030

#define ACCEL_RANGE			1			//2g - 00, 4g - 01, 8g - 10, 16g - 11
#define GYRO_RANGE			0			//250degps - 00, 500degps - 01, 1000degps - 10, 2000degps - 11


typedef struct {
	uint16_t T1;
	int16_t T2, T3;
	uint16_t P1;
	int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} rscs_bmp280_calibration_values_t;

void print_calibration_values();

/*функция пересчета ускорений (в м/с^2)*/
void recalc_accel(const int16_t * raw_accel_XYZ, float * accel_XYZ);

/*функция пересчета угловых скоростей (в рад/с)*/
void recalc_gyro(const int16_t * raw_gyro_XYZ, float * gyro_XYZ);

/*функция пересчета показаний компаса, выдает направляющие косинусы вектора B с осями ИСК*/
void recalc_compass(const int16_t * raw_compass_XYZ, float * compass_XYZ);

float recalc_ds18b20Temp(uint16_t raw_temp);
float recalc_bmp280Temp(int32_t rawtemp);
float recalc_bmp280Pressure(int32_t rawpress);
void recalc_adxl345(int16_t * raw_adxl345, float * adxl345);


#endif /* RECALCULATION_H_ */
