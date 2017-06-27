/*
 * recalculation.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#ifndef RECALCULATION_H_
#define RECALCULATION_H_

#define ACCEL_RANGE 4
#define ACCEL_SCALE_FACTOR 0.0062217
#define GYRO_RANGE 2
#define GYRO_SCALE_FACTOR 4

typedef struct {
	uint16_t T1;
	int16_t T2, T3;
	uint16_t P1;
	int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} rscs_bmp280_calibration_values_t;

void print_calibration_values();

/*функция пересчета ускорений (в м/с^2)*/
void recalc_accel(int16_t * raw_accel_XYZ, float * accel_XYZ);

/*функция пересчета угловых скоростей (в рад/с)*/
void recalc_gyro(int16_t * raw_gyro_XYZ, float * gyro_XYZ);

/*функция пересчета показаний компаса, выдает направляющие косинусы вектора B с осями ИСК*/
void recalc_compass(int16_t * raw_compass_XYZ, float * compass_XYZ);

float recalc_ds18b20Temp(uint16_t raw_temp);
float recalc_bmp280Temp(int32_t rawtemp);
float recalc_bmp280Pressure(int32_t rawpress);


#endif /* RECALCULATION_H_ */
