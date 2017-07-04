/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include <math.h>
#include <sofa.h>
#include <stdlib.h>

#include "kinematic_unit.h"

state DEVICE_STATE;

void kinematicInit()
{
	state STATE_;
	STATE_.aRelatedXYZ[0] = 0;	STATE_.aRelatedXYZ[1] = 0;	STATE_.aRelatedXYZ[2] = 0;
	STATE_.aALT_XYZ[0]    = 0;	STATE_.aALT_XYZ[1]    = 0;	STATE_.aALT_XYZ[2]    = 0;
	STATE_.gRelatedXYZ[0] = 0;	STATE_.gRelatedXYZ[1] = 0;	STATE_.gRelatedXYZ[2] = 0;
	STATE_.cRelatedXYZ[0] = 0;	STATE_.cRelatedXYZ[1] = 0;	STATE_.cRelatedXYZ[2] = 0;

	STATE_.height = 0;
	STATE_.zero_pressure = 0;
	STATE_.pressure = 0;

	STATE_.temp_bmp280 = 0;		STATE_.temp_ds18b20 = 0;

	STATE_.a_XYZ[0] = 0;		STATE_.a_XYZ[1] = 0;		STATE_.a_XYZ[2] = 0;
	STATE_.a_XYZ_prev[0] = 0;	STATE_.a_XYZ_prev[1] = 0;	STATE_.a_XYZ_prev[2] = 0;
	STATE_.v_XYZ[0] = 0;		STATE_.v_XYZ[1] = 0;		STATE_.v_XYZ[2] = 0;
	STATE_.v_XYZ_prev[0] = 0;	STATE_.v_XYZ_prev[1] = 0;	STATE_.v_XYZ_prev[2] = 0;
	STATE_.s_XYZ[0] = 0;		STATE_.s_XYZ[1] = 0;		STATE_.s_XYZ[2] = 0;
	STATE_.w_XYZ[0] = 0;		STATE_.w_XYZ[1] = 0;		STATE_.w_XYZ[2] = 0;
	STATE_.w_XYZ_prev[0] = 0;	STATE_.w_XYZ_prev[1] = 0;	STATE_.w_XYZ_prev[2] = 0;

	STATE_.f_XYZ[0][0] = 1;		STATE_.f_XYZ[0][1] = 0;		STATE_.f_XYZ[0][2] = 0;
	STATE_.f_XYZ[1][0] = 0;		STATE_.f_XYZ[1][1] = 1;		STATE_.f_XYZ[1][2] = 0;
	STATE_.f_XYZ[2][0] = 0;		STATE_.f_XYZ[2][1] = 0;		STATE_.f_XYZ[2][2] = 1;

	STATE_.f_XYZ_prev[0][0] = 1;	STATE_.f_XYZ_prev[0][1] = 0;	STATE_.f_XYZ_prev[0][2] = 0;
	STATE_.f_XYZ_prev[1][0] = 0;	STATE_.f_XYZ_prev[1][1] = 1;	STATE_.f_XYZ_prev[1][2] = 0;
	STATE_.f_XYZ_prev[2][0] = 0;	STATE_.f_XYZ_prev[2][1] = 0;	STATE_.f_XYZ_prev[2][2] = 1;

	STATE_.B_XYZ[0] = 0;		STATE_.B_XYZ[1] = 0;		STATE_.B_XYZ[2] = 0;

	STATE_.state = 0;

	STATE_.Time = 0;
	STATE_.previousTime = 0;

	DEVICE_STATE = STATE_;
}


void RSC_to_ISC_recalc(float * RSC_vect, float * ISC_vect)
{
	iauRxp(DEVICE_STATE.f_XYZ, RSC_vect, ISC_vect);
}

/*void set_ISC_offset()
{
	float dummy1;
	int16_t dummy2;
	int16_t accel_raw_XYZ[3];
	float accel_XYZ[3];

	float x1_unit_vect[3] = {1, 0, 0};
	float x_vect[3], x_unit_vect[3], y_vect[3], y_unit_vect[3], g_unit_vect[3];

	MPU9255_recalc_accel(STA, accel_XYZ);

	iauPn(accel_XYZ, &dummy1, g_unit_vect);

	//float g_vect = sqrt(pow(*(first_accel_XYZ + 0), 2) + pow(*(first_accel_XYZ + 1), 2) + pow(*(first_accel_XYZ + 2), 2));

	DEVICE_STATE.f_XYZ[2][0] = g_unit_vect[0];
	DEVICE_STATE.f_XYZ[2][1] = g_unit_vect[1];
	DEVICE_STATE.f_XYZ[2][2] = g_unit_vect[2];


	iauPxp(g_unit_vect, x1_unit_vect, y_vect);
	iauPn(y_vect, &dummy1, y_unit_vect);

	iauPxp(y_unit_vect, g_unit_vect, x_vect);
	iauPn(x_vect, &dummy1, x_unit_vect);

	DEVICE_STATE.f_XYZ[0][0] = x_unit_vect[0];
	DEVICE_STATE.f_XYZ[0][1] = x_unit_vect[1];
	DEVICE_STATE.f_XYZ[0][2] = x_unit_vect[2];

	DEVICE_STATE.f_XYZ[1][0] = y_unit_vect[0];
	DEVICE_STATE.f_XYZ[1][1] = y_unit_vect[1];
	DEVICE_STATE.f_XYZ[1][2] = y_unit_vect[2];

}
*/
/*
void set_magn_dir()
{
	rscs_e error1 = 1;
	int16_t compass_raw_XYZ[3];
	float compass_XYZ[3];
	float dummy1;
	float B_unit_vect[3];

	MPU9255_read_compass(compass_raw_XYZ);
	MPU9255_recalc_compass(compass_raw_XYZ, DEVICE_STATE.cRelatedXYZ);

	while (error1 != 0)
	{
		//iauPn(compass_XYZ, &dummy1, B_unit_vect);

		RSC_to_ISC_recalc(B_unit_vect, DEVICE_STATE.B_XYZ);
		iauPn(DEVICE_STATE.B_XYZ, &dummy1, DEVICE_STATE.B_XYZ);

		error1 = MPU9255_read_compass(compass_raw_XYZ);
		MPU9255_recalc_compass(compass_raw_XYZ, DEVICE_STATE.cRelatedXYZ);
	}
}*/

void recalc_ISC()
{
	float compass_XYZ[3];
	float dummy1;
	float B_vect[3], B_unit_vect[3], C_vect[3], C_unit_vect[3], A_unit_vect[3];		//C - ось вращения, A - третий вектор системы BAC

	for (int i = 0; i < 3; i++)		//переписываем в compass_XYZ показания магнитометра из STATE
	{
		compass_XYZ[i] = DEVICE_STATE.cRelatedXYZ[i];
	}

	RSC_to_ISC_recalc(compass_XYZ, B_vect);		//пересчет вектора магнитного поля в ИСК
	iauPn(B_vect, &dummy1, B_unit_vect);
	iauPxp(DEVICE_STATE.B_XYZ, B_unit_vect, C_vect);	//создаем ось вращения С, перпендикулярную плоскости (M,M1)
	iauPn(C_vect, &dummy1, C_unit_vect);		//нормируем вектор С

	iauPxp(C_unit_vect, DEVICE_STATE.B_XYZ, A_unit_vect);	//находим третий вектор (А) системы координат BАС
	iauPn(A_unit_vect, &dummy1, A_unit_vect);

	//создание матрицы перехода (М) (BАС->ИСК) и (М) транспонированной
	float M[3][3] = {	{B_unit_vect[0], A_unit_vect[0], C_unit_vect[0]},
						{B_unit_vect[1], A_unit_vect[1], C_unit_vect[1]},
						{B_unit_vect[2], A_unit_vect[2], C_unit_vect[2]}	};
	float M_1[3][3];

	iauTr(M, M_1);		//Транспонирование матрицы (М)
	float cosangle = iauPdp(B_unit_vect, DEVICE_STATE.B_XYZ);		//находим косинус угла между векторами В(старый) и B1(новый)
	float sinangle = sin(acos(cosangle));					//находим синус

	//создание матрицы поворота (R)
	float R[3][3] = {	{cosangle,	-sinangle,	0},
						{sinangle,	cosangle,	0},
						{0,			0,			1}	};
	//создание матрицы трансформации (Т) - переход, поворот, переход
	float T[3][3];
	//Т = M_1 * R * M (в два этапа)
	iauRxr(M_1, R, T);
	iauRxr(T, M, T);

	//переписываем DEVICE_STATE.f_XYZ
	iauRxr(T, DEVICE_STATE.f_XYZ, DEVICE_STATE.f_XYZ);
}
/*
void set_zero_pressure()
{
	int32_t raw_pressure32, raw_temp32, pressure32, temp32;
	rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);

	const rscs_bmp280_calibration_values_t * calibrate_values_ = rscs_bmp280_get_calibration_values(bmp280);
	rscs_bmp280_calculate(calibrate_values_, raw_pressure32, raw_temp32, &pressure32, &temp32);

	DEVICE_STATE.zero_pressure = (float)pressure32;
}

void pressure_read_recon(int32_t * pressure32, int32_t * temp32, float * height, float * temp)
{
	int32_t raw_pressure32, raw_temp32;
	rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);


	calibrate_values = rscs_bmp280_get_calibration_values(bmp280);
	rscs_bmp280_calculate(calibrate_values, raw_pressure32, raw_temp32, pressure32, temp32);

	DEVICE_STATE.pressure = (float)*pressure32;
	*height = 18.4 * log(DEVICE_STATE.zero_pressure / DEVICE_STATE.pressure);
	*temp = (float)*temp32 / 100;
}
*/
void pull_recon_data()
{
	//опрос MPU9255 и пересчет показаний
	//MPU9255_read_imu(TRANSMIT_DATA.aTransmitXYZ, TRANSMIT_DATA.gTransmitXYZ);
	//MPU9255_read_compass(TRANSMIT_DATA.cTransmitXYZ);
	//printf("compas_error: %d\n", error);

	//MPU9255_recalc_accel((int16_t*)(pack->aXYZ), DEVICE_STATE.aRelatedXYZ);
	//MPU9255_recalc_gyro((int16_t*)(pack->gXYZ), DEVICE_STATE.gRelatedXYZ);
	//MPU9255_recalc_compass((int16_t*)(pack->cXYZ), DEVICE_STATE.cRelatedXYZ);
	/*=====================================================================*/

	//Применение фильтра Калмана
	//apply_KalmanFilter(DEVICE_STATE.aRelatedXYZ, DEVICE_STATE.aRelatedXYZ_prev, ACCEL_KALMAN_GAIN, 3);
	//apply_KalmanFilter(DEVICE_STATE.gRelatedXYZ, DEVICE_STATE.gRelatedXYZ_prev, GYRO_KALMAN_GAIN, 3);
	/*=====================================================================*/

	//Фильтрация шума
	apply_NoiseFilter(DEVICE_STATE.gRelatedXYZ, GYRO_NOISE, 3);
	apply_NoiseFilter(DEVICE_STATE.aRelatedXYZ, ACCEL_NOISE, 3);
	/*=====================================================================*/

	//опрос барометра bmp280
	//pressure_read_recon(&TRANSMIT_DATA.pressure, &TRANSMIT_DATA.temp_bmp280, &DEVICE_STATE.height, &DEVICE_STATE.temp_bmp280);

	/*=====================================================================*/

	//опрос термометра ds18b20
	//if (rscs_ds18b20_check_ready())		//проверяем, готовы ли данные
	{
		//rscs_ds18b20_read_temperature(ds18b20, &TRANSMIT_DATA.temp_ds18b20);	//записываем температуру
		//DEVICE_STATE.temp_ds18b20 = rscs_ds18b20_count_temperature(ds18b20, TRANSMIT_DATA.temp_ds18b20);
		//rscs_ds18b20_start_conversion(ds18b20);
	}
	/*=====================================================================*/

	//опрос акселерометра ADXL345
	//rscs_adxl345_GetGXYZ(adxl345, &TRANSMIT_DATA.ADXL_transmit[0], &TRANSMIT_DATA.ADXL_transmit[1], &TRANSMIT_DATA.ADXL_transmit[2],
	//								&DEVICE_STATE.aALT_XYZ[0], &DEVICE_STATE.aALT_XYZ[1], &DEVICE_STATE.aALT_XYZ[2]);

	//DEVICE_STATE.Time = rscs_time_get();
}


void set_cos_to_1(float * cosalpha)
{
	if (*cosalpha > 1.0)	{*cosalpha = 1.0;}
	if (*cosalpha < -1.0)	{*cosalpha = -1.0;}

}

void construct_trajectory(float G_)
{

	float dt = (float)(DEVICE_STATE.Time - DEVICE_STATE.previousTime) / 1000;

	//определение угловых скоростей (в ИСК)
	RSC_to_ISC_recalc(DEVICE_STATE.gRelatedXYZ, DEVICE_STATE.w_XYZ);

	solveByRungeKutta((float)(DEVICE_STATE.Time - DEVICE_STATE.previousTime) / 1000, DEVICE_STATE.f_XYZ_prev[0], DEVICE_STATE.f_XYZ[0]);
	solveByRungeKutta((float)(DEVICE_STATE.Time - DEVICE_STATE.previousTime) / 1000, DEVICE_STATE.f_XYZ_prev[1], DEVICE_STATE.f_XYZ[1]);

	//находим третью строку матрицы поворота из условия ортогональности векторов
	iauPxp(DEVICE_STATE.f_XYZ[0], DEVICE_STATE.f_XYZ[1], DEVICE_STATE.f_XYZ[2]);

	set_cos_to_1(&DEVICE_STATE.f_XYZ[0][0]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[0][1]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[0][2]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[1][0]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[1][1]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[1][2]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[2][0]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[2][1]);
	set_cos_to_1(&DEVICE_STATE.f_XYZ[2][2]);

	//определение ускорений	RSC_to_ISC_recalc(DEVICE_STATE.aRelatedXYZ, DEVICE_STATE.a_XYZ);
	//DEVICE_STATE.a_XYZ[2] = DEVICE_STATE.a_XYZ[2] - G_VECT;
	DEVICE_STATE.a_XYZ[2] = DEVICE_STATE.a_XYZ[2] - G_;

	//определение скоростей
	DEVICE_STATE.v_XYZ[0] = DEVICE_STATE.v_XYZ[0] + (DEVICE_STATE.a_XYZ_prev[0] + DEVICE_STATE.a_XYZ[0]) * dt / 2;
	DEVICE_STATE.v_XYZ[1] = DEVICE_STATE.v_XYZ[1] + (DEVICE_STATE.a_XYZ_prev[1] + DEVICE_STATE.a_XYZ[1]) * dt / 2;
	DEVICE_STATE.v_XYZ[2] = DEVICE_STATE.v_XYZ[2] + (DEVICE_STATE.a_XYZ_prev[2] + DEVICE_STATE.a_XYZ[2]) * dt / 2;


	//расчет перемещений
	DEVICE_STATE.s_XYZ[0] = DEVICE_STATE.s_XYZ[0] + (DEVICE_STATE.v_XYZ_prev[0] + DEVICE_STATE.v_XYZ[0]) * dt / 2;
	DEVICE_STATE.s_XYZ[1] = DEVICE_STATE.s_XYZ[1] + (DEVICE_STATE.v_XYZ_prev[1] + DEVICE_STATE.v_XYZ[1]) * dt / 2;
	DEVICE_STATE.s_XYZ[2] = DEVICE_STATE.s_XYZ[2] + (DEVICE_STATE.v_XYZ_prev[2] + DEVICE_STATE.v_XYZ[2]) * dt / 2;

	for (int i = 0; i < 3; i++)
	{
		DEVICE_STATE.a_XYZ_prev[i] = DEVICE_STATE.a_XYZ[i];
		DEVICE_STATE.v_XYZ_prev[i] = DEVICE_STATE.v_XYZ[i];
		DEVICE_STATE.w_XYZ_prev[i] = DEVICE_STATE.w_XYZ[i];
		DEVICE_STATE.aRelatedXYZ_prev[i] = DEVICE_STATE.aRelatedXYZ[i];
		DEVICE_STATE.gRelatedXYZ_prev[i] = DEVICE_STATE.gRelatedXYZ[i];
		for (int j = 0; j < 3; j++)
		{
			DEVICE_STATE.f_XYZ_prev[i][j] = DEVICE_STATE.f_XYZ[i][j];
		}
	}
	DEVICE_STATE.previousTime = DEVICE_STATE.Time;
}

void apply_KalmanFilter(float * sensor_data, const float * sensor_data_prev, float Kalman_gain, int data_array_size)
{
	for (int i = 0; i < data_array_size; i++)
		sensor_data[i] = Kalman_gain * sensor_data[i] + (1 - Kalman_gain) * sensor_data_prev[i];
}

void apply_NoiseFilter(float * sensor_data, float noise, int data_array_size)
{
	for (int i = 0; i < data_array_size; i++)
	{
		if (fabsf(sensor_data[i]) < noise)
				sensor_data[i] = 0;
	}
}

inline float functionForRK(uint8_t i, float * y)
{
	switch (i) {
		case 0:
			return y[1] * DEVICE_STATE.w_XYZ_prev[2] - y[2] * DEVICE_STATE.w_XYZ_prev[1];
		case 1:
			return y[2] * DEVICE_STATE.w_XYZ_prev[0] - y[0] * DEVICE_STATE.w_XYZ_prev[2];
		case 2:
			return y[0] * DEVICE_STATE.w_XYZ_prev[1] - y[1] * DEVICE_STATE.w_XYZ_prev[0];
	}
	return 0;
}

void solveByRungeKutta(float dt, float * y, float * y_new)
{
	float k1[3], k2[3], k3[3], k4[3], y1[3], y2[3], y3[3];

	for (int i = 0; i < 3; i++)
		k1[i] = functionForRK(i, y);

	for (int i = 0; i < 3; i++)
		y1[i] = y[i] + (float)(k1[i] * dt / 2);

	for (int i = 0; i < 3; i++)
		k2[i] = functionForRK(i, y1);

	for (int i = 0; i < 3; i++)
		y2[i] = y[i] + (float)(k2[i] * dt / 2);

	for (int i = 0; i < 3; i++)
		k3[i] = functionForRK(i, y2);

	for (int i = 0; i < 3; i++)
		y3[i] = y[i] + (float)(k3[i] * dt);

	for (int i = 0; i < 3; i++)
		k4[i] = functionForRK(i, y3);

	for (int i = 0; i < 3; i++)
		y_new[i] = y[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) *  dt / 6;
}
