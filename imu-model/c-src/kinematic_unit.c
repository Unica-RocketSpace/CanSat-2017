/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include <stdio.h>
#include <math.h>
#include "rscs/error.h"
#include <sofa/sofa.h>
//#include "rscs/timeservice.h"
//#include "rscs/ds18b20.h"				//ДРАЙВЕР ДАТЧИКА ТЕМПЕРАТУРЫ DS18B20
//#include <rscs/bmp180.h>				//ДРАЙВЕР ДАТЧИКА ДАВЛЕНИЯ BMP180

//#include "ADXL345.h"					//ДРАЙВЕР АКСЕЛЕРОМЕТРА	ADXL345
//#include "MPU9255.h"					//ДРАЙВЕР ГИРОСКОПА, АКСЕЛЕРОМЕТРА и КОМПАСА (MPU9255)
#include "kinematic_unit.h"
#include "rscs/error.h"

state STATE;
transmit_data TRANSMIT_DATA;

void kinematicInit(float * a,float w,float * s, float * v)
{
	//w = w * 2 * M_PI;

	state STATE_ = {	{a[0], a[1], a[2]},	//ускорения в единицах g (в ССК)
						{0, 0, w},			//угловые скорости в degps (в ССК)
						{0},				//косинусы углов вектора магнитного поля с осями ССК
						 0,					//давление

						{0, 0, 0},			//ускорения в м/с^2 (ИСК)
						{0, 0, 0},			//ускорения в м/с^2 (ИСК) предыдущие
						{v[0], v[1], v[2]},	//скорости  в м/с   (ИСК)
						{v[0], v[1], v[2]},	//скорости  в м/с   (ИСК) предыдущие
						{s[0], s[1], s[2]},	//перемещения в м   (ИСК)
						{0, 0, 0},			//угловые скорости в 1/с (ИСК)
						{0, 0, 0},			//угловые скорости в 1/с (ИСК) предыдущие

					{	{1, 0, 0},			//косинусы углов оси Х ИСК с осями ССК
						{0, 1, 0},			//косинусы углов оси Y ИСК с осями ССК
					 	{0, 0, 1}	},		//косинусы углов оси Z ИСК с осями ССК
					{	{1, 0, 0},			//косинусы углов оси Х ИСК с осями ССК предыдущие
						{0, 1, 0},			//косинусы углов оси Y ИСК с осями ССК предыдущие
						{0, 0, 1}	},		//косинусы углов оси Z ИСК с осями ССК предыдущие
					 	{0, 0, 0},
					 	 0,					//время текущей итерации
					 	 0					//время предыдущей итерации
					};

	STATE = STATE_;

	transmit_data TRANSMIT_DATA_ = {	{0},
									{0},
									{0}
	};

	TRANSMIT_DATA = TRANSMIT_DATA_;

}


void RSC_to_ISC_recalc(float * RSC_vect, float * ISC_vect)
{
	ISC_vect[0] = RSC_vect[0] * STATE.f_XYZ[0][0] + RSC_vect[1] * STATE.f_XYZ[0][1] + RSC_vect[2] * STATE.f_XYZ[0][2];
	ISC_vect[1] = RSC_vect[0] * STATE.f_XYZ[1][0] + RSC_vect[1] * STATE.f_XYZ[1][1] + RSC_vect[2] * STATE.f_XYZ[1][2];
	ISC_vect[2] = RSC_vect[0] * STATE.f_XYZ[2][0] + RSC_vect[1] * STATE.f_XYZ[2][1] + RSC_vect[2] * STATE.f_XYZ[2][2];
}

/*
void set_ISC_offset()
{
	rscs_e error1, error2;
	float dummy1;
	int16_t dummy2;
	int16_t accel_raw_XYZ[3];
	float accel_XYZ[3];

	float x1_unit_vect[3] = {1, 0, 0};
	float x_vect[3], x_unit_vect[3], y_vect[3], y_unit_vect[3], g_unit_vect[3];


	error1 = MPU9255_read_imu(accel_raw_XYZ, &dummy2);
	error2 = MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);

	while ((error1 & error2) != 0)
	{
		iauPn(accel_XYZ, &dummy1, g_unit_vect);

		//float g_vect = sqrt(pow(*(first_accel_XYZ + 0), 2) + pow(*(first_accel_XYZ + 1), 2) + pow(*(first_accel_XYZ + 2), 2));

		STATE.f_XYZ[2][0] = - g_unit_vect[0];
		STATE.f_XYZ[2][1] = - g_unit_vect[1];
		STATE.f_XYZ[2][2] = - g_unit_vect[2];

		error1 = MPU9255_read_imu(accel_raw_XYZ, &dummy2);
		error2 = MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);

	}

	iauPxp(x1_unit_vect, g_unit_vect, y_vect);
	iauPn(y_vect, &dummy1, y_unit_vect);

	iauPxp(g_unit_vect, y_unit_vect, x_vect);
	iauPn(x_vect, &dummy1, x_unit_vect);

	STATE.f_XYZ[0][0] = x_unit_vect[0];
	STATE.f_XYZ[0][1] = x_unit_vect[1];
	STATE.f_XYZ[0][2] = x_unit_vect[2];

	STATE.f_XYZ[1][0] = y_unit_vect[0];
	STATE.f_XYZ[1][1] = y_unit_vect[1];
	STATE.f_XYZ[1][2] = y_unit_vect[2];

}


void set_magn_dir()
{
	rscs_e error1, error2;
	int16_t compass_raw_XYZ[3];
	float compass_XYZ[3];
	float dummy1;
	float B_unit_vect[3];

	error1 = MPU9255_read_compass(compass_raw_XYZ);
	error2 = MPU9255_recalc_compass(compass_raw_XYZ, compass_XYZ);

	while ((error1 & error2) != 0)
	{
		iauPn(compass_XYZ, &dummy1, B_unit_vect);

		RSC_to_ISC_recalc(B_unit_vect, STATE.B_XYZ);

		error1 = MPU9255_read_compass(compass_raw_XYZ);
		error2 = MPU9255_recalc_compass(compass_raw_XYZ, compass_XYZ);
	}
}


void recalc_ISC()
{
	float m1_vect[3 ], m1_unit_vect[3];

	rscs_e error1, error2;
	int16_t compass_raw_XYZ[3];
	float compass_XYZ[3];
	float dummy1;
	float B_vect[3], B_unit_vect[3], C_vect[3], C_unit_vect[3], A_unit_vect[3];		//C - ось вращения, A - третий вектор системы BAC

	error1 = MPU9255_read_compass(compass_raw_XYZ);
	error2 = MPU9255_recalc_compass(compass_raw_XYZ, compass_XYZ);


	RSC_to_ISC_recalc(compass_XYZ, B_vect);		//пересчет вектора магнитного поля в ИСК
	iauPxp(STATE.B_XYZ, B_unit_vect, C_vect);	//создаем ось вращения С, перпендикулярную плоскости (M,M1)
	iauPn(C_vect, &dummy1, C_unit_vect);		//нормируем вектор С

	iauPxp(C_unit_vect, STATE.B_XYZ, A_unit_vect);	//находим третий вектор (А) системы координат BАС

	//создание матрицы перехода (М) (BАС->ИСК) и (М) транспонированной
	float M[3][3] = {	{B_unit_vect[0], A_unit_vect[0], C_unit_vect[0]},
						{B_unit_vect[1], A_unit_vect[1], C_unit_vect[1]},
						{B_unit_vect[2], A_unit_vect[2], C_unit_vect[2]}	};
	float M_1[3][3];

	iauTr(M, M_1);		//Транспонирование матрицы (М)
	float cosangle = iauPdp(B_unit_vect, STATE.B_XYZ);		//находим косинус угла между векторами В(старый) и B1(новый)
	float sinangle = sin(acos(cosangle));					//находим синус

	//создание матрицы поворота (R)
	float R[3][3] = {	{cosangle,		sinangle,	0},
						{- sinangle,	cosangle,	0},
						{0,				0,			1}	};
	//создание матрицы трансформации (Т) - переход, поворот, переход
	float T[3][3];
	//Т = M_1 * R * M (в два этапа)
	iauRxr(M_1, R, T);
	iauRxr(T, M, T);

	//переписываем STATE.f_XYZ
	iauRxr(T, STATE.f_XYZ, STATE.f_XYZ);

}


void recon_AGC_STATE_TRANSMIT_DATA()
{

	MPU9255_read_imu(TRANSMIT_DATA.aTransmitXYZ, TRANSMIT_DATA.gTransmitXYZ);
	MPU9255_read_compass(TRANSMIT_DATA.cTransmitXYZ);

	MPU9255_recalc_accel(TRANSMIT_DATA.aTransmitXYZ, STATE.aRelatedXYZ);
	MPU9255_recalc_gyro(TRANSMIT_DATA.gTransmitXYZ, STATE.gRelatedXYZ);
	MPU9255_recalc_compass(TRANSMIT_DATA.cTransmitXYZ, STATE.cRelatedXYZ);

	//TODO: написать опрос барометра

	STATE.Time = rscs_time_get();

}

void calculate_height()
{
	STATE.s_XYZ[2] = 44330 * (1 - pow((STATE.pressure / ZERO_PRESSURE), 1/5.225));
}
*/

void set_cos_to_1(float * cosalpha)
{
	if (*cosalpha > 1.0)
	{
		*cosalpha = 1.0;
	}
}


void trajectoryConstruction(float time)
{

	float dt = time - STATE.previous_time;

	//определение угловых скоростей (в ИСК)


	//определение углов между осями ИСК и ССК
	rotation_matrix ROT_M = {0, 0, 0};
/*
	ROT_M.f_XYZ[0][0] = STATE.f_XYZ[0][0] + (STATE.f_XYZ[0][1] * STATE.w_XYZ[2] - STATE.f_XYZ[0][2] * STATE.w_XYZ[1]) * dt;
	ROT_M.f_XYZ[0][1] = STATE.f_XYZ[0][1] + (STATE.f_XYZ[0][2] * STATE.w_XYZ[0] - STATE.f_XYZ[0][0] * STATE.w_XYZ[2]) * dt;
	ROT_M.f_XYZ[0][2] = STATE.f_XYZ[0][2] + (STATE.f_XYZ[0][0] * STATE.w_XYZ[1] - STATE.f_XYZ[0][1] * STATE.w_XYZ[0]) * dt;
	ROT_M.f_XYZ[1][0] = STATE.f_XYZ[1][0] + (STATE.f_XYZ[1][1] * STATE.w_XYZ[2] - STATE.f_XYZ[1][2] * STATE.w_XYZ[1]) * dt;
	ROT_M.f_XYZ[1][1] = STATE.f_XYZ[1][1] + (STATE.f_XYZ[1][2] * STATE.w_XYZ[0] - STATE.f_XYZ[1][0] * STATE.w_XYZ[2]) * dt;
	ROT_M.f_XYZ[1][2] = STATE.f_XYZ[1][2] + (STATE.f_XYZ[1][0] * STATE.w_XYZ[1] - STATE.f_XYZ[1][1] * STATE.w_XYZ[0]) * dt;

	ROT_M.f_XYZ[2][2] =  ROT_M.f_XYZ[0][0] * ROT_M.f_XYZ[1][1] - ROT_M.f_XYZ[0][1] * ROT_M.f_XYZ[1][0];
	ROT_M.f_XYZ[2][0] = (ROT_M.f_XYZ[0][0] * ROT_M.f_XYZ[0][2] + ROT_M.f_XYZ[1][0] * ROT_M.f_XYZ[1][2]) / ROT_M.f_XYZ[2][2];
	ROT_M.f_XYZ[2][1] = (ROT_M.f_XYZ[0][1] * ROT_M.f_XYZ[0][2] + ROT_M.f_XYZ[1][1] * ROT_M.f_XYZ[1][2]) / ROT_M.f_XYZ[2][2];
*/

	/*float k1, k2;
	k1 =(STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [0][1] * STATE.w_XYZ     [2] - STATE.f_XYZ     [0][2] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[0][0] = STATE.f_XYZ_prev[0][0] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [0][2] * STATE.w_XYZ     [0] - STATE.f_XYZ     [0][0] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[0][1] = STATE.f_XYZ_prev[0][1] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [0][0] * STATE.w_XYZ     [1] - STATE.f_XYZ     [0][1] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[0][2] = STATE.f_XYZ_prev[0][2] + (k1 + k2) / 2;

	printf("f_XYZ[0] = %f\n", sqrt(pow(STATE.f_XYZ[0][0],2) + pow(STATE.f_XYZ[0][1],2) + pow(STATE.f_XYZ[0][2],2)));

	k1 =(STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [1][1] * STATE.w_XYZ     [2] - STATE.f_XYZ     [1][2] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[1][0] = STATE.f_XYZ_prev[1][0] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [1][2] * STATE.w_XYZ     [0] - STATE.f_XYZ     [1][0] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[1][1] = STATE.f_XYZ_prev[1][1] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [1][0] * STATE.w_XYZ     [1] - STATE.f_XYZ     [1][1] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[1][2] = STATE.f_XYZ_prev[1][2] + (k1 + k2) / 2;

	printf("f_XYZ[1] = %f\n", sqrt(pow(STATE.f_XYZ[1][0],2) + pow(STATE.f_XYZ[1][1],2) + pow(STATE.f_XYZ[1][2],2)));

	k1 =(STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][2] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [2][1] * STATE.w_XYZ     [2] - STATE.f_XYZ     [2][2] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[2][0] = STATE.f_XYZ_prev[2][0] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[2][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [2][2] * STATE.w_XYZ     [0] - STATE.f_XYZ     [2][0] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[2][1] = STATE.f_XYZ_prev[2][1] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [2][0] * STATE.w_XYZ     [1] - STATE.f_XYZ     [2][1] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[2][2] = STATE.f_XYZ_prev[2][2] + (k1 + k2) / 2;

	printf("f_XYZ[2] = %f\n", sqrt(pow(STATE.f_XYZ[2][0],2) + pow(STATE.f_XYZ[2][1],2) + pow(STATE.f_XYZ[2][2],2)));
*/

	/*float k1, k2;
	k1 =(STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [1][0] * STATE.w_XYZ     [2] - STATE.f_XYZ     [2][0] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[0][0] = STATE.f_XYZ_prev[0][0] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [2][0] * STATE.w_XYZ     [0] - STATE.f_XYZ     [0][0] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[1][0] = STATE.f_XYZ_prev[1][0] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [0][0] * STATE.w_XYZ     [1] - STATE.f_XYZ     [1][0] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[2][0] = STATE.f_XYZ_prev[2][0] + (k1 + k2) / 2;

	printf("f_XYZ[0] = %f\n", sqrt(pow(STATE.f_XYZ[0][0],2) + pow(STATE.f_XYZ[1][0],2) + pow(STATE.f_XYZ[2][0],2)));

	k1 =(STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [1][1] * STATE.w_XYZ     [2] - STATE.f_XYZ     [2][1] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[0][1] = STATE.f_XYZ_prev[0][1] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [2][1] * STATE.w_XYZ     [0] - STATE.f_XYZ     [0][1] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[1][1] = STATE.f_XYZ_prev[1][1] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [0][1] * STATE.w_XYZ     [1] - STATE.f_XYZ     [1][1] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[2][1] = STATE.f_XYZ_prev[2][1] + (k1 + k2) / 2;

	printf("f_XYZ[1] = %f\n", sqrt(pow(STATE.f_XYZ[0][1],2) + pow(STATE.f_XYZ[1][1],2) + pow(STATE.f_XYZ[2][1],2)));

	k1 =(STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][2] * STATE.w_XYZ_prev[1]) * dt;
	k2 =(STATE.f_XYZ     [1][2] * STATE.w_XYZ     [2] - STATE.f_XYZ     [2][2] * STATE.w_XYZ     [1]) * dt;
	STATE.f_XYZ[0][2] = STATE.f_XYZ_prev[0][2] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[2][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[2]) * dt;
	k2 =(STATE.f_XYZ     [2][2] * STATE.w_XYZ     [0] - STATE.f_XYZ     [0][2] * STATE.w_XYZ     [2]) * dt;
	STATE.f_XYZ[1][2] = STATE.f_XYZ_prev[1][2] + (k1 + k2) / 2;

	k1 =(STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[0]) * dt;
	k2 =(STATE.f_XYZ     [0][2] * STATE.w_XYZ     [1] - STATE.f_XYZ     [1][2] * STATE.w_XYZ     [0]) * dt;
	STATE.f_XYZ[2][2] = STATE.f_XYZ_prev[2][2] + (k1 + k2) / 2;

	printf("f_XYZ[2] = %f\n", sqrt(pow(STATE.f_XYZ[0][2],2) + pow(STATE.f_XYZ[1][2],2) + pow(STATE.f_XYZ[2][2],2)));
*/

	float free_vector[3], solution_vector[3];
	float components_matrix[3][3];
	//находим первый столбец матрицы поворота
	free_vector[0] = STATE.f_XYZ[0][0] + (STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[1]) * dt / 2;
	free_vector[1] = STATE.f_XYZ[1][0] + (STATE.f_XYZ_prev[2][0] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[2]) * dt / 2;
	free_vector[2] = STATE.f_XYZ[2][0] + (STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[0]) * dt / 2;

	components_matrix[0][0] =   1;
	components_matrix[0][1] = - STATE.w_XYZ[2] * dt / 2;
	components_matrix[0][2] =   STATE.w_XYZ[1] * dt / 2;
	components_matrix[1][0] =   STATE.w_XYZ[2] * dt / 2;
	components_matrix[1][1] =   1;
	components_matrix[1][2] = - STATE.w_XYZ[0] * dt / 2;
	components_matrix[2][0] = - STATE.w_XYZ[1] * dt / 2;
	components_matrix[2][1] =   STATE.w_XYZ[0] * dt / 2;
	components_matrix[2][2] =   1;

	solveSystemByKramer(*components_matrix, free_vector, solution_vector);
	STATE.f_XYZ[0][0] = solution_vector[0];
	STATE.f_XYZ[1][0] = solution_vector[1];
	STATE.f_XYZ[2][0] = solution_vector[2];
	printf("f_XYZ[0] = %f\n", sqrt(pow(STATE.f_XYZ[0][0],2) + pow(STATE.f_XYZ[1][0],2) + pow(STATE.f_XYZ[2][0],2)));


	//находим второй столбец матрицы поворота
	free_vector[0] = STATE.f_XYZ[0][1] + (STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[1]) * dt / 2;
	free_vector[1] = STATE.f_XYZ[1][1] + (STATE.f_XYZ_prev[2][1] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[2]) * dt / 2;
	free_vector[2] = STATE.f_XYZ[2][1] + (STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[0]) * dt / 2;

	components_matrix[0][0] =   1;
	components_matrix[0][1] = - STATE.w_XYZ[2] * dt / 2;
	components_matrix[0][2] =   STATE.w_XYZ[1] * dt / 2;
	components_matrix[1][0] =   STATE.w_XYZ[2] * dt / 2;
	components_matrix[1][1] =   1;
	components_matrix[1][2] = - STATE.w_XYZ[0] * dt / 2;
	components_matrix[2][0] = - STATE.w_XYZ[1] * dt / 2;
	components_matrix[2][1] =   STATE.w_XYZ[0] * dt / 2;
	components_matrix[2][2] =   1;

	solveSystemByKramer(*components_matrix, free_vector, solution_vector);
	STATE.f_XYZ[0][1] = solution_vector[0];
	STATE.f_XYZ[1][1] = solution_vector[1];
	STATE.f_XYZ[2][1] = solution_vector[2];
	printf("f_XYZ[1] = %f\n", sqrt(pow(STATE.f_XYZ[0][1],2) + pow(STATE.f_XYZ[1][1],2) + pow(STATE.f_XYZ[2][1],2)));


	//находим третий столбец матрицы поворота из условия ортогональности векторов
	STATE.f_XYZ[2][2] =  STATE.f_XYZ[0][0] * STATE.f_XYZ[1][1] - STATE.f_XYZ[1][0] * STATE.f_XYZ[0][1];
	STATE.f_XYZ[0][2] = (STATE.f_XYZ[0][0] * STATE.f_XYZ[2][0] + STATE.f_XYZ[0][1] * STATE.f_XYZ[2][1]) / STATE.f_XYZ[2][2];
	STATE.f_XYZ[1][2] = (STATE.f_XYZ[1][0] * STATE.f_XYZ[2][0] + STATE.f_XYZ[1][1] * STATE.f_XYZ[2][1]) / STATE.f_XYZ[2][2];
	printf("f_XYZ[2] = %f\n", sqrt(pow(STATE.f_XYZ[0][2],2) + pow(STATE.f_XYZ[1][2],2) + pow(STATE.f_XYZ[2][2],2)));
/*
	//TODO: ДОБАВИТЬ В ОСНОВНОЙ КОД!!!!!!!
	set_cos_to_1(&ROT_M.f_XYZ[0][0]);
	set_cos_to_1(&ROT_M.f_XYZ[0][1]);
	set_cos_to_1(&ROT_M.f_XYZ[0][2]);
	set_cos_to_1(&ROT_M.f_XYZ[1][0]);
	set_cos_to_1(&ROT_M.f_XYZ[1][1]);
	set_cos_to_1(&ROT_M.f_XYZ[1][2]);
	set_cos_to_1(&ROT_M.f_XYZ[2][0]);
	set_cos_to_1(&ROT_M.f_XYZ[2][1]);
	set_cos_to_1(&ROT_M.f_XYZ[2][2]);



	//обновление функций углов в STATE
	STATE.f_XYZ[0][0] = ROT_M.f_XYZ[0][0];
	STATE.f_XYZ[0][1] = ROT_M.f_XYZ[0][1];
	STATE.f_XYZ[0][2] = ROT_M.f_XYZ[0][2];
	STATE.f_XYZ[1][0] = ROT_M.f_XYZ[1][0];
	STATE.f_XYZ[1][1] = ROT_M.f_XYZ[1][1];
	STATE.f_XYZ[1][2] = ROT_M.f_XYZ[1][2];
	STATE.f_XYZ[2][0] = ROT_M.f_XYZ[2][0];
	STATE.f_XYZ[2][1] = ROT_M.f_XYZ[2][1];
	STATE.f_XYZ[2][2] = ROT_M.f_XYZ[2][2];
*/

	//определение ускорений
	RSC_to_ISC_recalc(STATE.aRelatedXYZ, STATE.a_XYZ);

	//определение скоростей
	STATE.v_XYZ[0] = STATE.v_XYZ[0] + (STATE.a_XYZ_prev[0] + STATE.a_XYZ[0]) * dt / 2;
	STATE.v_XYZ[1] = STATE.v_XYZ[1] + (STATE.a_XYZ_prev[1] + STATE.a_XYZ[1]) * dt / 2;
	STATE.v_XYZ[2] = STATE.v_XYZ[2] + (STATE.a_XYZ_prev[2] + STATE.a_XYZ[2]) * dt / 2;


	//расчет перемещений
	STATE.s_XYZ[0] = STATE.s_XYZ[0] + (STATE.v_XYZ_prev[0] + STATE.v_XYZ[0]) * dt / 2;
	STATE.s_XYZ[1] = STATE.s_XYZ[1] + (STATE.v_XYZ_prev[1] + STATE.v_XYZ[1]) * dt / 2;
	STATE.s_XYZ[2] = STATE.s_XYZ[2] + (STATE.v_XYZ_prev[2] + STATE.v_XYZ[2]) * dt / 2;
	//calculate_height();	//записывает в STATE.sZ значение высоты

	//TODO: STATE.pressure = BMP_recalc

	for (int i = 0; i < 3; i++)
	{
		STATE.a_XYZ_prev[i] = STATE.a_XYZ[i];
		STATE.v_XYZ_prev[i] = STATE.v_XYZ[i];
		STATE.w_XYZ_prev[i] = STATE.w_XYZ[i];
		for (int j = 0; j < 3; j++)
		{
			STATE.f_XYZ_prev[i][j] = STATE.f_XYZ[i][j];
		}
	}
	STATE.previous_time = time;
}

/*
void getTranslations (float * translations)
{
	float * first_translations = (float*)translations;

	*(first_translations + 0) = STATE.s_XYZ[0];
	*(first_translations + 1) = STATE.s_XYZ[1];
	*(first_translations + 2) = STATE.s_XYZ[2];
}


void getAngVelocity (float * angVelocity)
{
	float * first_angVelocity = (float*)angVelocity;

	*(first_angVelocity + 0) = STATE.w_XYZ[0];
	*(first_angVelocity + 1) = STATE.w_XYZ[1];
	*(first_angVelocity + 2) = STATE.w_XYZ[2];

}


void getRotationMatrix (float * RotationMatrix)
{
	float * first_RotationMatrix = (float*)RotationMatrix;

	*(first_RotationMatrix + 0) = STATE.f_XYZ[0][0];
	*(first_RotationMatrix + 1) = STATE.f_XYZ[0][1];
	*(first_RotationMatrix + 2) = STATE.f_XYZ[0][2];
	*(first_RotationMatrix + 3) = STATE.f_XYZ[1][0];
	*(first_RotationMatrix + 4) = STATE.f_XYZ[1][1];
	*(first_RotationMatrix + 5) = STATE.f_XYZ[1][2];
	*(first_RotationMatrix + 6) = STATE.f_XYZ[2][0];
	*(first_RotationMatrix + 7) = STATE.f_XYZ[2][1];
	*(first_RotationMatrix + 8) = STATE.f_XYZ[2][2];

}*/


void solveSystemByKramer (float * Matrix, float * vector, float * solution_vect)
{
	float det, det1, det2, det3;

	det = getDeterminant(Matrix);

	replaceColumn(Matrix, vector, 0);
	det1 = getDeterminant(Matrix);
	replaceColumn(Matrix, vector, 0);

	replaceColumn(Matrix, vector, 1);
	det2 = getDeterminant(Matrix);
	replaceColumn(Matrix, vector, 1);

	replaceColumn(Matrix, vector, 2);
	det3 = getDeterminant(Matrix);
	replaceColumn(Matrix, vector, 2);

	*(solution_vect + 0) = det1 / det;
	*(solution_vect + 1) = det2 / det;
	*(solution_vect + 2) = det3 / det;
}


void replaceColumn (float * Matrix, float * vector, int column_n)
{
	float temp;
	for (int i = 0; i < 3; i++)
	{
		temp = *(Matrix + column_n + 3 * i);
		*(Matrix + column_n + 3 * i) = *(vector + i);
		*(vector + i) = temp;
	}
}


float getDeterminant (float * Matrix)
{
	float det = 0;
	int i, j, k;

	for (i = 0; i < 3; i++)
	{
		j = i + 1;
		if (j >= 3) j = j - 3;
		k = i + 2;
		if (k >= 3) k = k - 3;
		det += *(Matrix + i) * (*(Matrix + 3 + j) * *(Matrix + 6 + k) - *(Matrix + 3 + k) * *(Matrix + 6 + j));
	}
	return det;
}
