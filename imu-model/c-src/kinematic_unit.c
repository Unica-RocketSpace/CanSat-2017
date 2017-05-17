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

	/*state STATE_ = {	{a[0], a[1], a[2]},	//ускорения в единицах g (в ССК)
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
					};*/
	state STATE_;
		STATE_.aRelatedXYZ[0] = a[0];	STATE_.aRelatedXYZ[1] = a[1];	STATE_.aRelatedXYZ[2] = a[2];
		STATE_.aALT_XYZ[0] = 0;		STATE_.aALT_XYZ[1] = 0;		STATE_.aALT_XYZ[2] = 0;
		STATE_.gRelatedXYZ[0] = 0;	STATE_.gRelatedXYZ[1] = 0;	STATE_.gRelatedXYZ[2] = w;
		STATE_.cRelatedXYZ[0] = 0;	STATE_.cRelatedXYZ[1] = 0;	STATE_.cRelatedXYZ[2] = 0;

		STATE_.height = 0;
		STATE_.zero_pressure = 0;
		STATE_.pressure = 0;

		STATE_.temp_bmp280 = 0;		STATE_.temp_ds18b20 = 0;

		STATE_.a_XYZ[0] = 0;		STATE_.a_XYZ[1] = 0;		STATE_.a_XYZ[2] = 0;
		STATE_.a_XYZ_prev[0] = 0;	STATE_.a_XYZ_prev[1] = 0;	STATE_.a_XYZ_prev[2] = 0;
		STATE_.v_XYZ[0] = v[0];		STATE_.v_XYZ[1] = v[1];		STATE_.v_XYZ[2] = v[2];
		STATE_.v_XYZ_prev[0] = v[0];STATE_.v_XYZ_prev[1] = v[1];STATE_.v_XYZ_prev[2] = v[2];
		STATE.s_XYZ[0] = s[0];		STATE.s_XYZ[1] = s[0];		STATE.s_XYZ[2] = s[0];
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
		STATE_.previous_time = 0;

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
	RSC_to_ISC_recalc(STATE.gRelatedXYZ, STATE.w_XYZ);

	//определение углов между осями ИСК и ССК
	rotation_matrix ROT_M = {0, 0, 0};


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

	set_cos_to_1(&STATE.f_XYZ[0][0]);
	set_cos_to_1(&STATE.f_XYZ[0][1]);
	set_cos_to_1(&STATE.f_XYZ[0][2]);
	set_cos_to_1(&STATE.f_XYZ[1][0]);
	set_cos_to_1(&STATE.f_XYZ[1][1]);
	set_cos_to_1(&STATE.f_XYZ[1][2]);
	set_cos_to_1(&STATE.f_XYZ[2][0]);
	set_cos_to_1(&STATE.f_XYZ[2][1]);
	set_cos_to_1(&STATE.f_XYZ[2][2]);

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
