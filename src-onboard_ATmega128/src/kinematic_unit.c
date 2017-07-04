/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include <math.h>
#include <sofa.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <rscs/timeservice.h>
#include <rscs/ds18b20.h>				//ДРАЙВЕР ДАТЧИКА ТЕМПЕРАТУРЫ DS18B20
#include <rscs/bmp280.h>				//ДРАЙВЕР ДАТЧИКА ДАВЛЕНИЯ BMP180
#include <rscs/onewire.h>
#include <rscs/stdext/stdio.h>
#include <rscs/adxl345.h>				//ДРАЙВЕР АКСЕЛЕРОМЕТРА	ADXL345



#include "MPU9255.h"					//ДРАЙВЕР ГИРОСКОПА, АКСЕЛЕРОМЕТРА и КОМПАСА (MPU9255)
#include "kinematic_unit.h"
#include "radio_transmitter.h"
#include "timer.h"

state STATE;
transmit_data TRANSMIT_DATA;
times TIMES;
rscs_bmp280_descriptor_t * bmp280;
rscs_ds18b20_t * ds18b20;
rscs_adxl345_t * adxl345;


const rscs_bmp280_calibration_values_t * calibrate_values;


void hardwareInit(void)
{
	/*инициализация радиопередатчика*/
	transmition_init();

	/*инициализация i2c*/
	rscs_i2c_init();
	rscs_i2c_set_scl_rate(200);

	/*инициализация 1-wire*/
	rscs_ow_init_bus();

	/*инициализация ds18b20*/
	ds18b20 = rscs_ds18b20_init(0x00);		//создание дескриптора
	rscs_ds18b20_start_conversion(ds18b20);	//начало первого замера

	/*инициализация BMP280*/

	rscs_bmp280_parameters_t parameters;
	parameters.pressure_oversampling = RSCS_BMP280_OVERSAMPLING_X4;		//4		16		измерения на один результат
	parameters.temperature_oversampling = RSCS_BMP280_OVERSAMPLING_X2;	//1		2		измерение на один результат
	parameters.standbytyme = RSCS_BMP280_STANDBYTIME_500US;				//0.5ms	62.5ms	время между 2 измерениями
	parameters.filter = RSCS_BMP280_FILTER_X16;							//x16	x16		фильтр

	bmp280 = rscs_bmp280_initi2c(RSCS_BMP280_I2C_ADDR_LOW);				//создание дескриптора
	rscs_bmp280_setup(bmp280, &parameters);								//запись параметров
	rscs_bmp280_changemode(bmp280, RSCS_BMP280_MODE_NORMAL);			//установка режима NORMAL, постоянные измерения


	/*инициализация MPU9255*/
	MPU9255_init();

	/*инициализация ADXL345*/
	adxl345 = rscs_adxl345_initi2c(RSCS_ADXL345_ADDR_ALT);
	rscs_adxl345_startup(adxl345);
	rscs_adxl345_set_range(adxl345, RSCS_ADXL345_RANGE_4G);
	rscs_adxl345_set_rate(adxl345, RSCS_ADXL345_RATE_100HZ);

	/*инициализация таймеров*/
	sei();
	rscs_time_init();	//таймер времени
	timer1PWMInit();	//таймер для ШИМ

}


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

	STATE = STATE_;

	transmit_data TRANSMIT_DATA_ = {
					{0, 0, 0},			//ускорения c MPU9255
					{0, 0, 0},			//ускорения с ADXL345
					{0, 0, 0},			//угловые скорости
					{0, 0, 0},			//вектор магнитного поля
					 0,					//температура с барометра
					 0,					//температура с термометра
					 0					//давление
	};

	TRANSMIT_DATA = TRANSMIT_DATA_;

}


void RSC_to_ISC_recalc(float * RSC_vect, float * ISC_vect)
{
	iauRxp(STATE.f_XYZ, RSC_vect, ISC_vect);
	//ISC_vect[0] = RSC_vect[0] * STATE.f_XYZ[0][0] + RSC_vect[1] * STATE.f_XYZ[0][1] + RSC_vect[2] * STATE.f_XYZ[0][2];
	//ISC_vect[1] = RSC_vect[0] * STATE.f_XYZ[1][0] + RSC_vect[1] * STATE.f_XYZ[1][1] + RSC_vect[2] * STATE.f_XYZ[1][2];
	//ISC_vect[2] = RSC_vect[0] * STATE.f_XYZ[2][0] + RSC_vect[1] * STATE.f_XYZ[2][1] + RSC_vect[2] * STATE.f_XYZ[2][2];
}

void set_ISC_offset()
{
	rscs_e error1 = 1;
	float dummy1;
	int16_t dummy2;
	int16_t accel_raw_XYZ[3];
	float accel_XYZ[3];

	float x1_unit_vect[3] = {1, 0, 0};
	float x_vect[3], x_unit_vect[3], y_vect[3], y_unit_vect[3], g_unit_vect[3];

	MPU9255_read_imu(accel_raw_XYZ, &dummy2);
	MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);

	while (error1 != 0)
	{
		iauPn(accel_XYZ, &dummy1, g_unit_vect);

		//float g_vect = sqrt(pow(*(first_accel_XYZ + 0), 2) + pow(*(first_accel_XYZ + 1), 2) + pow(*(first_accel_XYZ + 2), 2));

		STATE.f_XYZ[2][0] = g_unit_vect[0];
		STATE.f_XYZ[2][1] = g_unit_vect[1];
		STATE.f_XYZ[2][2] = g_unit_vect[2];

		error1 = MPU9255_read_imu(accel_raw_XYZ, &dummy2);
		printf("error1 = %d\n", error1);
		MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);
	}


	iauPxp(g_unit_vect, x1_unit_vect, y_vect);
	iauPn(y_vect, &dummy1, y_unit_vect);

	iauPxp(y_unit_vect, g_unit_vect, x_vect);
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
	rscs_e error1 = 1;
	int16_t compass_raw_XYZ[3];
	float compass_XYZ[3];
	float dummy1;
	float B_unit_vect[3];

	MPU9255_read_compass(compass_raw_XYZ);
	MPU9255_recalc_compass(compass_raw_XYZ, STATE.cRelatedXYZ);

	while (error1 != 0)
	{
		//iauPn(compass_XYZ, &dummy1, B_unit_vect);

		RSC_to_ISC_recalc(B_unit_vect, STATE.B_XYZ);
		iauPn(STATE.B_XYZ, &dummy1, STATE.B_XYZ);

		error1 = MPU9255_read_compass(compass_raw_XYZ);
		MPU9255_recalc_compass(compass_raw_XYZ, STATE.cRelatedXYZ);
	}
}

void recalc_ISC()
{
	float compass_XYZ[3];
	float dummy1;
	float B_vect[3], B_unit_vect[3], C_vect[3], C_unit_vect[3], A_unit_vect[3];		//C - ось вращения, A - третий вектор системы BAC

	for (int i = 0; i < 3; i++)		//переписываем в compass_XYZ показания магнитометра из STATE
	{
		compass_XYZ[i] = STATE.cRelatedXYZ[i];
	}

	RSC_to_ISC_recalc(compass_XYZ, B_vect);		//пересчет вектора магнитного поля в ИСК
	iauPn(B_vect, &dummy1, B_unit_vect);
	iauPxp(STATE.B_XYZ, B_unit_vect, C_vect);	//создаем ось вращения С, перпендикулярную плоскости (M,M1)
	iauPn(C_vect, &dummy1, C_unit_vect);		//нормируем вектор С

	iauPxp(C_unit_vect, STATE.B_XYZ, A_unit_vect);	//находим третий вектор (А) системы координат BАС
	iauPn(A_unit_vect, &dummy1, A_unit_vect);

	//создание матрицы перехода (М) (BАС->ИСК) и (М) транспонированной
	float M[3][3] = {	{B_unit_vect[0], A_unit_vect[0], C_unit_vect[0]},
						{B_unit_vect[1], A_unit_vect[1], C_unit_vect[1]},
						{B_unit_vect[2], A_unit_vect[2], C_unit_vect[2]}	};
	float M_1[3][3];

	iauTr(M, M_1);		//Транспонирование матрицы (М)
	float cosangle = iauPdp(B_unit_vect, STATE.B_XYZ);		//находим косинус угла между векторами В(старый) и B1(новый)
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

	//переписываем STATE.f_XYZ
	iauRxr(T, STATE.f_XYZ, STATE.f_XYZ);
}

void set_zero_pressure()
{
	int32_t raw_pressure32, raw_temp32, pressure32, temp32;
	rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);

	const rscs_bmp280_calibration_values_t * calibrate_values_ = rscs_bmp280_get_calibration_values(bmp280);
	rscs_bmp280_calculate(calibrate_values_, raw_pressure32, raw_temp32, &pressure32, &temp32);

	STATE.zero_pressure = (float)pressure32;
}

void pressure_read_recon(int32_t * pressure32, int32_t * temp32, float * height, float * temp)
{
	int32_t raw_pressure32, raw_temp32;
	rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);


	calibrate_values = rscs_bmp280_get_calibration_values(bmp280);
	rscs_bmp280_calculate(calibrate_values, raw_pressure32, raw_temp32, pressure32, temp32);

	STATE.pressure = (float)*pressure32;
	*height = 18.4 * log(STATE.zero_pressure / STATE.pressure);
	*temp = (float)*temp32 / 100;
}

void pull_recon_data()
{


	//TIMES.imu = 0;
	//TIMES.filters = 0;
	//TIMES.bmp280 = 0;
	//TIMES.ds18b20 = 0;
	//TIMES.adxl345 = 0;
	//TIMES.zero = rscs_time_get();
	//TIMES.total = TIMES.zero;


	//опрос MPU9255 и пересчет показаний
	MPU9255_read_imu(TRANSMIT_DATA.aTransmitXYZ, TRANSMIT_DATA.gTransmitXYZ);
	MPU9255_read_compass(TRANSMIT_DATA.cTransmitXYZ);

	MPU9255_recalc_accel(TRANSMIT_DATA.aTransmitXYZ, STATE.aRelatedXYZ);
	MPU9255_recalc_gyro(TRANSMIT_DATA.gTransmitXYZ, STATE.gRelatedXYZ);
	MPU9255_recalc_compass(TRANSMIT_DATA.cTransmitXYZ, STATE.cRelatedXYZ);
	//TIMES.imu = rscs_time_get() - TIMES.zero;
	//TIMES.total = TIMES.total + TIMES.imu;
	/*=====================================================================*/

	//Применение фильтра Калмана
	//apply_KalmanFilter(STATE.aRelatedXYZ, STATE.aRelatedXYZ_prev, ACCEL_KALMAN_GAIN, 3);
	//apply_KalmanFilter(STATE.gRelatedXYZ, STATE.gRelatedXYZ_prev, GYRO_KALMAN_GAIN, 3);
	/*=====================================================================*/

	//Фильтрация шума
	//printf("Gyroscope:\n");
	//apply_NoiseFilter(STATE.gRelatedXYZ, GYRO_NOISE, 3);
	//printf("Accelerometer:\n");
	//apply_NoiseFilter(STATE.aRelatedXYZ, ACCEL_NOISE, 3);
	//TIMES.filters = rscs_time_get() - TIMES.total;
	//TIMES.total = TIMES.total + TIMES.filters;
	/*=====================================================================*/

	//опрос барометра bmp280
	pressure_read_recon(&TRANSMIT_DATA.pressure, &TRANSMIT_DATA.temp_bmp280, &STATE.height, &STATE.temp_bmp280);
	//TIMES.bmp280 = rscs_time_get() - TIMES.total;
	//TIMES.total = TIMES.total + TIMES.bmp280;
	/*=====================================================================*/

	//опрос термометра ds18b20
	if (rscs_ds18b20_check_ready())		//проверяем, готовы ли данные
	{
		rscs_ds18b20_read_temperature(ds18b20, &TRANSMIT_DATA.temp_ds18b20);	//записываем температуру
		STATE.temp_ds18b20 = rscs_ds18b20_count_temperature(ds18b20, TRANSMIT_DATA.temp_ds18b20);
		rscs_ds18b20_start_conversion(ds18b20);
	}
	//TIMES.ds18b20 = rscs_time_get() - TIMES.total;
	//TIMES.total = TIMES.total + TIMES.ds18b20;
	/*=====================================================================*/

	//опрос акселерометра ADXL345
	rscs_adxl345_GetGXYZ(adxl345, &TRANSMIT_DATA.ADXL_transmit[0], &TRANSMIT_DATA.ADXL_transmit[1], &TRANSMIT_DATA.ADXL_transmit[2],
									&STATE.aALT_XYZ[0], &STATE.aALT_XYZ[1], &STATE.aALT_XYZ[2]);
	//TIMES.adxl345 = rscs_time_get() - TIMES.total;
	//TIMES.total = TIMES.total + TIMES.adxl345;
	/*=====================================================================*/

	STATE.Time = rscs_time_get();
}


void set_cos_to_1(float * cosalpha)
{
	if (*cosalpha > 1.0)	{*cosalpha = 1.0;}
	if (*cosalpha < -1.0)	{*cosalpha = -1.0;}

}

void construct_trajectory()
{

	float dt = (float)(STATE.Time - STATE.previousTime) / 1000;

	//определение угловых скоростей (в ИСК)
	RSC_to_ISC_recalc(STATE.gRelatedXYZ, STATE.w_XYZ);

	/*
	//определение углов между осями ИСК и ССК
	float free_vector[3], solution_vector[3];
	float components_matrix[3][3];
	//находим первую строку матрицы поворота
	free_vector[0] = STATE.f_XYZ[0][0] + (STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[1]) * dt / 2;
	free_vector[1] = STATE.f_XYZ[0][1] + (STATE.f_XYZ_prev[0][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[2]) * dt / 2;
	free_vector[2] = STATE.f_XYZ[0][2] + (STATE.f_XYZ_prev[0][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[0][1] * STATE.w_XYZ_prev[0]) * dt / 2;

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
	STATE.f_XYZ[0][0] = solution_vector[0] / iauPm(solution_vector);
	STATE.f_XYZ[0][1] = solution_vector[1] / iauPm(solution_vector);
	STATE.f_XYZ[0][2] = solution_vector[2] / iauPm(solution_vector);

	//printf("modulus (first row) = %f  ", iauPm(solution_vector));

	//находим вторую строку матрицы поворота
	free_vector[0] = STATE.f_XYZ[1][0] + (STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[2] - STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[1]) * dt / 2;
	free_vector[1] = STATE.f_XYZ[1][1] + (STATE.f_XYZ_prev[1][2] * STATE.w_XYZ_prev[0] - STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[2]) * dt / 2;
	free_vector[2] = STATE.f_XYZ[1][2] + (STATE.f_XYZ_prev[1][0] * STATE.w_XYZ_prev[1] - STATE.f_XYZ_prev[1][1] * STATE.w_XYZ_prev[0]) * dt / 2;

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
	STATE.f_XYZ[1][0] = solution_vector[0] / iauPm(solution_vector);
	STATE.f_XYZ[1][1] = solution_vector[1] / iauPm(solution_vector);
	STATE.f_XYZ[1][2] = solution_vector[2] / iauPm(solution_vector);
	*/

	//printf("f_XYZ_prev[0] = %f,   %f,   %f\n", STATE.f_XYZ_prev[0][0], STATE.f_XYZ_prev[0][1], STATE.f_XYZ_prev[0][2]);
	//printf("w_XYZ_prev    = %f,   %f,   %f\n", STATE.w_XYZ_prev[0]   , STATE.w_XYZ_prev[1]   , STATE.w_XYZ_prev[2]   );
	//printf("dt = %f\n", (float)(STATE.Time - STATE.previousTime) / 1000);
	solveByRungeKutta((float)(STATE.Time - STATE.previousTime) / 1000, STATE.f_XYZ_prev[0], STATE.f_XYZ[0]);
	solveByRungeKutta((float)(STATE.Time - STATE.previousTime) / 1000, STATE.f_XYZ_prev[1], STATE.f_XYZ[1]);

	//printf("SOLVED_BY_RUNGE-KUTTA\n");
	//printf("first row) = %f,   %f,   %f\n", STATE.f_XYZ[0][0], STATE.f_XYZ[0][1], STATE.f_XYZ[0][2]);
	//printf("first row) = %f,   %f,   %f\n", STATE.f_XYZ[1][0], STATE.f_XYZ[1][1], STATE.f_XYZ[1][2]);
	//printf("\n");
	//находим третью строку матрицы поворота из условия ортогональности векторов
	iauPxp(STATE.f_XYZ[0], STATE.f_XYZ[1], STATE.f_XYZ[2]);

	//printf("modulus (third row) = %f\n", sqrt(pow(STATE.f_XYZ[2][2], 2) + pow(STATE.f_XYZ[2][0], 2) + pow(STATE.f_XYZ[2][1], 2)));

	//printf("scalar dot xy = %f  ", iauPdp(STATE.f_XYZ[0], STATE.f_XYZ[1]));
	//printf("scalar dot xz = %f  ", iauPdp(STATE.f_XYZ[0], STATE.f_XYZ[2]));
	//printf("scalar dot yz = %f\n", iauPdp(STATE.f_XYZ[2], STATE.f_XYZ[1]));

	set_cos_to_1(&STATE.f_XYZ[0][0]);
	set_cos_to_1(&STATE.f_XYZ[0][1]);
	set_cos_to_1(&STATE.f_XYZ[0][2]);
	set_cos_to_1(&STATE.f_XYZ[1][0]);
	set_cos_to_1(&STATE.f_XYZ[1][1]);
	set_cos_to_1(&STATE.f_XYZ[1][2]);
	set_cos_to_1(&STATE.f_XYZ[2][0]);
	set_cos_to_1(&STATE.f_XYZ[2][1]);
	set_cos_to_1(&STATE.f_XYZ[2][2]);

	//printf("SOLVED_BY_KRAMER\n");
	//printf("%f,   %f,   %f\n", STATE.f_XYZ[0][0], STATE.f_XYZ[0][1], STATE.f_XYZ[0][2]);
	//printf("%f,   %f,   %f\n", STATE.f_XYZ[1][0], STATE.f_XYZ[1][1], STATE.f_XYZ[1][2]);
	//printf("%f,   %f,   %f\n\n", STATE.f_XYZ[2][0], STATE.f_XYZ[2][1], STATE.f_XYZ[2][2]);

	//определение ускорений
	RSC_to_ISC_recalc(STATE.aRelatedXYZ, STATE.a_XYZ);
	//STATE.a_XYZ[2] = STATE.a_XYZ[2] - G_VECT;

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
		STATE.aRelatedXYZ_prev[i] = STATE.aRelatedXYZ[i];
		STATE.gRelatedXYZ_prev[i] = STATE.gRelatedXYZ[i];
		for (int j = 0; j < 3; j++)
		{
			STATE.f_XYZ_prev[i][j] = STATE.f_XYZ[i][j];
		}
	}
	STATE.previousTime = STATE.Time;
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

inline float functionForRK(uint8_t i, float * y)
{
	switch (i) {
		case 0:
			//printf("y = %f, %f, %f\n", y[0], y[1], y[2]);
			return y[1] * STATE.w_XYZ_prev[2] - y[2] * STATE.w_XYZ_prev[1];
		case 1:
			return y[2] * STATE.w_XYZ_prev[0] - y[0] * STATE.w_XYZ_prev[2];
		case 2:
			return y[0] * STATE.w_XYZ_prev[1] - y[1] * STATE.w_XYZ_prev[0];
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
}



