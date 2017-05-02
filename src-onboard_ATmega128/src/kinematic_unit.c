/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include <math.h>
#include <sofa.h>
#include <rscs/timeservice.h>
#include <rscs/ds18b20.h>				//ДРАЙВЕР ДАТЧИКА ТЕМПЕРАТУРЫ DS18B20
#include <rscs/bmp280.h>				//ДРАЙВЕР ДАТЧИКА ДАВЛЕНИЯ BMP180
#include <rscs/onewire.h>
#include <timer.h>
#include "rscs/stdext/stdio.h"
#include "util/delay.h"

//#include "ADXL345.h"					//ДРАЙВЕР АКСЕЛЕРОМЕТРА	ADXL345
#include "MPU9255.h"					//ДРАЙВЕР ГИРОСКОПА, АКСЕЛЕРОМЕТРА и КОМПАСА (MPU9255)
#include "kinematic_unit.h"
#include "radio_transmitter.h"

state STATE;
transmit_data TRANSMIT_DATA;
rscs_bmp280_descriptor_t * bmp280;
rscs_ds18b20_t * ds18b20;

const rscs_bmp280_calibration_values_t * calibrate_values;


void hardwareInit(void)
{
	/*инициализация i2c*/
	rscs_i2c_init();
	rscs_i2c_set_scl_rate(100);

	/*инициализация 1-wire*/
	rscs_ow_init_bus();

	/*инициализация ds18b20*/
	ds18b20 = rscs_ds18b20_init(0x00);		//создание дескриптора
	//FIXME: вернуть rscs_ds18b20_start_conversion(ds18b20);	//начало первого замера

	/*инициализация BMP280*/
	rscs_bmp280_parameters_t * parameters;
	parameters->pressure_oversampling = RSCS_BMP280_OVERSAMPLING_X4;		//4		измерения на один результат
	parameters->temperature_oversampling = RSCS_BMP280_OVERSAMPLING_X1;		//1		измерение на один результат
	parameters->standbytyme = RSCS_BMP280_STANDBYTIME_500US;				//0.5ms	время между 2 измерениями
	parameters->filter = RSCS_BMP280_FILTER_X16;							//x16	фильтр

	bmp280 = rscs_bmp280_initi2c(RSCS_BMP280_I2C_ADDR_LOW/*!!!*/);			//создание дескриптора
	rscs_bmp280_setup(bmp280, parameters);									//запись параметров
	rscs_bmp280_changemode(bmp280, RSCS_BMP280_MODE_NORMAL);				//установка режима NORMAL, постоянные измерения

	/*инициализация MPU9255*/
	MPU9255_init();


	/*инициализация таймеров*/
	rscs_time_init();	//таймер времени
	timer1PWMInit();	//таймер для ШИМ


	/*инициализация радиопередатчика*/
	transmition_init();
}


void kinematicInit()
{
	state STATE_ = {
					{0, 0, 0},			//ускорения в единицах g (в ССК)
					{0, 0, 0},			//угловые скорости в degps (в ССК)
					{0, 0, 0},			//косинусы углов вектора магнитного поля с осями ССК
					 0,					//высота по давлению
					 100376.0,					//нулевое давление
					 0,					//температура 		FIXME:ВРЕМЕННО
					 0,					//температура 		FIXME:ВРЕМЕННО

					{0, 0, 0},			//ускорения в м/с^2 (ИСК)
					{0, 0, 0},			//скорости в м/с (ИСК)
					{0, 0, 0},			//перемещения в м (ИСК)
					{0, 0, 0},			//угловые скорости в 1/с (ИСК)

				{	{1, 0, 0},			//косинусы углов оси Х ИСК с осями ССК
					{0, 1, 0},			//косинусы углов оси Y ИСК с осями ССК
					{0, 0, 1}	},		//косинусы углов оси Z ИСК с осями ССК

					{0, 0, 0},			//единичный вектор магнитного поля
					 0,					//состояние
					 0					//время
	};

	STATE = STATE_;

	transmit_data TRANSMIT_DATA_ = {
					{0, 0, 0},			//ускорения
					{0, 0, 0},			//угловые сколости
					{0, 0, 0},			//вектор магноитного поля
					 0,					//температура с термометра
					 0,					//температура с барометра
					 0					//давление
	};

	TRANSMIT_DATA = TRANSMIT_DATA_;

}


void RSC_to_ISC_recalc(float * RSC_vect, float * ISC_vect)
{
	ISC_vect[0] = RSC_vect[0] * STATE.f_XYZ[0][0] + RSC_vect[1] * STATE.f_XYZ[0][1] + RSC_vect[2] * STATE.f_XYZ[0][2];
	ISC_vect[1] = RSC_vect[0] * STATE.f_XYZ[1][0] + RSC_vect[1] * STATE.f_XYZ[1][1] + RSC_vect[2] * STATE.f_XYZ[1][2];
	ISC_vect[2] = RSC_vect[0] * STATE.f_XYZ[2][0] + RSC_vect[1] * STATE.f_XYZ[2][1] + RSC_vect[2] * STATE.f_XYZ[2][2];
}



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
	float compass_XYZ[3];
	float dummy1;
	float B_vect[3], B_unit_vect[3], C_vect[3], C_unit_vect[3], A_unit_vect[3];		//C - ось вращения, A - третий вектор системы BAC

	for (int i = 0; i < 3; i++)		//переписываем в compass_XYZ показания магнитометра из STATE
	{
		compass_XYZ[i] = STATE.cRelatedXYZ[i];
	}

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

void set_zero_pressure()
{
	int32_t raw_pressure32, raw_temp32, pressure32, temp32;
	rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);

	calibrate_values = rscs_bmp280_get_calibration_values(bmp280);
	rscs_bmp280_calculate(calibrate_values, raw_pressure32, raw_temp32, &pressure32, &temp32);

	STATE.zero_pressure = (float)pressure32;
}

void pressure_read_recon(int16_t * raw_pressure, int16_t * raw_temp, float * height, float * temp)
{
	int32_t raw_pressure32, raw_temp32, pressure32, temp32;
	/*rscs_e error = */rscs_bmp280_read(bmp280, &raw_pressure32, &raw_temp32);

	*raw_pressure = (int16_t)(raw_pressure32 >> 4);
	*raw_temp = (int16_t)(raw_temp32 >> 4);

	calibrate_values = rscs_bmp280_get_calibration_values(bmp280);

	rscs_bmp280_calculate(calibrate_values, raw_pressure32, raw_temp32, &pressure32, &temp32);

	*height = 18.4 * log(STATE.zero_pressure / (float)pressure32);
	*temp = (float)temp32 / 100;

	//printf("error: %d ", error);
}


void pull_recon_data()
{
	//опрос MPU9255 и пересчет показаний
	MPU9255_read_imu(TRANSMIT_DATA.aTransmitXYZ, TRANSMIT_DATA.gTransmitXYZ);
	MPU9255_read_compass(TRANSMIT_DATA.cTransmitXYZ);

	MPU9255_recalc_accel(TRANSMIT_DATA.aTransmitXYZ, STATE.aRelatedXYZ);
	MPU9255_recalc_gyro(TRANSMIT_DATA.gTransmitXYZ, STATE.gRelatedXYZ);
	MPU9255_recalc_compass(TRANSMIT_DATA.cTransmitXYZ, STATE.cRelatedXYZ);
	/*=====================================================================*/

	//опрос барометра bmp280
	pressure_read_recon(&TRANSMIT_DATA.pressure, &TRANSMIT_DATA.temp1, &STATE.height, &STATE.temp_bmp280);

	//опрос термометра ds18b20
	if (rscs_ds18b20_check_ready())		//проверяем, готовы ли данные
	{
		rscs_e error = rscs_ds18b20_read_temperature(ds18b20, &TRANSMIT_DATA.temp2);	//записываем температуру
		printf("ds18b20_read_error: %d\n", error);
		STATE.temp_ds18b20 = rscs_ds18b20_count_temperature(ds18b20, &TRANSMIT_DATA.temp2);
		rscs_ds18b20_start_conversion(ds18b20);
	}

	STATE.Time = rscs_time_get();
}


void set_cos_to_1(float * cosalpha)
{
	if (*cosalpha > 1.0)
	{
		*cosalpha = 1.0;
	}
}


void construct_trajectory()
{

	float dt = rscs_time_get() / 1000;

	//определение угловых скоростей (в ИСК)
	RSC_to_ISC_recalc(STATE.gRelatedXYZ, STATE.w_XYZ);

	//определение углов между осями ИСК и ССК
	rotation_matrix ROT_M;		//FIX: Убрал обнуление структуры, так как не имеет особого смысла

	ROT_M.f_XYZ[0][0] = STATE.f_XYZ[0][0] + (STATE.f_XYZ[0][1] * STATE.w_XYZ[2] - STATE.f_XYZ[0][2] * STATE.w_XYZ[1]) * dt;
	ROT_M.f_XYZ[0][1] = STATE.f_XYZ[0][1] + (STATE.f_XYZ[0][2] * STATE.w_XYZ[0] - STATE.f_XYZ[0][0] * STATE.w_XYZ[2]) * dt;
	ROT_M.f_XYZ[0][2] = STATE.f_XYZ[0][2] + (STATE.f_XYZ[0][0] * STATE.w_XYZ[1] - STATE.f_XYZ[0][1] * STATE.w_XYZ[0]) * dt;
	ROT_M.f_XYZ[1][0] = STATE.f_XYZ[1][0] + (STATE.f_XYZ[1][1] * STATE.w_XYZ[2] - STATE.f_XYZ[1][2] * STATE.w_XYZ[1]) * dt;
	ROT_M.f_XYZ[1][1] = STATE.f_XYZ[1][1] + (STATE.f_XYZ[1][2] * STATE.w_XYZ[0] - STATE.f_XYZ[1][0] * STATE.w_XYZ[2]) * dt;
	ROT_M.f_XYZ[1][2] = STATE.f_XYZ[1][2] + (STATE.f_XYZ[1][0] * STATE.w_XYZ[1] - STATE.f_XYZ[1][1] * STATE.w_XYZ[0]) * dt;

	ROT_M.f_XYZ[2][2] =  ROT_M.f_XYZ[0][0] * ROT_M.f_XYZ[1][1] - ROT_M.f_XYZ[0][1] * ROT_M.f_XYZ[1][0];
	ROT_M.f_XYZ[2][0] = (ROT_M.f_XYZ[0][0] * ROT_M.f_XYZ[0][2] + ROT_M.f_XYZ[1][0] * ROT_M.f_XYZ[1][2]) / ROT_M.f_XYZ[2][2];
	ROT_M.f_XYZ[2][1] = (ROT_M.f_XYZ[0][1] * ROT_M.f_XYZ[0][2] + ROT_M.f_XYZ[1][1] * ROT_M.f_XYZ[1][2]) / ROT_M.f_XYZ[2][2];


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


	//определение ускорений
	RSC_to_ISC_recalc(STATE.aRelatedXYZ, STATE.a_XYZ);

	//определение скоростей
	STATE.v_XYZ[0] = STATE.v_XYZ[0] + STATE.a_XYZ[0] * dt;
	STATE.v_XYZ[1] = STATE.v_XYZ[1] + STATE.a_XYZ[1] * dt;
	STATE.v_XYZ[2] = STATE.v_XYZ[2] + STATE.a_XYZ[2] * dt;


	//расчет перемещений
	STATE.s_XYZ[0] = STATE.s_XYZ[0] + STATE.v_XYZ[0] * dt + STATE.a_XYZ[0] * dt * dt / 2;
	STATE.s_XYZ[1] = STATE.s_XYZ[1] + STATE.v_XYZ[1] * dt + STATE.a_XYZ[1] * dt * dt / 2;
	//STATE.s_XYZ[2] = STATE.s_XYZ[2] + STATE.v_XYZ[2] * dt + STATE.a_XYZ[2] * dt * dt / 2;
	STATE.s_XYZ[2] = STATE.height;	//записывает в STATE.sZ значение высоты

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



