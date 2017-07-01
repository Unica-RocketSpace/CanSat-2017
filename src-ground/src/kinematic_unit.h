/*
 * kinematic_unit.h
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

/*
 * МОДУЛЬ, ОТВЕЧАЮЩИЙ ЗА ОПРЕДЕЛЕНИЕ
 * ПОЛОЖЕНИЯ И ОРИНЕТАЦИИ АППАРАТА
 * В ПРОСТРАНСТВЕ
 * */
#ifndef KINEMATIC_UNIT_H_
#define KINEMATIC_UNIT_H_

#include <stdint.h>
#include <stdio.h>


#define G_VECT 9.81

#define ACCEL_KALMAN_GAIN	0.5
#define GYRO_KALMAN_GAIN	0.1

#define ACCEL_NOISE	0.4
#define GYRO_NOISE	0.01


/*=================================================================================*/
/*===============================ОПИСАНИЕ=СТРУКТУР=================================*/
/*=================================================================================*/
//СТРУКТУРА КИНЕМАТИЧЕСКОГО СОСТОЯНИЯ АППАРАТА
typedef struct
{

	float aRelatedXYZ[3];	//ускорения в единицах g (в ССК)
	float aALT_XYZ[3];		//ускорения в единицах g (в ССК) альтернативное	FIXME:ВРЕМЕННО
	float gRelatedXYZ[3];	//угловые скорости в degps (в ССК)
	float gRelatedXYZ_prev[3];
	float aRelatedXYZ_prev[3];
	float cRelatedXYZ[3];	//косинусы углов вектора магнитного поля с осями ССК
	float height;
	float zero_pressure;
	float pressure;
	float temp_bmp280;			//FIXME: ВРЕМЕННО! после проверки убрать
	float temp_ds18b20;			//FIXME: ВРЕМЕННО! после проверки убрать


	float a_XYZ[3];			//ускорения в м/с^2 (ИСК)
	float a_XYZ_prev[3];	//ускорения в м/с^2 (ИСК) предыдущие
	float v_XYZ[3];			//скорости  в м/с   (ИСК)
	float v_XYZ_prev[3];	//скорости  в м/с   (ИСК) предыдущие
	float s_XYZ[3];			//перемещения в м   (ИСК)
	float w_XYZ[3];			//угловые скорости в 1/с (ИСК)
	float w_XYZ_prev[3];	//угловые скорости в 1/с (ИСК) предыдущие

	//Матрица поворота ССК относительно ИСК
	float f_XYZ[3][3];		//(строка, столбец)
	float f_XYZ_prev[3][3];	//(строка, столбец) предыдущая

	//Единичный вектор магнитного поля
	float B_XYZ[3];

	uint8_t state;		//состояние, можно писать интересующие биты

	uint32_t Time;
	uint32_t previousTime;

}state;

/*=================================================================================*/
/*=================================================================================*/

//КИНЕМАТИЧЕСКОЕ СОСТОЯНИЕ АППАРАТА
extern state DEVICE_STATE;

/*=================================================================================*/
/*===============================ОПИСАНИЕ=ФУНКЦИЙ==================================*/
/*=================================================================================*/
//ИНИЦИАЛИЗИРУЕТ ПРОГРАММУ КИНЕМАТИЧЕСКОГО СОСТОЯНИЯ АППАРАТА
void kinematicInit();

//УСТАНАВЛИВАЕТ НУЛЕВОЕ ДАВЛЕНИЕ
void set_zero_pressure();

//ЧИТАЕТ BMP280, ПЕРЕВОДИТ В uint16_t И ПЕРЕСЧИТЫВАЕТ ДАВЛЕНИЕz В float
void pressure_read_recon(int32_t * pressure32, int32_t * temp32, float * height, float * temp);

//ПЕРЕВОДИТ ПОЛУЧЕННЫЙ ВЕКТОР ИЗ СВЯЗАННОЙ СИСТЕМЫ КООРДИНАТ В ИНЕРЦИАЛЬНУЮ, ИСПОЛЬЗУЯ МАТРИЦУ ПОВОРОТА STATE.fXYZ[3][3]
void RSC_to_ISC_recalc(float * RSC_vect, float * ISC_vect);		//R - related, I - inertional

//УСТАНОВКА ИСК (запись поправочных направляющих косинусов)
void set_ISC_offset();

//ЗАПИСЬ МАТРИЦЫ НАПРАВЛЯЮЩИХ КОСИНУСОВ ВЕКТОРА МАГНИТНОГО ПОЛЯ С ОСЯМИ ИСК
void set_magn_dir();

//ОСУЩЕСТВЛЯЕТ КОРРЕКТИРОВКУ МАТРИЦЫ НАПРАВЛЯЮЩИХ КОСИНУСОВ ПО ПОКАЗАНИЯМ МАГНИТОМЕТРА
void recalc_ISC();

//ОПРАШИВАЕТ ДАТЧИКИ
void pull_recon_data();

//ОПРЕДЕЛЯЕТ ВЫСОТУ ПО ДАВЛЕНИЮ
void calculate_height();

//РАССЧИТЫВАЕТ МАТРИЦУ ПОВОРОТА, УГЛОВЫЕ СКОРОСТИ, СКОРОСТИ, УСКОРЕНИЯ И ПЕРЕМЕЩЕНИЯ В ИСК И ЗАПИСЫВАЕТ ИХ В STATE
void construct_trajectory();

void apply_KalmanFilter(float * sensor_data, const float * sensor_data_prev, float Kalman_gain, int data_array_size);

void apply_NoiseFilter(float * sensor_data, float noise, int data_array_size);

float functionForRK(uint8_t i, float * y);

void solveByRungeKutta(float dt, float * y, float * y_new);



#endif /* KINEMATIC_UNIT_H_ */
