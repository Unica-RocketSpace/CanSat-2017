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

#include <rscs/ds18b20.h>
#include <rscs/bmp280.h>
#include <rscs/adxl345.h>

extern rscs_bmp280_descriptor_t * bmp280;
extern rscs_ds18b20_t * ds18b20;
extern rscs_adxl345_t * adxl345;

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


//Необработанные данные для передачи по радиоканалу
typedef struct
{
	int16_t aTransmitXYZ[3];			//ускорения c MPU9255
	int16_t ADXL_transmit[3];			//ускорения c ADXL345
	int16_t gTransmitXYZ[3];
	int16_t cTransmitXYZ[3];


	int16_t temp_ds18b20;
	int32_t temp_bmp280;
	int32_t pressure;

}transmit_data;


typedef struct
{
	uint32_t zero;
	uint32_t imu;
	uint32_t filters;
	uint32_t bmp280;
	uint32_t ds18b20;
	uint32_t adxl345;
	uint32_t transmition;
	uint32_t total;
} times;

/*=================================================================================*/
/*=================================================================================*/

//КИНЕМАТИЧЕСКОЕ СОСТОЯНИЕ АППАРАТА
extern state STATE;

extern transmit_data TRANSMIT_DATA;

extern times TIMES;

extern const rscs_bmp280_calibration_values_t * calibrate_values;

/*=================================================================================*/
/*===============================ОПИСАНИЕ=ФУНКЦИЙ==================================*/
/*=================================================================================*/
//ИНИЦИАЛИЗИРУЕТ ПРОГРАММУ КИНЕМАТИЧЕСКОГО СОСТОЯНИЯ АППАРАТА
void kinematicInit();

//ИНИЦИАЛИЗИРУЕТ ШИНЫ И ДАТЧИКИ
void hardwareInit();

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

//РЕШАЕТ СИСТЕМУ ЛИНЕЙНЫХ УРАВНЕНИЙ МЕТОДОМ КРАМЕРА
//ПАРАМЕТР:		* matrix - ссылка на массив[3][3] множителей переменных
//ПАРАМЕТР:		* vector - ссылка на вектор[3] свободных членов
//ПАРАМЕТР:		* solution_vect - ссылка на вектор[3], в который будет записано решение
void solveSystemByKramer (float * Matrix, float * vector, float * solution_vect);

//ПРОИЗВОДИТ ЗАМЕНУ ЗАДАННОГО СТОЛБЦА МАТРИЦЫ НА ЗАДАННЫЙ ВЕКТОР (ДЛЯ МЕТОДА КРАМЕРА)
//при этом замененный вектор матрицы перезаписывается в исходный "vector"
//ПАРАМЕТР:		* matrix - ссылка на массив[3][3]
//ПАРАМЕТР:		* vector - ссылка на вектор[3] для замены
//ПАРАМЕТР:		* column_n - номер заменяемого столбца
void replaceColumn (float * matrix, float * vector, int column_n);

//НАХОДИТ ОПРЕДЕЛИТЕЛЬ МАТРИЦЫ 3х3
//ПАРАМЕТР:		* matrix - ссылка на массив[3][3]
float getDeterminant (float * matrix);

float functionForRK(uint8_t i, float * y);

void solveByRungeKutta(float dt, float * y, float * y_new);

//ПЕРЕДАЕТ ПЕРЕМЕЩЕНИЯ АППАРАТА В ИСК
//ПАРАМЕТР:		* translations - ссылка на массив[3], в который будут записаны перемещения аппарата по осям X, Y и Z соответственно (м)
void getTranslations (float * translations);

//ПЕРЕДАЕТ УГЛОВЫЕ СКОРОСТИ АППАРАТА В ИСК
//ПАРАМЕТР:		* angVelocityX - ссылка на массив[3], в который будут записаны угловые скорости аппарата вокруг осей X, Y и Z соответственно (рад/с)
void getAngVelocity (float * angVelocity);

//ПЕРЕДАЕТ МАТРИЦУ ПОВОРОТА АППАРАТА
//ПАРАМЕТР:		* RotationMatrix - ссылка на массив[9], в который будут записаны косинусы углов поворота ССК относительно ИСК (-)
void getRotationMatrix (float * RotationMatrix);



#endif /* KINEMATIC_UNIT_H_ */
