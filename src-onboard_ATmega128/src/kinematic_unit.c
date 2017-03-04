/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include "../librscs/rscs/ds18b20.h"	//ДРАЙВЕР ДАТЧИКА ТЕМПЕРАТУРЫ DS18B20
//#include "../librscs/rscs/bmp180.h"	//ДРАЙВЕР ДАТЧИКА ДАВЛЕНИЯ BMP180

#include "ADXL345.h"					//ДРАЙВЕР АКСЕЛЕРОМЕТРА	ADXL345
//#include "MPU9255.h"					//ДРАЙВЕР ГИРОСКОПА, АКСЕЛЕРОМЕТРА и КОМПАСА (MPU9255)
#include "kinematic_unit.h"


void kinematicInit()
{
	state STATE = {	0, 0, 0,	//ускорения в единицах g (в ССК)
					0, 0, 0,	//угловые скорости в degps (в ССК)
					0, 0, 0,	//косинусы углов вектора магнитного поля с осями ССК

					0, 0, 0,	//ускорения в м/с^2 (ИСК)
					0, 0, 0,	//скорости в м/с (ИСК)
					0, 0, 0,	//перемещения в м (ИСК)
					0, 0, 0,	//угловые скорости в 1/с (ИСК)

					0, 0, 0,	//косинусы углов оси Х ИСК с осями ССК
					0, 0, 0,	//косинусы углов оси Y ИСК с осями ССК
					0, 0, 0		//косинусы углов оси Z ИСК с осями ССК
				  };



}


void reconState()
{

}


void getTranslations (float * translations)
{

}


void getAngVelocity (float * angVelocity)
{

}


void getRotationMatrix (float * RotationMatrix)
{

}

