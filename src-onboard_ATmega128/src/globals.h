/*
 * globals.h
 *
 *  Created on: 11 февр. 2017 г.
 *      Author: developer
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_


#define	coordinateTargetX	0	//координата цели по оси X в ИСК
#define	coordinateTargetY	0	//координата цели по оси Y в ИСК



#include "globals.h"
#include <stdlib.h>
#include <stdint.h>

//СТРУКТУРА КИНЕМАТИЧЕСКОГО СОСТОЯНИЯ АППАРАТА
typedef struct
{
	float aRawX; float aRawY; float aRawZ;	//ускорения в единицах g (в ССК)
	float gRawX; float gRawY; float gRawZ;	//угловые скорости в degps (в ССК)
	float mRawX; float mRawY; float mRawZ;	//косинусы углов вектора магнитного поля с осями ССК

	float aX; float aY; float aZ;		//ускорения в м/с^2 (ИСК)
	float vX; float vY; float vZ;		//скорости в м/с (ИСК)
	float sX; float sY; float sZ;		//перемещения в м (ИСК)
	float wX; float wY; float wZ;		//угловые скорости в 1/с (ИСК)

	//Матрица поворота ССК относительно ИСК
	float fXX1; float fXY1; float fXZ1;	//косинусы углов оси Х ИСК с осями ССК
	float fYX1; float fYY1; float fYZ1;	//косинусы углов оси Y ИСК с осями ССК
	float fZX1; float fZY1; float fZZ1;	//косинусы углов оси Z ИСК с осями ССК

}state;

//СТРУКТУРА КООРДИНАТ ОТСЛЕЖИВАЕМОЙ ЦЕЛИ
typedef struct
{
	float X;
	float Y;
}target;


extern uint8_t count;

//КИНЕМАИТЧЕСКОЕ СОСТОЯНИЕ АППАРАТА
extern state STATE;

//КОРДИНАТЫ ЦЕЛИ
extern target TARGET;

#endif /* GLOBALS_H_ */








































