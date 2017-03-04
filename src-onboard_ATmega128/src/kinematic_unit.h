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


//КИНЕМАИТЧЕСКОЕ СОСТОЯНИЕ АППАРАТА
extern state STATE;




//ИНИЦИАЛИЗИРУЕТ ПРОГРАММУ КИНЕМАТИЧЕСКОГО СОСТОЯНИЯ АППАРАТА, А ТАК ЖЕ ДРАЙВЕРЫ ВНЕШНИХ УСТРОЙСТВ

void kinematicInit();


//РАССЧИТЫВАЕТ ТЕКУЩЕЕ СОСТОЯНИЕ АППАРАТА

void reconState();


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
