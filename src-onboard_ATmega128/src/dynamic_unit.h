/*
 * dynamic_unit.h
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

/* МОДУЛЬ, ОТВЕЧАЮЩИЙ ЗА УПРАВЛЕНИЕ
 * ИСПОЛНИТЕЛЬНЫМИ ОРГАНАМИ АППАРАТА
 */

#ifndef DYNAMIC_UNIT_H_
#define DYNAMIC_UNIT_H_

#define DEV_INERT_MOM		1		//момент инерции аппарата по оси Z
#define WHEEL_INERT_MOM		1		//момент инерции маховика

#define WHEEL_SPEED_REG		OCR1A	//регистр ШИМ, управляющий двигателем
#define MAX_WHEEL_SPEED		200

#define DRV_PORT PORTC
#define DRV_DDR DDRC
#define WH1_C 0
#define WH1_D 1

//СТРУКТУРА КООРДИНАТ ОТСЛЕЖИВАЕМОЙ ЦЕЛИ
typedef struct
{
	float X;
	float Y;
}target;

//КОРДИНАТЫ ЦЕЛИ
extern target TARGET;




//ИНИЦИАЛИЗИРУЕТ ПРОГРАММУ ДИНАМИЧЕСКОГО УПРАВЛЕНИЯ АППАРАТОМ, А ТАК ЖЕ ДРАЙВЕРЫ ИСПОЛНИТЕЛЬНЫХ ОРГАНОВ
void dynamicInit();


//РАССЧИЫТВЕТ СКОРОСТЬ ВРАЩЕНИЯ ДВИГАТЕЛЯ-МАХОВИКА, ОБЕСПЕЧИВАЮЩУЮ ЗАДАННУЮ ОРИЕНТАЦИЮ АППАРАТА

//ПАРАМЕТР:		angVelocityX - скорость вращения аппарата вокруг оси Х (рад/с)
//ПАРАМЕТР:		angVelocityY - скорость вращения аппарата вокруг оси Y (рад/с)
//ПАРАМЕТР:		angVelocityZ - скорость вращения аппарата вокруг оси Z (рад/с)
//ВОЗВРАЩАЕТ:	угловая скорость двигателя-маховика (рад/с)
float calculateWheelSpeed(float angVelocityX, float angVelocityY, float angVelocityZ);


//РАССЧИЫТВЕТ УГОЛ ПОВОРОТА СЕРВОПРИВОДА, ДЛЯ ОТСЛЕЖИВАНИЯ ЗАДАННОЙ ЦЕЛИ

//ПАРАМЕТР:		positionX - координаты аппарата по оси X (м)
//ПАРАМЕТР:		positionY - координаты аппарата по оси Y (м)
//ПАРАМЕТР:		positionZ - координаты аппарата по оси Z (м)
//ПАРАМЕТР:		cosAngleX_Z1 - косинус угла между осью X ИСК и осью Z ССК (-)
//ПАРАМЕТР:		cosAngleY_Z1 - косинус угла между осью Y ИСК и осью Z ССК (-)
//ПАРАМЕТР:		cosAngleZ_Z1 - косинус угла между осью Z ИСК и осью Z ССК (-)
//ПАРАМЕТР:		targetX - координаты цели по оси X (м)
//ПАРАМЕТР:		targetZ - координаты цели по оси Y (м)
//ВОЗВРАЩАЕТ:	угол отклонения сервопривода (рад)
float calculateServoAngle(	float positionX,	float positionY,	float positionZ,
							float cosAngleX_Z1,	float cosAngleY_Z1,	float cosAngleZ_Z1,
							float targetX,		float targetY);


//ЗАДАЕТ УГЛОВУЮ СКОРОСТЬ ДВИГАТЕЛЮ-МАХОВИКУ

//ПАРАМЕТР:		WheelSpeed - угловая скорость двигателя-маховика (рад/с)
void setWheelSpeed(float WheelSpeed);


//ЗАДАЕТ УГЛОВУЮ СКОРОСТЬ ДВИГАТЕЛЮ-МАХОВИКУ

//ПАРАМЕТР:		ServoAngle - угол отклонения сервопривода (рад)
void setServoAngle(float ServoAngle);


#endif /* DYNAMIC_UNIT_H_ */
