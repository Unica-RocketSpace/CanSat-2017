/*
 * sccb.h
 *
 *  Created on: 20 июня 2017 г.
 *      Author: developer
 */

#ifndef OV7670_SCCB_H_
#define OV7670_SCCB_H_

#define SIO_C_Pin			6
#define SIO_D_Pin			7

#define SCCB_CLOCK_SPEED	( 100000 )
#define SCCB_TIM_US_PERIOD	( 72 )



typedef struct
{
	uint32_t sccb_ClockSpeed;

}sccb_InitTypeDef;

typedef enum
{
	sccb_DataLine	= 4,
	sccb_ClockLine	= 3
}sccb_BusLines;

typedef enum
{
	HIGH	= 1,
	LOW		= 0
}sccb_BusState;


/*Инициализация шины (заполнение структуры sccb_TypeDef)*/
void sccb_InitStruct(sccb_InitTypeDef * sccb_Init);

/*Установка линии SIO_C или SIO_D в 0 или 1*/
void sccb_SetLine_(sccb_BusLines sccb_Line, sccb_BusState sccb_bus_state);

void sccb_DataLineInit(GPIOMode_TypeDef sccb_DataLineMode);
void sccb_ClockLineInit();


void sccb_SetDelay(int _us_DelayVal);

void sccb_TimInit();

/*Инициализация шины и прочего*/
void sccb_Init();

/*Генерирование сигнала START*/
void sccb_GenerateStart();



#endif /* OV7670_SCCB_H_ */
