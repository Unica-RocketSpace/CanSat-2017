/*
 * sccb.c
 *
 *  Created on: 20 июня 2017 г.
 *      Author: developer
 */
#include <stm32f10x_rcc.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stdbool.h>

#include "defs_OV7670.h"
#include "sccb.h"


static TIM_TimeBaseInitTypeDef sccb_TimInit_struct =
{
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Prescaler = 0x0000,
		.TIM_Period = 0xFFFF - 1
};

static bool _tim_update_ = 0;


void sccb_InitStruct(sccb_InitTypeDef * sccb_Init)
{
	sccb_Init->sccb_ClockSpeed = SCCB_CLOCK_SPEED;
}

void sccb_SetLine_(sccb_BusLines sccb_Line, sccb_BusState sccb_bus_state)
{

	if (sccb_bus_state == HIGH)
		GPIOB->BSRR |= (1 << sccb_Line);
	else
		GPIOB->BRR |= (1 << sccb_Line);
}

void sccb_DataLineInit(GPIOMode_TypeDef sccb_DataLineMode)
{
	GPIO_InitTypeDef sccb_DataLineInit;
	sccb_DataLineInit.GPIO_Mode = sccb_DataLineMode;
	sccb_DataLineInit.GPIO_Pin 	= sccb_DataLine;
	sccb_DataLineInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &sccb_DataLineInit);
}

void sccb_ClockLineInit()
{
	/*Настраиваем ногу на вывод клока*/
	GPIO_InitTypeDef sccb_ClockLineInit;
	sccb_ClockLineInit.GPIO_Mode = GPIO_Mode_AF_PP;
	sccb_ClockLineInit.GPIO_Pin = sccb_ClockLine;	//B10
	sccb_ClockLineInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &sccb_ClockLineInit);
}


void sccb_TimInit()
{
	/*Инициализация таймера, управляющего SIO_C*/
	TIM_TimeBaseStructInit(&sccb_TimInit_struct);
	TIM_TimeBaseInit(TIM1, &sccb_TimInit_struct);
	TIM_SetCounter(TIM1, 0);

	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0; // высший приоритет!
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_init);
	NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void sccb_SetDelay(int _us_DelayVal)
{
	TIM_SetCounter(TIM1, 0);
	sccb_TimInit_struct.TIM_Period = _us_DelayVal * SCCB_TIM_US_PERIOD;
	TIM_TimeBaseInit(TIM1, &sccb_TimInit_struct);
	TIM_Cmd(TIM1, ENABLE);

	while (_tim_update_ == 0)	{}
	TIM_Cmd(TIM1, DISABLE);

}

void sccb_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/*Инициализация пинов 6 и 7 порта B (SIO_C и SIO_D)*/
	GPIO_InitTypeDef sccb_PinsInit;
	GPIO_StructInit(&sccb_PinsInit);
	sccb_PinsInit.GPIO_Mode =  GPIO_Mode_AF_OD;
	sccb_PinsInit.GPIO_Pin = sccb_DataLine | sccb_ClockLine;
	sccb_PinsInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &sccb_PinsInit);

	/*Запись настроек шины*/
	sccb_InitTypeDef sccb_BusInit;
	sccb_InitStruct(&sccb_BusInit);

	sccb_TimInit();

	sccb_DataLineInit(GPIO_Mode_Out_PP);
	sccb_ClockLineInit();

	sccb_SetLine_(sccb_DataLine, HIGH);
	sccb_SetLine_(sccb_ClockLine, HIGH);
}




void sccb_GenerateStart()
{
	sccb_SetLine_(sccb_DataLine, LOW);
	vTaskDelay(1);
	sccb_SetLine_(sccb_ClockLine, LOW);
}

void sccb_Write(uint8_t * data)
{

}

void TIM1_UP_IRQHandler()
{

	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	_tim_update_ = 1;
}

