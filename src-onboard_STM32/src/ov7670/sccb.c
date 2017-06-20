/*
 * sccb.c
 *
 *  Created on: 20 июня 2017 г.
 *      Author: developer
 */

#include "defs_OV7670.h"
#include <stm32f10x_rcc.h>
#include "sccb.h"

void sccb_InitStruct(sccb_InitTypeDef * sccb_Init)
{
	sccb_Init->sccb_ClockSpeed = SCCB_CLOCK_SPEED;
}

void sccb_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/*Инициализация пинов 6 и 7 порта B (SIO_C и SIO_D)*/
	GPIO_InitTypeDef sccb_pins_init;
	GPIO_StructInit(&sccb_pins_init);
	sccb_pins_init.GPIO_Mode =  GPIO_Mode_AF_OD;
	sccb_pins_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	sccb_pins_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &sccb_pins_init);

	sccb_InitTypeDef sccb_bus_init;
	sccb_InitStruct(&sccb_bus_init);


}

void sccb_GenerateStart()
{

}


