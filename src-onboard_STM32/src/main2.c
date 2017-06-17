/*
 * main2.c
 *
 *  Created on: 10 июня 2017 г.
 *      Author: developer
 */

//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "core_cm3.h"

#include <sd.h>
#include <dump.h>
#include "OV7670.h"
#include "i2c.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=72000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

//Порт A
#define VIDEO_DATA		GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
#define VIDEO_CLK		GPIO_Pin_8
#define VIDEO_RRST		GPIO_Pin_9
#define VIDEO_WRST		GPIO_Pin_10
#define VIDEO_HREF		GPIO_Pin_11
#define VIDEO_VSYNC	GPIO_Pin_12

//Порт B
#define VIDEO_OE		GPIO_Pin_0
#define VIDEO_WE		GPIO_Pin_1

// ----- functions() --------------------------------------------------------

typedef enum
{
	OV7670_START_FRAME	= 0xFF7C0000,
	OV7670_END_FRAME	= 0x007CFFFF,
	OV7670_SE_FRAME		= 0xFF7C0000007CFFFF	//end-start
}ov7670_flag;


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


uint16_t string_point = 0;
uint16_t frame_point = 0;

void reset_FIFO()
{
	GPIOB->BSRR |= VIDEO_OE | VIDEO_WE;
	GPIOA->BRR |= VIDEO_CLK;
	GPIOA->BSRR |= VIDEO_CLK;
	GPIOA->BRR |= VIDEO_RRST | VIDEO_WRST;
	for (int i = 0; i < 50; i++)
	{
		GPIOA->BRR |= VIDEO_CLK;
		GPIOA->BSRR |= VIDEO_CLK;
	}

	GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
	GPIOA->BRR |= VIDEO_CLK;
	GPIOA->BSRR |= VIDEO_CLK;
}

void exti_enable(FunctionalState state)
{
	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_init.NVIC_IRQChannelCmd = state;
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_init);
}

int main(int argc, char* argv[])
{
	//Включаем тактирование
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	blink_led_init();

	//Инициализация портов видеокамеры
	GPIO_InitTypeDef OV7670_initialise;

	OV7670_initialise.GPIO_Pin = 	VIDEO_DATA | VIDEO_HREF | VIDEO_VSYNC;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &OV7670_initialise);

	OV7670_initialise.GPIO_Pin = VIDEO_CLK | VIDEO_RRST | VIDEO_WRST;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &OV7670_initialise);

	OV7670_initialise.GPIO_Pin = VIDEO_OE | VIDEO_WE;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &OV7670_initialise);

	//Инициализация внешнего прерывания
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);		//Включение внешнего прерывания для HREF
	EXTI_InitTypeDef EXTI_init;											//Настройка линии 11 внешних прерываний
	EXTI_init.EXTI_Line = EXTI_Line11;
	EXTI_init.EXTI_LineCmd = ENABLE;
	EXTI_init.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_init.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_init);

	/*i2c_init();

	OV7670_init();
	trace_printf("%d", OV7670.init_success);
	OV7670_init_rgb565_qvga();*/

	dump_state_t stream_file;
	char filename[] = "video-";
	dump_init(&stream_file, filename);

	reset_FIFO();

	uint8_t buffer[512*15];
	ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;

	/*dump(&stream_file, &OV7670_FLAG, 8);
	while(1)
	{
		for (uint16_t i = 0; i < sizeof(buffer); i++)
		{
			if (i % 2 == 0)
				buffer[i] = 0b00011111;
			else
				buffer[i] = 0b00000000;
		}

		dump(&stream_file, buffer, sizeof(buffer));
		dump(&stream_file, &OV7670_FLAG, 8);
	}*/


	while (1)
	{
		if ((GPIOA->IDR & VIDEO_VSYNC))
		{
			if (!string_point) GPIOB->BRR |= VIDEO_WE;		//разрешаем запись в FIFO
			//exti_enable(ENABLE);

			while(string_point < 300)				//ждем, пока в FIFO запишется 300 строк
			{
				if (~(GPIOA->IDR) & VIDEO_HREF)
				{
				string_point++;
				}
			}

			GPIOB->BSRR |= VIDEO_WE;				//запрещаем запись в FIFO
			GPIOB->BRR |= VIDEO_OE;					//разрешаем чтение из FIFO
			//string_point = 0;

			while (frame_point < 50)				//записываем на SD 50 пакетов 512*15 байт
			{
				for (size_t i = 0; i < sizeof(buffer); i++)			//записываем кадр в buffer[]
				{
					GPIOA->BRR |= VIDEO_CLK;
					buffer[i] = (uint8_t)GPIO_ReadInputData(GPIOA);
					GPIOA->BSRR |= VIDEO_CLK;
				}
				dump(&stream_file, buffer, sizeof(buffer));				//отправляем содержимое буфера на SD
				frame_point++;
			}

			dump(&stream_file, &OV7670_FLAG, 8);	//отправляем флаг в конце кадра
			frame_point = 0;
			reset_FIFO();		//чистим FIFO
		}
	}

/*
//Инициализация UART
	GPIO_InitTypeDef PORTA_init_struct;

	//Настраиванм ногу TXD (PA9) как выход push-pull с альтернативной функцией
	PORTA_init_struct.GPIO_Pin = GPIO_Pin_9;
	PORTA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	PORTA_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &PORTA_init_struct);

	//Настраиваем пин 0 порта А
	PORTA_init_struct.GPIO_Pin = GPIO_Pin_0;
	PORTA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	PORTA_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &PORTA_init_struct);

	//Настраиваем UART
	USART_InitTypeDef UART_struct;
	UART_struct.USART_BaudRate 				= 9600;
	UART_struct.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	UART_struct.USART_Mode					= USART_Mode_Tx;
	UART_struct.USART_Parity				= USART_Parity_No;
	UART_struct.USART_StopBits				= USART_StopBits_1;
	UART_struct.USART_WordLength			= USART_WordLength_8b;
	//Инициализируем UART1
	USART_Init(USART1, &UART_struct);
	//Включаем UART1
	USART_Cmd(USART1, ENABLE);


	//Инициализация таймера
	TIM_TimeBaseInitTypeDef TIM_init_structure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	//TIM_init_structure.TIM_RepetitionCounter = 0x00;
	TIM_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
	//Устанавливаем предделитель (частота / предделитель = кол-во тактов в секунду)
	TIM_init_structure.TIM_Prescaler = 100;//50000 - 1;
	//Устанавливаем значение счетчика, по достижении которого он обнулится
	TIM_init_structure.TIM_Period = 1;
	TIM_TimeBaseInit(TIM2, &TIM_init_structure);
	//Разрешаем прерывание по переполнению таймера
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//НАЧИНАЕМ ОТСЧЕТ
	TIM_Cmd(TIM2, ENABLE);
	//Разрешаем прерывание от таймера
	NVIC_EnableIRQ(TIM2_IRQn);

	while(1)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		//ставим 1 на А0
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);	//ставим 0 на А0
		blink_led_on();
		blink_led_off();
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		//ставим 1 на А0
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);	//ставим 0 на А0
		//GPIOA->BSRR |= 1;
		//GPIOA->BRR |= 1;
	}* /

/ *
//Инициализация таймера
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
   //Устанавливаем предделитель (частота 24 МГц / предделитель = кол-во тактов в секунду)
   TIM6->PSC = 24000 - 1;
   //Устанавливаем значение счетчика, по достижении которого он обнулится
   TIM6->ARR = 1000;
   //Разрешаем прерывание от таймера
   TIM6->DIER |= TIM_DIER_UIE;
   //НАЧИНАЕМ ОТСЧЕТ
   TIM6->CR1 |= TIM_CR1_CEN;
   //
   //NVIC_EnableIRQ();* /

 / * while (1)
  {
	  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)
	  {}
	  USART_SendData(USART1, 'H');
  }*/
}


void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line11);
	string_point++;
	if (string_point >= 300)
	{
		GPIOB->BSRR |= VIDEO_WE;

		//exti_enable(DISABLE);
		string_point = 0;
	}
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------



