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
	for (int i = 0; i < 50000; i++)
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

void camera_init()
{
	GPIOB->BRR |=  VIDEO_WE;//запрещаем запись в FIFO и чтение из него
	GPIOB->BSRR |= VIDEO_OE;

	GPIOA->BRR |= VIDEO_RRST | VIDEO_WRST;			//переводим маркеры чтения и записи FIFO на начало
	for (int i = 0; i < 10; i++)					//clock-аем
	{
		GPIOA->BSRR |= VIDEO_CLK;
		GPIOA->BRR |= VIDEO_CLK;

	}
	GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
}


int
main(int argc, char* argv[])
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

	OV7670_initialise.GPIO_Pin = VIDEO_OE | VIDEO_WE | (1 << 6) | (1 << 7);
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &OV7670_initialise);


	uint8_t buffer[512*15];
	ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;
	dump_state_t stream_file;
	char filename[] = "video-";
	dump_init(&stream_file, filename);

	//Пишем на SD флаг начала кадра
	dump(&stream_file, &OV7670_FLAG, 8);


again:
	camera_init();

	// ждем VSYNC
	while((GPIOA->IDR & VIDEO_VSYNC) != 0)
	{}
	while((GPIOA->IDR & VIDEO_VSYNC) == 0)
	{}


	// VSYNC пришел, нужно срочно включать запись в фифо
	GPIOB->BSRR |= VIDEO_WE;

	// теперь считаем HREF-ы
	size_t href_cnt = 0;
	while(1)
	{
		// пока HREF в нуле ничего не делаем
		while((GPIOA->IDR & VIDEO_HREF) == 0)
		{}

		href_cnt++;

		// ждем пока опустится обратно
		while((GPIOA->IDR & VIDEO_HREF) != 0)
		{}

		if (href_cnt >= 300)
			break;
	}

	// и быстро запрещаем запись в фифо
	GPIOB->BRR |= VIDEO_WE | (1 << 7);

	// сбрасываем указатель чтения
	GPIOA->BRR |= VIDEO_RRST;						//переводим маркеры чтения и записи FIFO на начало
	for (int i = 0; i < 100; i++)					//clock-аем
	{
		GPIOA->BSRR |= VIDEO_CLK;
		GPIOA->BRR |= VIDEO_CLK;
	}
	GPIOA->BSRR |= VIDEO_RRST;

	//camera_init();
	//разрешаем чтение из FIFO
	GPIOB->BRR |= VIDEO_OE;

	// и читаем и пишем на флешку
	for (size_t j = 0; j < 50; j++)
	{
		for (size_t i = 0; i < sizeof(buffer); i++)
		{
			GPIOA->BRR |= VIDEO_CLK;
			buffer[i] = (uint8_t)GPIO_ReadInputData(GPIOA);
			GPIOA->BSRR |= VIDEO_CLK;
		}
		dump(&stream_file, buffer, sizeof(buffer));
		//camera_init();
	}
	dump(&stream_file, &OV7670_FLAG, 8);

	static bool led = false;
	if (led)
		blink_led_on();
	else
		blink_led_off();
	led = !led;

	// продолжаем
	goto again;


}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
