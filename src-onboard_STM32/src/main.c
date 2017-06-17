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
#include "tim_dma_control.h"
#include "registers.h"



#define BUFFER_SIZE	(1280*6)
#define BUFFER_CNT	(50)		//640*300*2/BUFFER_SIZE

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


uint8_t buffer[BUFFER_SIZE];
size_t href_cnt = 0;
size_t buff_cnt = 0;

void reset_FIFO()
{
	GPIOB->BSRR |= VIDEO_OE | VIDEO_WE;
	tdcs_pull_data(buffer, 1);
	GPIOA->BRR |= VIDEO_RRST | VIDEO_WRST;
	tdcs_pull_data(buffer, sizeof(buffer));
	GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
	//GPIOA->BRR |= VIDEO_CLK;
	//GPIOA->BSRR |= VIDEO_CLK;
}

void exti_enable(FunctionalState state)
{
	EXTI_InitTypeDef exti;
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line11;
	exti.EXTI_LineCmd = ENABLE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&exti);

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
	tdcs_pull_data(buffer, 10);
	GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
}

void blink_led()
{
	static bool led_st = false;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_st ? RESET : SET);
	led_st = !led_st;
}



#include <FreeRTOS.h>
#include <task.h>



void task__(void * args)
{
	//xTaskCreate(task__2, "2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	for(;;)
	{
		blink_led_on();
		TickType_t ticks = xTaskGetTickCount();
		//trace_printf("ticks = %d\n", ticks);
		blink_led_off();
	}
}




int main()
{
	blink_led_init();

	xTaskCreate(task__, "1", configMINIMAL_STACK_SIZE, NULL, 2, NULL);


	vTaskStartScheduler();

	for(;;)
	{
		volatile int x = 0;
	}

	return 0;
}

int main_x(int argc, char* argv[])
{
	tdcs_init();
	blink_led_init();

	ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;
	dump_state_t stream_file;
	char filename[] = "video-";
	dump_init(&stream_file, filename);

	//Пишем на SD флаг начала кадра
	dump(&stream_file, &OV7670_FLAG, 8);

again:
	camera_init();

	href_cnt = 0;
	buff_cnt = 0;

	// ждем VSYNC
	while((GPIOA->IDR & VIDEO_VSYNC) != 0)
	{}
	while((GPIOA->IDR & VIDEO_VSYNC) == 0)
	{}
	//разрешаем запись в fifo
	GPIOB->BSRR |= VIDEO_WE;
	exti_enable(ENABLE);

	// сбрасываем указатель чтения
	GPIOA->BRR |= VIDEO_RRST;
	tdcs_pull_data(buffer, 100);
	GPIOA->BSRR |= VIDEO_RRST;

	//разрешаем чтение из FIFO
	GPIOB->BRR |= VIDEO_OE;

	// и читаем и пишем на флешку
	while (buff_cnt < BUFFER_CNT)
	{
		if (href_cnt >= 2)
		{
			tdcs_pull_data(buffer, sizeof(buffer));
			dump(&stream_file, buffer, sizeof(buffer));
			buff_cnt++;
		}

	}
	dump(&stream_file, &OV7670_FLAG, 8);
	blink_led();

	GPIOB->BSRR |= VIDEO_OE;

	// продолжаем
	goto again;


	return 0;
}

void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line11);
	href_cnt++;
	if (href_cnt >= 300)
	{
		GPIOB->BRR |= VIDEO_WE;
		exti_enable(DISABLE);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
