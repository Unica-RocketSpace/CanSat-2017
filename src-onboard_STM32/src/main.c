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


uint8_t _buffer[512*15] = {0x00};

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


void setup_timers()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// НАСТРОЙКА ТАЙМЕРА 2 (тот, который пинает DMA)
	// ===============================================================
	// ===============================================================
	// ===============================================================
	// настраиваем таймер для генерации выводной частоты
	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = 0x0000;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 0x05; // два такта таймера до переполнения. это даст нам частоту в 72/2 = 36 мгц?
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	// tim.TIM_RepetitionCounter не используется на нашем таймере
	TIM_TimeBaseInit(TIM2, &tim);

	// настраиваем PWM, чтобы формировать выходной клок
	// Используем третий канал, так как он позволяет расталкивать DMA по его первому каналу
	TIM_OCInitTypeDef tim_oc;
	TIM_OCStructInit(&tim_oc);
	tim_oc.TIM_OCMode = TIM_OCMode_PWM1; // FIXME: Почему pwm1?
	tim_oc.TIM_OutputState = TIM_OutputState_Enable; // включаем вывод на пин
	//tim_pwm.TIM_OCNIdleState // только для таймеров 1, 8
	tim_oc.TIM_Pulse = 0x02; // значение в CCR регистре
	tim_oc.TIM_OCPolarity = TIM_OCPolarity_High; // FIXME: Тут нужно подумать
	//tim_pwm.TIM_OCNPolarity; // только для таймеров 1, 8
	//tim_pwm.TIM_OCIdleState; // только для таймеров 1, 8
	//tim_pwm.TIM_OCNIdleState; // только для таймеров 1, 8
	TIM_OC3Init(TIM2, &tim_oc);

	// разрешаем таймеру обращаться к этому каналу DMA по событию совпадения счетчика с 3им CCR регистром
	// для прогона одного байта через DMA
	//TIM_DMAConfig(TIM2, TIM_DMABase_CCR3, TIM_DMABurstLength_1Byte);
	//TIM_DMACmd(TIM2, TIM_DMA_CC3, ENABLE);

	// РАзрешаем этому таймеру управлять ведомыми таймерами
	//TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	// Выдаем им сигнал UPDATE
	//TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
		// Включаем для этого таймепра входной триггер с таймера 3
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
	// Разрешаем другим таймерам управлять нашим клоком режимом гейтирования
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);




	// НАСТРОЙКА ТАЙМЕРА 3 (тот, который выключает таймер 2)
	// ===============================================================
	// ===============================================================
	// ===============================================================
	//настраиваем таймер для отсчета количества прерываний таймера 2
	TIM_TimeBaseInitTypeDef tim_cnt;
	TIM_TimeBaseStructInit(&tim_cnt);
	tim_cnt.TIM_Prescaler = 0x0000;
	tim_cnt.TIM_CounterMode = TIM_CounterMode_Up;
	tim_cnt.TIM_Period = 0xFFFF-1; // FIXME: Оформить как макрос
	tim_cnt.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &tim_cnt);

	// настраиваем PWM, чтобы формировать выходной клок
	// Используем третий канал, так как он позволяет расталкивать DMA по его первому каналу
	TIM_OCStructInit(&tim_oc);
	tim_oc.TIM_OCMode = TIM_OCMode_PWM2; // FIXME: В чем разница между режимами?
	tim_oc.TIM_OutputState = TIM_OutputState_Enable;
	//tim_pwm.TIM_OCNIdleState // только для таймеров 1, 8
	tim_oc.TIM_Pulse = 1280 * 5; // значение в CCR регистре
	tim_oc.TIM_OCPolarity = TIM_OCPolarity_High; // FIXME: Тут нужно подумать
	//tim_pwm.TIM_OCNPolarity; // только для таймеров 1, 8
	//tim_pwm.TIM_OCIdleState; // только для таймеров 1, 8
	//tim_pwm.TIM_OCNIdleState; // только для таймеров 1, 8
	TIM_OC1Init(TIM3, &tim_oc);

	// Разрешаем ему управлять ведомыми таймерами
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	// Передаем им наш сигнал с CCR1
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC1Ref);
	// Сами будем использовать сигнал с таймера 2
	//TIM_SelectInputTrigger(TIM3, TIM_TS_ITR1);
	// Будем слушать его клоки и тикать
	//TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);

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



	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	GPIO_InitTypeDef p;
	p.GPIO_Mode = GPIO_Mode_AF_PP;
	p.GPIO_Pin = GPIO_Pin_10;
	p.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &p);

	p.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &p);


	setup_timers();

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM2, ENABLE);


	while(1)
	{}





	//blink_led_init();

	//Инициализация портов видеокамеры
	GPIO_InitTypeDef OV7670_initialise;

	OV7670_initialise.GPIO_Pin = VIDEO_DATA | VIDEO_HREF | VIDEO_VSYNC;
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

	// переключаем таймер2 на альтернативные пины
	GPIO_InitTypeDef PortB;
	// включаем на вывод пин CCR3
	GPIO_StructInit(&PortB);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	PortB.GPIO_Mode = GPIO_Mode_AF_OD; // FIXME: для отладки на лампочке. Заменить на AF_PP
	PortB.GPIO_Pin = GPIO_Pin_10;
	PortB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &PortB);

	/*
	// настраиваем DMA для чтения данных по клоку, генерируемому таймером
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef dma;
	dma.DMA_BufferSize = sizeof(_buffer);
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_M2M = DMA_M2M_Disable;
	dma.DMA_MemoryBaseAddr = (uint32_t)_buffer;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->IDR;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_High;

	// Настраиваем первый канал DMA и включаем его
	DMA_Init(DMA1_Channel1, &dma);
	DMA_Cmd(DMA1_Channel1, ENABLE);




	// FIXME: для отладки - включаем прерывание по завершению прогона блока с DMA
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	NVIC_InitTypeDef dma_nvic;
	dma_nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	dma_nvic.NVIC_IRQChannelPreemptionPriority = 0;
	dma_nvic.NVIC_IRQChannelSubPriority = 0;
	dma_nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&dma_nvic);
	*/



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
	GPIOB->BRR |= VIDEO_WE;

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
