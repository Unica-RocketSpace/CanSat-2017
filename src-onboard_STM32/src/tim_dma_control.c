/*
 * tim_dma_control.c
 *
 *  Created on: 15 июня 2017 г.
 *      Author: developer
 */
//АВТОР - snork


#include <stdlib.h>
#include <assert.h>
#include <stm32f10x_conf.h>

#include <FreeRTOS.h>
#include <task.h>

#include "tim_dma_control.h"
#include "registers.h"

#define DMA_KICK_PERIOD (20)


// Базовые параметры синхротаймера
static TIM_TimeBaseInitTypeDef _kick_tim_params;
static TIM_OCInitTypeDef _kick_ocr_params;

// параметры таймера, отключаещего базовый
static TIM_TimeBaseInitTypeDef _gate_tim_params;
static TIM_OCInitTypeDef _gate_ocr_params;

// настройки DMA
static DMA_InitTypeDef _dma_params;

static TaskHandle_t _thisTaskHandle = NULL;


void tdcs_init()			//TIM-DMA CONTROLLING SYSTEM
{
	// Тактируем все используемое железо
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Настраиваем параметры, которые затем будем применять к таймерам как можно быстрее!

// НАСТРОЙКИ ТАЙМЕРА 2 (тот, который пинает DMA)
// ===============================================================
// ===============================================================
	// настраиваем таймер для генерации выводной частоты
	TIM_TimeBaseStructInit(&_kick_tim_params);
	_kick_tim_params.TIM_ClockDivision = TIM_CKD_DIV1;
	_kick_tim_params.TIM_Prescaler = 0x0000;
	_kick_tim_params.TIM_CounterMode = TIM_CounterMode_Up;
	_kick_tim_params.TIM_Period = DMA_KICK_PERIOD;

	// настраиваем PWM, чтобы формировать выходной клок
	// Используем третий канал, так как он позволяет расталкивать DMA по его первому каналу
	TIM_OCStructInit(&_kick_ocr_params);
	_kick_ocr_params.TIM_OCMode = TIM_OCMode_PWM1;
	_kick_ocr_params.TIM_OutputState = TIM_OutputState_Enable; // включаем вывод на пин
	_kick_ocr_params.TIM_Pulse = DMA_KICK_PERIOD / 2;
	_kick_ocr_params.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_TimeBaseInit(TIM2, &_kick_tim_params);
	TIM_OC3Init(TIM2, &_kick_ocr_params); 	// используем CCR3, так как оно удачно связано с DMA1-1

	// разрешаем ТИМ2 пинать ДМА1-1 на перекачку одного байта по событию CCR3
	TIM_DMAConfig(TIM2, TIM_DMABase_CCR3, TIM_DMABurstLength_1Byte);
	TIM_DMACmd(TIM2, TIM_DMA_CC3, ENABLE);

	// мы хотим передавать наши такты таймеру 3, чтобы он считал их и выключал нас аппаратно по завершению
	// Разрешаем этому таймеру передавать сигналы "ведомым"
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	// Выдаем им сигнал UPDATE
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	// чтобы позволить таймеру 3 выключать нас, включаем себе режим гейтирования
	// Включаем для этого таймера входной триггер с таймера 3 на управление в режиме гейтирования
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);



// НАСТРОЙКИ ТАЙМЕРА 3 (тот, который выключает таймер 2)
// ===============================================================
// ===============================================================
	//настраиваем таймер для отсчета количества прерываний таймера 2
	TIM_TimeBaseStructInit(&_gate_tim_params);
	_gate_tim_params.TIM_ClockDivision = TIM_CKD_DIV1;
	_gate_tim_params.TIM_Prescaler = 0x0000;
	_gate_tim_params.TIM_CounterMode = TIM_CounterMode_Up;
	_gate_tim_params.TIM_Period = 0xFFFF-1; // ведем таймер до максимума, нас интересует CCR канал, который и будет задавать гейтирование

	// настраиваем CCR канал, чтобы при помощи него "гейтировать" ТИМ2 и вовремя его выключать,
	// когда он натикает достаточно тактов
	// используем первый CCR канал. Почему бы и нет?
	TIM_OCStructInit(&_gate_ocr_params);
	_gate_ocr_params.TIM_OCMode = TIM_OCMode_PWM1; // Именно в этом режиме наш гейтирующий сигнал будет == 1 по старту таймеру
	_gate_ocr_params.TIM_OutputState = TIM_OutputState_Enable;/*TIM_OutputState_Disable*///TIM_OutputState_Enable;
	_gate_ocr_params.TIM_Pulse = 0x100; // значение в CCR регистре == количество перекачиваемых байт,
	_gate_ocr_params.TIM_OCPolarity = TIM_OCPolarity_High; // походу этот параметр влияет только на вывод на пине
	TIM_OC1Init(TIM3, &_gate_ocr_params); // используем CCR1, потому что почему бы и нет?

	TIM_TimeBaseInit(TIM3, &_gate_tim_params);

	// Мы хотим, чтобы этот таймер "гейтировал" ТИМ2 и вовремя его выключал
	// Разрешаем этому таймеру передавать сигналы "ведомым"
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	// Передаем им наш сигнал с CCR1
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC1Ref);

	// Себя же настраиваем как ведомого от TIM2 и настраиваемся на то, что будем считать его входные импульсы
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR1);

// Настройки DMA
// ===============================================================
// ===============================================================
	// настраиваем DMA для чтения данных по клоку, генерируемому таймером
	DMA_StructInit(&_dma_params);
	_dma_params.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->IDR;
	// _dma_params.DMA_MemoryBaseAddr = ..; // настраивается потом
	_dma_params.DMA_DIR = DMA_DIR_PeripheralSRC;
	//_dma_params.DMA_BufferSize = ...; // настраивается потом
	_dma_params.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	_dma_params.DMA_MemoryInc = DMA_MemoryInc_Enable;
	_dma_params.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//;DMA_PeripheralDataSize_Byte
	_dma_params.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	_dma_params.DMA_Mode = DMA_Mode_Normal;
	_dma_params.DMA_Priority = DMA_Priority_VeryHigh;
	_dma_params.DMA_M2M = DMA_M2M_Disable;


// Настройки пинов, они не меняются, настроим их сразу
// ===============================================================
// ===============================================================
	// Пин для вывода синхроклока с TIM2
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	GPIO_InitTypeDef p;
	p.GPIO_Mode = GPIO_Mode_AF_PP;
	p.GPIO_Pin = VIDEO_CLK;	//B10
	p.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &p);

/*	// Пин для вывода синхроклока с TIM3
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	p.GPIO_Mode = GPIO_Mode_AF_PP;
	p.GPIO_Pin = TIM3_OUT_REMAP_REG;	//A6
	p.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &p);*/

	GPIO_InitTypeDef OV7670_initialise;
	OV7670_initialise.GPIO_Pin = VIDEO_DATA | VIDEO_HREF | VIDEO_VSYNC;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &OV7670_initialise);

	OV7670_initialise.GPIO_Pin = VIDEO_RRST | VIDEO_WRST;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &OV7670_initialise);

	OV7670_initialise.GPIO_Pin = VIDEO_OE | VIDEO_WE;
	OV7670_initialise.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_initialise.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &OV7670_initialise);


	// Настройки NVIC для прерываний от DMA. Так же не меняются, настроим сразу
	// ===============================================================
	// ===============================================================
	NVIC_InitTypeDef dma_it_nvic;
	dma_it_nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	dma_it_nvic.NVIC_IRQChannelPreemptionPriority = 0; 	// FIXME: Уточнить в контексте RTOS
	dma_it_nvic.NVIC_IRQChannelSubPriority = 0;			// FIXME: Уточнить в контексте RTOS
	dma_it_nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&dma_it_nvic);
}


void tdcs_pull_data(void * buffer, size_t buffer_size)
{
	// включение ТИМ3
	// ========================================================================
	// так же используем прекешированные настройки, но тут уже фигуруриет размер буфера
	TIM_SetCompare1(TIM3, buffer_size);


	// Включение ДМА
	// ========================================================================
	// Указываем параметры буфера
	_dma_params.DMA_BufferSize = buffer_size;
	_dma_params.DMA_MemoryBaseAddr = (uint32_t)buffer;
	// все настраиваем
	DMA_Init(DMA1_Channel1, &_dma_params);
	// и включаем
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// включаем прерывания от DMA
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // на успешную передачу
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TE, ENABLE);	// на ошибку при передаче

	// Настройка железа завершена!
	// все запускаем

	_thisTaskHandle = xTaskGetCurrentTaskHandle();

	// порядок важен
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM2, ENABLE);


	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	NVIC_SetPriority(DMA1_Channel1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	uint32_t transferResult;
	xTaskNotifyWait(0xFFFFFFFF, 0x00000000, &transferResult, portMAX_DELAY);


	// Все выключаем
	DMA_DeInit(DMA1_Channel1);
	//TIM_DeInit(TIM2);
	//TIM_DeInit(TIM3);

	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM3, 0);


	// завершились
	return;
}

void DMA1_Channel1_IRQHandler(void)
{
	BaseType_t woken = 0;

	if (SET == DMA_GetITStatus(DMA1_IT_TC1))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		vTaskNotifyGiveFromISR(_thisTaskHandle, &woken);
		_thisTaskHandle = NULL;
		portYIELD_FROM_ISR(woken);
	}
	else
	{
		abort(); // чет какая-то фигня случилась
	}
}


