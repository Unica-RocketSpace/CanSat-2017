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
	OV7670_SE_FRAME		= 0xFF7C0000007CFFFF	//start-end
}ov7670_flag;

//Разгон МК
void Overclocking()
{
	RCC_HSICmd(ENABLE);							//включаем ВНУТРЕННИЙ генератор частоты
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);		//выбираем источником тактирования внутренний генератор
	RCC_PLLCmd(DISABLE);						//выключаем умножитель частоты
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	//устанавливаем множитель частоты (9) на ВНЕШНИЙ генератор
	RCC_PLLCmd(ENABLE);							//включаем умножитель частоты

	while ((RCC->CR & RCC_CR_PLLRDY) == 0);		//ждем запуска умножителя

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	//выбираем источником тактирования умножитель частоты

	SystemCoreClockUpdate();
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


uint16_t string_point = 0;

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
	GPIO_InitTypeDef OV7670_init;

	OV7670_init.GPIO_Pin = 	VIDEO_DATA | VIDEO_HREF | VIDEO_VSYNC;
	OV7670_init.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &OV7670_init);

	OV7670_init.GPIO_Pin = VIDEO_CLK | VIDEO_RRST | VIDEO_WRST;
	OV7670_init.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &OV7670_init);

	OV7670_init.GPIO_Pin = VIDEO_OE | VIDEO_WE;
	OV7670_init.GPIO_Speed = GPIO_Speed_50MHz;
	OV7670_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &OV7670_init);

	//Инициализация внешнего прерывания
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);		//Включение внешнего прерывания для HREF
	EXTI_InitTypeDef EXTI_init;											//Настройка линии 11 внешних прерываний
	EXTI_init.EXTI_Line = EXTI_Line11;
	EXTI_init.EXTI_LineCmd = ENABLE;
	EXTI_init.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_init.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_init);

	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_init);

	dump_state_t stream_file;
	char filename[] = "video-";
	dump_init(&stream_file, filename);

	GPIOA->BRR |= VIDEO_RRST | VIDEO_WRST;

	uint8_t buffer[512*16];
	uint16_t frame_point = 0;
	ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;

	while (1)
	{
		if (GPIOA->IDR & VIDEO_VSYNC)
		{
			GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
			GPIOB->BRR |= VIDEO_OE | VIDEO_WE;
			while (1)
			{
				if ((frame_point % 75) == 0)
				{
					dump(&stream_file, &OV7670_FLAG, 8);
					frame_point = 0;
				}
				for (size_t i = 0; i < sizeof(buffer); i++)
				{
					GPIOA->BRR |= VIDEO_CLK;
					buffer[i] = GPIO_ReadInputData(GPIOA) & 0xFF;
					GPIOA->BSRR |= VIDEO_CLK;
				}
				dump(&stream_file, buffer, sizeof(buffer));
				frame_point++;
			}
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
*/
	/*while(1)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		//ставим 1 на А0
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);	//ставим 0 на А0
		blink_led_on();
		blink_led_off();
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		//ставим 1 на А0
		//GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);	//ставим 0 на А0
		//GPIOA->BSRR |= 1;
		//GPIOA->BRR |= 1;
	}*/


  /*//Инициализация таймера
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
   //NVIC_EnableIRQ();*/


 /* while (1)
  {
	  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)
	  {}
	  USART_SendData(USART1, 'H');
  }*/
}

/*void TIM2_IRQHandler()
{
	blink_led_on();
	blink_led_off();
	//GPIOA->BSRR |= 1;
	//GPIOA->BRR |= 1;
	blink_led_on();
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)
	{}
	RCC_ClocksTypeDef clock_str;
	RCC_GetClocksFreq(&clock_str);
	uint16_t sys_frec = (uint16_t)clock_str.SYSCLK_Frequency;
	//uint16_t sys_frec = 0xff;
	//USART_SendData(USART1, sys_frec);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	blink_led_off();
}*/

void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line11);
	//blink_led_on();
	string_point++;
	//blink_led_off();
	if (string_point >= 300)
	{
		GPIOB->BSRR |= VIDEO_WE;

		NVIC_InitTypeDef NVIC_init;
		NVIC_init.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_init.NVIC_IRQChannelCmd = DISABLE;
		NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_init.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_init);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
