#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"




#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

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



typedef enum
{
	OV7670_START_FRAME	= 0xFF7C0000,
	OV7670_END_FRAME	= 0x007CFFFF,
	OV7670_SE_FRAME		= 0xFF7C0000007CFFFF	//end-start
} ov7670_flag;

static const ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;
static const char filename[] = "video-";

#define BUFFER_SIZE			(1280*5)
#define BUFFER_CNT 			(2)
#define BUFFER_READ_COUNT	(640*300*2/BUFFER_SIZE)		//640*300*2/BUFFER_SIZE

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


size_t href_cnt = 0;
static QueueHandle_t _fullBufferQ;
static QueueHandle_t _emptyBufferQ;
static dump_state_t stream_file;
static uint8_t _buffers[BUFFER_CNT][BUFFER_SIZE];

static uint8_t _dummyBuffer[10];

void reset_FIFO()
{
	GPIOB->BSRR |= VIDEO_OE | VIDEO_WE;
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
	GPIOA->BRR |= VIDEO_RRST | VIDEO_WRST;
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
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
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
	GPIOA->BSRR |= VIDEO_RRST | VIDEO_WRST;
}

void blink_led()
{
	static bool led_st = false;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_st ? RESET : SET);
	led_st = !led_st;
}




void cam_task(void* args);


void sd_task(void * args)
{
	(void)args;

	dump_init(&stream_file, filename);
	//Пишем на SD флаг начала кадра
	dump(&stream_file, &OV7670_FLAG, 8);

	_emptyBufferQ = xQueueCreate(BUFFER_CNT, sizeof(uint8_t*));
	_fullBufferQ = xQueueCreate(BUFFER_CNT, sizeof(uint8_t*));

	for (size_t i = 0; i < BUFFER_CNT; i++)
	{
		uint8_t * buffPtr = _buffers[i];
		xQueueSendToBack(_emptyBufferQ, &buffPtr, portMAX_DELAY);
	}

	xTaskCreate(cam_task, "cam", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	while(1)
	{
		int dump_buffers_cnt = 0;
		dump_buffers_cnt++;
		uint8_t * buffer;
		xQueueReceive(_fullBufferQ, &buffer, portMAX_DELAY);
		dump(&stream_file, buffer, sizeof(buffer));
		xQueueSend(_emptyBufferQ, &buffer, portMAX_DELAY);


		if (dump_buffers_cnt == 50)
			dump(&stream_file, &OV7670_FLAG, 8);

	}
}


void cam_task(void* args)
{
	(void)args;

again:
	camera_init();

	href_cnt = 0;

	taskENTER_CRITICAL();

	// ждем VSYNC
	while((GPIOA->IDR & VIDEO_VSYNC) != 0)
	{}
	while((GPIOA->IDR & VIDEO_VSYNC) == 0)
	{}

	taskEXIT_CRITICAL();

	//разрешаем запись в fifo
	GPIOB->BSRR |= VIDEO_WE;
	exti_enable(ENABLE);

	//разрешаем чтение из FIFO
	GPIOB->BRR |= VIDEO_OE;

	// сбрасываем указатель чтения
	GPIOA->BRR |= VIDEO_RRST;
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
	GPIOA->BSRR |= VIDEO_RRST;


	// и читаем и пишем на флешку
	size_t buff_cnt = 0;
	while (buff_cnt < BUFFER_READ_COUNT)
	{
		volatile size_t tmp;
		__disable_irq();
		tmp = href_cnt;
		__enable_irq();
		if (tmp >= 2)
		{
			uint8_t * buffer;
			xQueueReceive(_emptyBufferQ, &buffer, portMAX_DELAY);
			tdcs_pull_data(buffer, BUFFER_SIZE);
			xQueueSend(_fullBufferQ, &buffer, portMAX_DELAY);

			buff_cnt++;
		}
	}

	blink_led();

	// продолжаем
	goto again;
}



int main(int argc, char* argv[])
{
	tdcs_init();
	blink_led_init();

	xTaskCreate(sd_task, "sd", 5*configMINIMAL_STACK_SIZE, NULL, 2, NULL);


	vTaskStartScheduler();

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
