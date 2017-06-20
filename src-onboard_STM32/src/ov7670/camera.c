#include "camera.h"

#include "defs_hw.h"
#include "ov7670.h"
#include "tim_dma_control.h"
#include "diag/Trace.h"

static uint8_t _dummyBuffer[10];
static volatile size_t _hrefCntLeft = 0;

static EXTI_InitTypeDef _extiCfg =
{
	.EXTI_Line = EXTI_Line11,
	.EXTI_Mode = EXTI_Mode_Interrupt,
	.EXTI_Trigger = EXTI_Trigger_Falling,
	// .EXTI_LineCmd = ENABLE, // настраивается потом
};

int camera_init()
{
	// Настраиваем интерфейсные пины для фифо
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

	// чтение из фифо разрешаем на постоянку, так как оно мало на что влияет и управляется строго клоком
	GPIOB->BRR |= VIDEO_OE;

	// Разрешаем на NVICe прерывания для подсчета HREF-ов
	// на самом EXTI прерывания пока запрещены, поэтому работать они не будут
	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0; // высший приоритет!
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_init);

	tdcs_init();

	struct ov7670_config cfg;
	cfg.clock_speed = 24;
	cfg.pclk_hb_disable = false;
	int ret = ov7670_init(&cfg);
	if (ret)
		trace_printf("cam init_fails: %d\n", ret);

	return ret;
}


void camera_wait_vsync()
{
	while((GPIOA->IDR & VIDEO_VSYNC) != 0)
	{}
	while((GPIOA->IDR & VIDEO_VSYNC) == 0)
	{}
}


void camera_fifo_reset_write()
{
	GPIOB->BRR |=  VIDEO_WE;//запрещаем запись в FIFO

	GPIOA->BRR |= VIDEO_WRST ;		//переводим маркер записи FIFO на начало
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
	GPIOA->BSRR |= VIDEO_WRST;
}


void camera_fifo_start_capture(size_t href_count)
{
	// Записываем сколько строк нам нужно загнать в фифо
	_hrefCntLeft = href_count;

	// разрешаем запись в фифо
	GPIOB->BSRR |=  VIDEO_WE;

	// разрешаем прерывания для подсчета HREF-ов
	_extiCfg.EXTI_LineCmd = ENABLE;
	EXTI_Init(&_extiCfg);
}


void camera_fifo_reset_read()
{
	GPIOA->BRR |= VIDEO_RRST;
	tdcs_pull_data(_dummyBuffer, sizeof(_dummyBuffer));
	GPIOA->BSRR |= VIDEO_RRST;
}


void camera_fifo_read(void * buffer, size_t bufferSize)
{
	tdcs_pull_data(buffer, bufferSize);
}


void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line11);
	_hrefCntLeft--;
	if (0 == _hrefCntLeft)
	{
		GPIOB->BRR |= VIDEO_WE; // запрещаем запись в фифо
		// запрещаем прерывания
		_extiCfg.EXTI_LineCmd = DISABLE;
		EXTI_Init(&_extiCfg);
	}
}
