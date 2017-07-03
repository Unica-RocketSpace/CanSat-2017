#include "camera.h"

#include "defs_hw.h"
#include "OV7670.h"
#include "tim_dma_control.h"
#include "diag/Trace.h"

static uint8_t _dummyBuffer[10];
static volatile ssize_t _hrefCntLeft = 0;
static volatile ssize_t _hrefSkipLeft = 0;

static EXTI_InitTypeDef _extiCfg =
{
	.EXTI_Line = EXTI_Line11,
	.EXTI_Mode = EXTI_Mode_Interrupt,
	.EXTI_Trigger = EXTI_Trigger_Falling,
	.EXTI_LineCmd = ENABLE,
};

int camera_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
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

	// Настраиваем прерывания EXTI на подсчет HREF
	EXTI_Init(&_extiCfg);

	// Настраиваем на NVIC прерывания EXTI
	NVIC_InitTypeDef NVIC_init;
	NVIC_init.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_init.NVIC_IRQChannelCmd = DISABLE; // потом включим
	NVIC_init.NVIC_IRQChannelPreemptionPriority = 0; // высший приоритет!
	NVIC_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_init);

	tdcs_init();

	/*
	int ret;
	struct ov7670_config cfg;
	cfg.clock_speed = 12;
	cfg.pclk_hb_disable = false;
	ret = ov7670_init(&cfg);
	if (ret)
		trace_printf("cam init_fails: %d\n", ret);

	return ret;
	*/

	return 0;
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


void camera_fifo_start_capture(ssize_t href_skip, ssize_t href_count)
{
	// Записываем сколько строк нам нужно загнать в фифо
	_hrefCntLeft = href_count;
	// И сколько пропустить перед этим
	_hrefSkipLeft = href_skip;

	// разрешаем прерывания для подсчета HREF-ов
	NVIC_EnableIRQ(EXTI15_10_IRQn);
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

size_t camera_fifo_lines_left()
{
	volatile size_t tmp;
	//__disable_irq();
	tmp = _hrefCntLeft;
	//__enable_irq();
	return tmp;
}


void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line11);

	if (_hrefSkipLeft >= 0)
	{
		_hrefSkipLeft--;
		if (_hrefSkipLeft == 0)
		{
			// разрешаем запись в фифо
			GPIOB->BSRR |=  VIDEO_WE;
		}
		return;
	}

	_hrefCntLeft--;
	if (0 == _hrefCntLeft)
	{
		GPIOB->BRR |= VIDEO_WE; // запрещаем запись в фифо
		// запрещаем прерывания
		NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
}
