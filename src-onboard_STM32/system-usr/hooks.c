/*
 * hooks.c
 *
 *  Created on: 1 июн. 2017 г.
 *      Author: snork
 */

//#if !defined(OS_USE_SEMIHOSTING) && !(__STDC_HOSTED__ == 0)

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <signal.h>

#include <stm32f10x_usart.h>
#include <stm32f10x_usart.h>

void __initialize_hardware(void);

void __initialize_hardware(void)
{
	SystemCoreClockUpdate();

	// требуется freertos-ом (http://www.freertos.org/RTOS-Cortex-M3-M4.html)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetPriorityGrouping(0);
}

