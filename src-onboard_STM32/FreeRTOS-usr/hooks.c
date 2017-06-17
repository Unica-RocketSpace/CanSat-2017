
/*
 * freertos_hooks.c
 *
 *  Created on: 02 окт. 2016 г.
 *      Author: snork
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#pragma GCC diagnostic pop


#include <diag/Trace.h>

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName);

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName)
{
	(void)xTask; (void)pcTaskName;
	trace_printf("stack_overflow at %s\n", pcTaskName);
	abort();
}
