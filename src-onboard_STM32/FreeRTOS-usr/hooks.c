
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


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
