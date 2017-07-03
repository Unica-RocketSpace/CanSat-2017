/*
 * tim_dma_control.h
 *
 *  Created on: 15 июня 2017 г.
 *      Author: developer
 */
//АВТОР - snork

#ifndef TIM_DMA_CONTROL_H_
#define TIM_DMA_CONTROL_H_

#include <stddef.h>

void tdcs_init();
void tdcs_pull_data(void * buffer, size_t buffer_size);

#endif /* TIM_DMA_CONTROL_H_ */
