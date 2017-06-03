/*
 * dump.h
 *
 *  Created on: 05 окт. 2016 г.
 *      Author: snork
 */

#ifndef DUMP_H_
#define DUMP_H_

#include <stddef.h>
#include <ff.h>

#include <stm32f10x_conf.h>


typedef struct
{
   FIL file;
} dump_state_t;


void dump_init(dump_state_t * state, char * filename);
void dump(dump_state_t * state, const void * data, size_t datasize);



#endif /* DUMP_H_ */
