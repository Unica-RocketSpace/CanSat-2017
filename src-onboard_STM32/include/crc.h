/*
 * crc.h
 *
 *  Created on: 29 июля 2016 г.
 *      Author: snork
 */

#ifndef CRC_H_
#define CRC_H_

#include <stddef.h>
#include <stdint.h>

uint8_t crc8(const void * data, size_t data_size);
uint8_t crc7(const void * data, size_t data_size);

#endif /* CRC_H_ */
