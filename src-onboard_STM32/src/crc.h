/*
 * crc.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#ifndef CRC_H_
#define CRC_H_

#include <stddef.h>
#include <stdint.h>

// позаимствовано из sdfatlib для ардуино
// https://github.com/jbeynon/sdfatlib/blob/2ee66d98e28758783400617f477da37d4379d47d/SdFat/Sd2Card.cpp
uint8_t crc7(const void * data_, size_t data_size);

#endif /* CRC_H_ */
