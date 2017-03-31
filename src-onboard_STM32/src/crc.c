/*
 * crc.c
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#include "crc.h"

// позаимствовано из sdfatlib для ардуино
// https://github.com/jbeynon/sdfatlib/blob/2ee66d98e28758783400617f477da37d4379d47d/SdFat/Sd2Card.cpp
uint8_t crc7(const void * data_, size_t data_size)
{
   uint8_t crc = 0;
   const uint8_t * data = (const uint8_t*)data_;

   for (size_t i = 0; i < data_size; i++)
   {
      uint8_t d = data[i];
      for (uint_fast8_t j = 0; j < 8; j++)
      {
         crc = (uint8_t)(crc << 1);
         if ((d & 0x80) ^ (crc & 0x80)) crc ^= 0x09;
            d = (uint8_t)(d << 1);
      }
   }
   return (uint8_t)(crc << 1) | 1;
}

