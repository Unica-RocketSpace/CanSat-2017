/*
 * i2c.h
 *
 *  Created on: 3 июн. 2017 г.
 *      Author: raketov
 */

#ifndef I2C_H_
#define I2C_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void sccb_init(void);
bool sccb_write(uint8_t reg_addr, uint8_t data);


#endif /* I2C_H_ */
