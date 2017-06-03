/*
 * i2c.h
 *
 *  Created on: 3 июн. 2017 г.
 *      Author: raketov
 */

#ifndef I2C_H_
#define I2C_H_

int i2c_init(void);
static s8 i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8  i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


#endif /* I2C_H_ */
