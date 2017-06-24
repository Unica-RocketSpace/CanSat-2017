/*
 * i2c.h
 *
 *  Created on: 3 июн. 2017 г.
 *      Author: raketov
 */

#ifndef I2C_H_
#define I2C_H_

int i2c_init(void);
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t  i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);


#endif /* I2C_H_ */
