/*
 * Si1133.h
 *
 *  Created on: Feb 22, 2022
 *      Author: Iskandar
 */

#ifndef SI1133_HG
#define SI1133_HG
#include "i2c.h"
#include "HW_delay.h"
#include "app.h"

void Si1133_open(void);
void Si1133_read(uint32_t callback_to_pass, I2C_TypeDef *i2c, uint32_t reg_address);
void Si1133_write(uint32_t callback_to_pass,I2C_TypeDef *i2c, uint32_t reg_address, uint32_t data_to_write);
void Si1133_configure();
void Si1133_FORCE();
void Si1133_RETURN();
uint32_t Si1133_return_data();

#endif /* HEADER_FILES_SI1133_H_ */
