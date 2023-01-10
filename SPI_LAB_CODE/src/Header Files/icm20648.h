/*
 * icm20648.h
 *
 *  Created on: Apr 14, 2022
 *      Author: Iskandar
 */

#ifndef HEADER_FILES_ICM20648_H_
#define HEADER_FILES_ICM20648_H_

#include "spi.h"
#include "HW_delay.h"

#define PWR_MGMT_1             0x06
#define PWR_MGMT_2             0x07
#define LP_CONFIG              0x05
#define ACCEL_WOM_THR          0x13
#define CALLBACK_NULL_VALUE    0
#define PWR_1_LP_TEMPDIS       0b00101000
#define PWR_2_ACCELEN_DISGYRO  0b00000111
#define LP_DUTY                0b00100000
#define ACCELERATION_THRESHOLD 0b00111100  //240mg in decimal converted
#define DEVICE_ADDRESS         0xE0
void icm_open();
void icm_read(uint32_t callback, USART_TypeDef *usart, uint32_t reg_address);
void icm_write(uint32_t callback, USART_TypeDef *usart, uint32_t reg_address, uint32_t data_to_write);
void icm_configure();

#endif /* HEADER_FILES_ICM20648_H_ */
