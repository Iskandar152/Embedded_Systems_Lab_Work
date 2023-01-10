/*
 * spi.h
 *
 *  Created on: Apr 14, 2022
 *      Author: Iskandar
 */

#ifndef HEADER_FILES_SPI_H_
#define HEADER_FILES_SPI_H_

#include "brd_config.h"
#include "em_usart.h"
#include "sleep_routines.h"
#include "icm20648.h"
#include "app.h"
#define SPI_EM_BLOCK  EM2
#define NIL_VAL       0
typedef enum{
  initialize_spi,
  handle_junk,
  receive_send_data,
  receive,
  send_junk,
  finish
}SPI_DEFINED_STATES;

typedef struct{
  USART_Enable_TypeDef     enable;
  uint32_t                 refFreq;
  uint32_t                 baudrate;
  USART_Databits_TypeDef   databits;
  bool                     master;
  bool                     msbf;
  USART_ClockMode_TypeDef  clockMode;
  bool                     prsRxEnable;
  USART_PRS_Channel_t      prsRxCh;
  bool                     autoTx;
  bool                     autoCsEnable;
  bool                     csInv;
  uint8_t                  autoCsHold;
  uint8_t                  autoCsSetup;

  bool                     in_pin_rx;
  bool                     out_pin_tx;
  bool                     out_pin_cs;
  bool                     out_pin_clk;
}SPI_OPEN_STRUCT;

typedef struct{
  SPI_DEFINED_STATES     state;
  uint32_t               device_address;
  uint32_t               register_address;
  bool                   rw_bit;
  uint32_t               bytes_to_write;
  uint32_t               bytes_to_read;
  uint32_t               data_string;
  uint32_t               callback;
  USART_TypeDef          *usart_value;
  uint32_t               *pointer_to_result;
  bool                   busy_bit;
  bool                   junk_clear;
}SPI_STATE_MACHINE;
void spi_open(USART_TypeDef *usart, SPI_OPEN_STRUCT *usart_setup);
void USART3_RX_IRQHandler();
void USART3_TX_IRQHandler();
void spi_start(USART_TypeDef *usart, uint32_t *result_address, uint32_t register_address, uint32_t data_to_write,  uint32_t callback_value, uint32_t read_write);
void spi_state_machine();
void usart_end();
void spi_rxdatav(USART_TypeDef *usart);

#endif /* HEADER_FILES_SPI_H_ */
