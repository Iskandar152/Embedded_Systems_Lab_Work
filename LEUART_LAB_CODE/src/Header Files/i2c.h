/*
 * i2c.h
 *
 *  Created on: Feb 22, 2022
 *      Author: Iskandar
 */

#ifndef I2C_HG
#define I2C_HG

//Includes
#include "em_i2c.h"
#include "stdbool.h"
#include "sleep_routines.h"

//Defines
#define ZERO_VALUE                  0
#define SEND_CMD                    0x0B
#define PARAM_SET                   0b10000000
#define ADCCONFIG0                  0x2
#define ZERO_VALUE                  0
#define WHITE_LIGHT_SENSOR          0b00001011
#define I2C_EM_BLOCK  EM2
#define SI1133_CHANNEL0_ADDRESS     0x01
#define SI1133_CHAN_LIST            0x01
#define SI1133_ADDRESS              0x55
#define SI1133_PARTID_ADDRESS       0x00
#define SI1133_RESPONSE0_ADDRESS    0x11
#define SI1133_INPUT0_ADDRESS       0x0A
#define SHIFTED_ADDRESS_READ        (SI1133_ADDRESS<<1) | 1
#define SHIFTED_ADDRESS_WRITE       (SI1133_ADDRESS<<1) | 0
#define SI1133_HOSTOUT0             0x13
#define READ_BIT                    1
#define BYTE_GIVEN                  1
#define NULL_CALL_BACK              0
#define MASK_NEEDED                 0x0F
typedef enum{
  initialize,
  sending_address,
  resending_data,
  reading,
  writing,
  end
}DEFINED_STATES;

typedef struct{
  bool                 enable;    //enable I2C peripheral when initialization completed
  bool                 master;    //Set to controller (true) or Target (false) mode
  uint32_t             refFreq;   //I2C reference clock assumed when configuring bus frequency setup
  uint32_t             freq;      //(Max) I2C bus frequency use
  I2C_ClockHLR_TypeDef clhr;      //Clock low/high ratio control
  bool                 out_pin_scl_en;
  uint32_t             out_pin_scl;
  bool                 out_pin_sda_en;
  uint32_t             out_pin_sda;

}I2C_OPEN_STRUCT;

typedef struct{
  DEFINED_STATES       current_state;
  volatile bool                 busy;

  I2C_TypeDef          *i2c_used;
  uint32_t             device_address;
  uint32_t             register_address;
  uint32_t             *pointer_to_result;
  uint32_t             bytes_to_transfer;
  uint32_t             rw_type;   //read/write bit
  uint32_t             callback;
  uint32_t             bytes_to_read;
  uint32_t             bytes_to_write;
  uint32_t             writing_data;


}I2C_STATE_MACHINE;

void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *i2c_setup);
void i2c_bus_reset(I2C_TypeDef *i2c);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void i2c_start(I2C_TypeDef *i2c,uint32_t *result_address, uint32_t rw, uint32_t register_address,uint32_t callback_value, uint32_t data_to_write);
void i2c_ack();
void ending();
void i2c_rx_data(I2C_TypeDef *i2c);
bool check_busy(I2C_TypeDef *i2c);
#endif /* HEADER_FILES_I2C_H_ */
