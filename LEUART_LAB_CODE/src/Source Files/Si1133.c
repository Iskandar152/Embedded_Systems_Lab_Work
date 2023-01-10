/**
 * @file
 *  Si1133.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  February 11, 2022
 * @brief
 *  Setting up our Si1133 sensor with proper interrupt triggering
 *  and reading and writing
 *
 */

#include "Si1133.h"
/*
#include "i2c.h"
#include "HW_delay.h"
#include "app.h"
*/
uint32_t data;
/***************************************************************************//**
 * @brief
 *   We set up our Si1133 device with the proper timing, routing, and interrupts
 *   through this function
 * @details
 *   A local I2C_OPEN struct is used to set up the device. The frequency is set to
 *   I2C_FREQ_FAST_MAX and the clock is set to i2cCLOCKHLRAsymetric. We route on
 *   base SDA and SCL value of LOC17. An I2C_open function is called that sets up
 *   our routing, interrupts, and bus resets
 *
 *
 * @param[in] none
 *
 ******************************************************************************/
void Si1133_open(){
  timer_delay(48.88);
  I2C_OPEN_STRUCT local_open_struct;

  local_open_struct.enable = true;
  local_open_struct.master = true;
  local_open_struct.refFreq = 0;
  local_open_struct.freq = I2C_FREQ_FAST_MAX;
  local_open_struct.clhr = i2cClockHLRAsymetric ;
  local_open_struct.out_pin_scl_en = true;
  local_open_struct.out_pin_scl = I2C_ROUTELOC0_SCLLOC_LOC17;  //Choosing I2C1 for now
  local_open_struct.out_pin_sda_en = true;
  local_open_struct.out_pin_sda = I2C_ROUTELOC0_SDALOC_LOC17;
  i2c_open(I2C1, &local_open_struct);
  Si1133_configure();
}
/***************************************************************************//**
 * @brief
 *   The si133_read function is called when we want to initiate a read from the
 *   sensor
 * @details
 *   The i2c_start function is called and passed the necessary data for it
 *   to do a proper read from the sensor
 *
 * @param[in] uint32_t callback_to_pass,
 *   Specifies the callback to pass for the scheduler
 * @param[in] I2C_TypeDef *i2c
 *   Specifies which I2C peripheral will be used for reading from the sensor
 *
 ******************************************************************************/
void Si1133_read(uint32_t callback_to_pass, I2C_TypeDef *i2c,uint32_t reg_address){
  i2c_start(i2c,&data,SHIFTED_ADDRESS_READ, reg_address,callback_to_pass,ZERO_VALUE);

}
/***************************************************************************//**
 * @brief
 *   Simply returns the data it fetches from the device
 * @details
 *   Simple
 * @note
 *   Called within our app.c when the data is needed
 *
 * @param[in] none
 *
 ******************************************************************************/
uint32_t Si1133_return_data(){
  return data;
}

/***************************************************************************//**
 * @brief
 *   Writes a data_value to a specific register within our Si1133 device
 * @details
 *   The write function takes in data regarding the location of the register within
 *   the Si1133 device and the data to write to it. The i2c_start function gets called
 *   with the automatically inputed ones based on whether it's a write or read command
 *
 *
 * @param[in] uint32_t callback_to_pass
 *   The call back value to pass
 *
 * @param[in] I2C_TypeDef *i2c
 *   The I2C peripheral that we will be writing to
 *
 * @param[in] uint32_t reg_address
 *   The address of the register we are writing to
 *
 * @param[in] uint32_t data_to_write
 *   The exact data that will be written
 ******************************************************************************/
void Si1133_write(uint32_t callback_to_pass,I2C_TypeDef *i2c, uint32_t reg_address, uint32_t data_to_write){
  i2c_start(i2c,&data,SHIFTED_ADDRESS_WRITE,reg_address,callback_to_pass, data_to_write);
}

/***************************************************************************//**
 * @brief
 *   The Si1133_configure goes through the steps required to request data from the
 *   white sensor light so that we can make comparisons in our Si1133_test function
 * @details
 *   Writes ask for the white light sensor data, and the response goes to
 *   the Si1133 response0 register. We also enable channel0. The exact steps are commented
 *   within the function
 *
 *
 * @param[in] none
 *
 ******************************************************************************/
void Si1133_configure(){
  uint32_t response0_value;
  uint32_t para_command;
  uint32_t response0_value2;
  uint32_t chan_list_command;
  uint32_t response0_value3;
  //Obtain current command count from RESPONSE0 register
  Si1133_read(NULL_CALL_BACK, I2C1, SI1133_RESPONSE0_ADDRESS);

  //Check if the state machine is busy
  while(check_busy(I2C1));

  //Store the value of CMD_CTR from RESPONSE0 into a local variable
  response0_value = (Si1133_return_data() & MASK_NEEDED);

  //Write the value needed to set ADCMUX to INPUT0 register
  Si1133_write(NULL_CALL_BACK, I2C1, SI1133_INPUT0_ADDRESS,WHITE_LIGHT_SENSOR);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Specify Parameter Table Write command ORED with ADCCONFIG0
  //while writing to COMMAND register
  //Based on page 14 of data sheet, PARAMSET is 0b10xxxxxx which is then ORED with
  //the proper command. In this case ADCCONFIG0 which is 0x2.
  //To access parameter table, need to write 0b10010101 to COMMAND which is address 0x0B
  para_command = PARAM_SET | ADCCONFIG0;
  Si1133_write(NULL_CALL_BACK, I2C1, SEND_CMD, para_command);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Read RESPONSE0 register to verify COMMAND register write executed successfully
  Si1133_read(NULL_CALL_BACK,I2C1,SI1133_RESPONSE0_ADDRESS);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Is the new CMD_CTR = to original CMD_CRT + 1?
  response0_value2 = (Si1133_return_data() & MASK_NEEDED);
  EFM_ASSERT((response0_value + 1) == response0_value2);

  //As long as ASSERT didn't fail, write 0b000001 to INPUT0 reg to prepare to set Channel 0 as
  //active si1133 channel
  Si1133_write(NULL_CALL_BACK, I2C1, SI1133_INPUT0_ADDRESS, SI1133_CHANNEL0_ADDRESS);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Specify Parameter Table Write command ORED with CHAN_LIST while writing to COMMAND reg.
  //Chan list found in the parameter table (page 28) as 0x01
  chan_list_command = PARAM_SET | SI1133_CHAN_LIST;
  Si1133_write(NULL_CALL_BACK, I2C1, SEND_CMD, chan_list_command);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Read RESPONSE0 Register to verify command reg write executed successfully
  Si1133_read(NULL_CALL_BACK,I2C1,SI1133_RESPONSE0_ADDRESS);

  //Check if the state machine is busy again
  while(check_busy(I2C1));

  //Is the new CMD_CTR = to original CMD_CRT + 2?
  response0_value3 = (Si1133_return_data() & MASK_NEEDED);
  EFM_ASSERT((response0_value + 2) == response0_value3);

  //As long as that assert passed, we exit function
}

/***************************************************************************//**
 * @brief
 *   Forces a write to the Si1133 to initiate a sensing operation
 * @details
 *  We send a write command to the Response0 register to initiate the single
 *  sensing operation
 *
 * @param[in] none
 *
 ******************************************************************************/
void Si1133_FORCE(){
  Si1133_write(SI1133_REG_READ_CB, I2C1, SEND_CMD, SI1133_RESPONSE0_ADDRESS);
}

/***************************************************************************//**
 * @brief
 *   Requests the result of the Si1133_FORCE() function which will give the sensing
 *   data from the singular operation
 * @details
 *   2 bytes will be read from the HOSTOUT0 register (0x13) for the result from
 *   the sensing operation
 *
 *
 * @param[in] none
 *
 ******************************************************************************/
void Si1133_RETURN(){
  //Using running state based on captured sensor data (page 16)
  //HostOut0 used 0x13
  Si1133_read(SI1133_REG_READ_CB, I2C1, SI1133_HOSTOUT0);
}
