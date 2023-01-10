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
void Si1133_read(uint32_t callback_to_pass, I2C_TypeDef *i2c){
  i2c_start(i2c,&data, 0, SI1133_PARTID_ADDRESS,callback_to_pass);
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
