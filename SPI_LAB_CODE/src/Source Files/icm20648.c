/**
 * @file
 *  icm20648.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  April 24, 2022
 * @brief
 *  Preparing device to set up the icm20648 peripheral. The Test Driven Development test function
 *  is inside icm_configure
 *
 */


#include "icm20648.h"

uint32_t result_data;
/***************************************************************************//**
 * @brief
 *   The icm_open() function initializes the icm20648 peripheral and prepares it for SPI communication
 * @details
 *   Local structures containing information regarding spi communication are initialized, an spi initializer
 *   function spi_open is called, and then our test driven development function is called, icm_configure();
 *
 * @note
 *   Called once to initialize icm
 *
 * @param[in] none
 ******************************************************************************/
void icm_open(){
  SPI_OPEN_STRUCT local_open_struct;
  timer_delay(1);
  local_open_struct.autoCsEnable = true;
  local_open_struct.autoTx = false;
  local_open_struct.autoCsSetup = false;
  local_open_struct.baudrate = 1000000;
  local_open_struct.clockMode = usartClockMode3;
  local_open_struct.databits = usartDatabits8;
  local_open_struct.enable = usartEnable;
  local_open_struct.in_pin_rx = true;
  local_open_struct.master = true;
  local_open_struct.msbf = true;
  local_open_struct.out_pin_clk = true;
  local_open_struct.out_pin_tx = true;
  local_open_struct.out_pin_cs = true;
  local_open_struct.prsRxEnable = false;
  local_open_struct.refFreq = 0;
  //local_open_struct.autoCsHold = 0;
  //local_open_struct.autoCsSetup = 0;
  //local_open_struct.csInv = false;
  //local_open_struct.prsRxCh = 0;
  //block em2
  spi_open(USART3,&local_open_struct);
  icm_configure();
}
/***************************************************************************//**
 * @brief
 *   The icm_read() function reads from a specific register from the icm20648 device peripheral
 * @details
 *   icm_read() calls spi_start to begin spi communication in a reading format
 *
 * @param[in] uint32_t callback,
 *   Callback to send to the spi_start function
 * @param[in] USART_TypeDef *usart
 *   Specific usart communication peripheral used. For icm20648 this will always be USART3
 * @param[in] uint32_t reg_address
 *  The register within the peripheral we are reading from
 *
 ******************************************************************************/
void icm_read(uint32_t callback, USART_TypeDef *usart, uint32_t reg_address){
  spi_start(usart,&result_data,reg_address, 0, callback, 1);
}
/***************************************************************************//**
 * @brief
 *   The icm_write() function writes to a specific register within the icm20648 peripheral
 * @details
 *   icm_write() calls spi_start to begin spi communication in a writing format
 *
 * @param[in] uint32_t callback,
 *   Callback to send to the spi_start function
 * @param[in] USART_TypeDef *usart
 *   Specific usart communication peripheral used. For icm20648 this will always be USART3
 * @param[in] uint32_t reg_address
 *  The register within the peripheral we are writing to
 * @param[in] uint32_t data_to_write
 *  Value to write to the register
 ******************************************************************************/
void icm_write(uint32_t callback, USART_TypeDef *usart, uint32_t reg_address, uint32_t data_to_write){
  spi_start(usart,&result_data,reg_address, data_to_write, callback, 0);
}
/***************************************************************************//**
 * @brief
 *   the icm_configure() function sets up the different registers in the icm20648 for proper
 *   utilization of the peripheral. This is my TDD function. Currently does not pass since
 *   reading and writing fails
 *
 * @param[in] none
 *
 ******************************************************************************/
void icm_configure(){
  //WRITE AND READ OPERATIONS AREN'T CURRENTLY COMPLETE, BUT TDD IS HERE
  //Delay after every write equaling 22 MICROSECONDS required after every write

  /*-------------------Enable Low Power, Disable Temperature Sensor in PWR MGMT 1------------------*/
  icm_write(CALLBACK_NULL_VALUE, USART3, PWR_MGMT_1,PWR_1_LP_TEMPDIS);
  timer_delay(22);
  icm_read(CALLBACK_NULL_VALUE, USART3, PWR_MGMT_1);
  if(!(PWR_1_LP_TEMPDIS & result_data)){
      EFM_ASSERT(false);
  }


  /*-------------------Enable Accelerometer and Disable Gyroscope in PWR MGMT 2 -------------------*/
  icm_write(CALLBACK_NULL_VALUE, USART3, PWR_MGMT_2, PWR_2_ACCELEN_DISGYRO);
  timer_delay(22);
  icm_read(CALLBACK_NULL_VALUE, USART3, PWR_MGMT_2);
  if(!(result_data && PWR_2_ACCELEN_DISGYRO)){
      EFM_ASSERT(false);
  }
  /*-------------------Enable Accelerometer to operate in duty cycle mode in the LP CONFIG---------*/
  icm_write(CALLBACK_NULL_VALUE, USART3, LP_CONFIG,LP_DUTY);
  timer_delay(22);
  icm_read(CALLBACK_NULL_VALUE, USART3, LP_CONFIG);
  if(!(result_data && LP_DUTY)){
        EFM_ASSERT(false);
    }
  /*-------------------Set Accelerometer Threshold to 240 mg in ACCEL WOM THR register-------------*/
  icm_write(CALLBACK_NULL_VALUE, USART3, ACCEL_WOM_THR, ACCELERATION_THRESHOLD);
  timer_delay(22);
  icm_read(CALLBACK_NULL_VALUE, USART3, ACCEL_WOM_THR);
  if(!(result_data && ACCEL_WOM_THR)){
        EFM_ASSERT(false);
    }
}
