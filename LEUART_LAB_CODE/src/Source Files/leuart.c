/**
 * @file leuart.c
 * @author Iskandar Shoyusupov
 * @date   April 5, 2022
 * @brief Contains all the functions of the LEUART peripheral
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************

//** Standard Library includes
#include <string.h>

//** Silicon Labs include files
#include "em_gpio.h"
#include "em_cmu.h"

//** Developer/user include files
#include "leuart.h"
#include "scheduler.h"
//Including this to use HM10 macros
#include "brd_config.h"
#include "app.h"
//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// private variables
//***********************************************************************************
uint32_t	rx_done_evt;
uint32_t	tx_done_evt;
bool		leuart0_tx_busy;
LEUART_STATE_MACHINE local_state;
/***************************************************************************//**
 * @brief LEUART driver
 * @details
 *  This module contains all the functions to support the driver's state
 *  machine to transmit a string of data across the LEUART bus.  There are
 *  additional functions to support the Test Driven Development test that
 *  is used to validate the basic set up of the LEUART peripheral.  The
 *  TDD test for this class assumes that the LEUART is connected to the HM-18
 *  BLE module.  These TDD support functions could be used for any TDD test
 *  to validate the correct setup of the LEUART.
 *
 ******************************************************************************/

//***********************************************************************************
// Private functions
//***********************************************************************************



//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *   LEUART_OPEN is an initialization function that sets up the necessary
 *   clocks, the local leuart struct, and properly routes the uart peripheral
 *   device
 *
 * @details
 *   Interrupts for the leuart device are enabled, the LEUART clock is enabled,
 *   the local leuart struct is initialized, and we EFM check whether the
 *   routing of the peripheral is done correctly
 *
 * @param[in] *leuart
 *   The leuart peripheral being used
 *
 * @param[in] *leuart_settings
 *   The struct being initialized
 *
 * @return
 *   none
 *
 ******************************************************************************/
void leuart_open(LEUART_TypeDef *leuart, LEUART_OPEN_STRUCT *leuart_settings){
  //Enabling the clock for the leuart peripheral
  CMU_ClockEnable(cmuClock_LEUART0,true);

  //Testing to verify that the clock is enabled
  EFM_ASSERT(cmuClock_LEUART0);

  //Going off the I2C example, we will write to the STARFRAME register a 0x1
  //and then check the SYNBUSY register on whether the STARTFRAME register is within
  //the lower frequency
  //????

  /*
  if((leuart->STARTFRAME & 0x01) == 0){

  }
  */
  LEUART_Init_TypeDef local_uart;

  local_uart.baudrate = leuart_settings->baudrate;
  local_uart.databits = leuart_settings->databits;
  local_uart.enable =   leuart_settings->enable;
  local_uart.parity =   leuart_settings->parity;
  local_uart.refFreq =  HM10_REFREQ;
  local_uart.stopbits = HM10_STOPBITS;
  LEUART_Init(leuart,&local_uart);

  //Routing ROUTELOC0 register for GPIO connection
  LEUART0->ROUTELOC0 = leuart_settings->rx_loc | leuart_settings->tx_loc;
  //Routpen ROUTEPEN
  LEUART0->ROUTEPEN = (leuart_settings->rx_en * LEUART_ROUTEPEN_RXPEN) |
                      (leuart_settings->tx_en * LEUART_ROUTEPEN_TXPEN);

  while(leuart->SYNCBUSY);  //location where?
  //CLEARING TX and RX
  //in I2C a command would be sent to the CMD register to clear out the RX and TX
  //in this case the macro LEUART_CMD_CLEARTX and LEUART_CMD_CLEARRX will be useful in clearing
  leuart->CMD |= (LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX);

  //Enabling the LEUART0, input argument is the leuart and an enable var
  LEUART_Enable(leuart,leuart_settings->enable);

  //Verifing that TX and RX are enabled
  EFM_ASSERT(leuart->STATUS & LEUART_STATUS_RXENS);  //Checking RXENS
  EFM_ASSERT(leuart->STATUS & LEUART_STATUS_TXENS);  //Checking TXENS

  //Enabling the IRQ for TX
  NVIC_EnableIRQ(LEUART0_IRQn);
}

/***************************************************************************//**
 * @brief
 *   leuart_txbl() is my state machine handler that does different functions
 *   based on the current state of transmission to the transmit buffer
 *
 * @details
 *   Initiate state simply routs to transmission
 *   Transmission send a singular character or if the string is fully transmitted
 *   it finishes up the transmission
 *
 * @param[in] none
 *
 * @return
 *   none
 *
 ******************************************************************************/
void leuart_txbl(){
  switch(local_state.state){
    case Initiate:
      local_state.state = Transmitting;
      break;
    case Transmitting:
      if(local_state.character_counter == (local_state.length_of_string - 1)){
          local_state.peripheral->TXDATA = local_state.string_to_pass[local_state.character_counter];
          //&~(specific bits) to clear them
          local_state.peripheral->IEN &= ~LEUART_IEN_TXBL;   //Enable the TXC interrupt while disabling the TXBL
          local_state.peripheral->IEN |= LEUART_IEN_TXC;
          local_state.state = Finish;
      }
      else{
          local_state.peripheral->TXDATA = local_state.string_to_pass[local_state.character_counter];
          local_state.character_counter += 1;
          local_state.state = Checking;
      }
      break;
    case Checking:
         local_state.state = Initiate;
      break;
    case Finish:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
  }
}

/***************************************************************************//**
 * @brief
 *   leuart_finish handles the different things that need to be disabled
 *   and the events scheduled based on transmission being finished
 *
 * @details
 *   The done callback for transmission is scheduled, the busy bit is reset,
 *   the peripherals TXC transmit is turned off, and sleep mode is unblocked
 *
 * @param[in] none
 *
 * @return
 *   none
 *
 ******************************************************************************/
void leuart_finish(){
  add_scheduled_event(BLE_TX_DONE_CB);
  local_state.busy_bit = 0;
  local_state.peripheral->IEN &= ~LEUART_IEN_TXC;
  sleep_unblock_mode(LEUART_TX_EM);
}
/***************************************************************************//**
 * @brief
 *   Based on a local interrupt flag, we determine if we call the leuart_txbl()
 *   or finish up transmission
 * @details
 *   The LEUART0_IRQHandler gets called whenever we get a condition where
 *   the transmit buffer is empty. The IRQ hanler either transfers to the
 *   leuart_txbl() state machine function, or calls the function that does
 *   the necessary tasks to finish up transmission
 *
 * @param[in] none
 *
 * @return
 *   none
 *
 ******************************************************************************/

void LEUART0_IRQHandler(void){
  //Just like the i2c, will have a local interrupt flag that
  //will hold the value of the interrupt flags for LEUART, and the
  //enable interrupts register values
  uint32_t int_flag;
  int_flag = local_state.peripheral->IF & local_state.peripheral->IEN;
  local_state.peripheral->IFC = int_flag;

  //Check whether the interrupt raised is the TXBL or the TXC
  if(int_flag & LEUART_IF_TXBL){
      leuart_txbl();
  }
  if(int_flag & LEUART_IF_TXC){
      if(local_state.state == Finish){
          leuart_finish();
          /*
          add_scheduled_event(BLE_TX_DONE_CB);
          local_state.busy_bit = 0;
          local_state.peripheral->IEN &= ~LEUART_IEN_TXC;
          sleep_unblock_mode(LEUART_TX_EM);
           */
          //clear IEN and TXC
          //add the scheduled event
      }
      else{
          EFM_ASSERT(false);
      }
  }
}

/***************************************************************************//**
 * @brief
 *   leuart_start() does all the initialization tasks for the local state
 *   machhine and begins transmission
 * @details
 *   Busy bit is set to busy, the string details such as the length and value
 *   is transmitted, other variables are initialized. The TXBL interrupt is
 *   enabled. Sleep mode is blocked for EM3. It is atomic
 *
 * @param[in] leuart
 *    The specific peripheral that we are transmitting to
 * @param[in] string
 *    String being transmitted
 * @param[in] string_len
 *    Length of the string being transmitted
 * @return
 *   none
 *
 ******************************************************************************/

void leuart_start(LEUART_TypeDef *leuart, char *string, uint32_t string_len){
  //Has to be atomic..
  while(local_state.busy_bit);
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  local_state.busy_bit = true;
  local_state.character_counter = 0;
  local_state.length_of_string = string_len;
  local_state.peripheral = leuart;
  local_state.state = Initiate;
  strcpy(local_state.string_to_pass,string);
  //Sleep Block
  sleep_block_mode(LEUART_TX_EM);

  //Enabling the TXBL register
  leuart->IEN |= LEUART_IEN_TXBL;

  //Start transmission
  local_state.state = Transmitting;
  CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *   LEUART STATUS function returns the STATUS of the peripheral for the
 *   TDD test
 *
 * @details
 * 	 This function enables the LEUART STATUS register to be provided to
 * 	 a function outside this .c module.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the STATUS register value as an uint32_t value
 *
 ******************************************************************************/

uint32_t leuart_status(LEUART_TypeDef *leuart){
	uint32_t	status_reg;
	status_reg = leuart->STATUS;
	return status_reg;
}

/***************************************************************************//**
 * @brief
 *   LEUART CMD Write sends a command to the CMD register
 *
 * @details
 * 	 This function is used by the TDD test function to program the LEUART
 * 	 for the TDD tests.
 *
 * @note
 *   Before exiting this function to update  the CMD register, it must
 *   perform a SYNCBUSY while loop to ensure that the CMD has by synchronized
 *   to the lower frequency LEUART domain.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] cmd_update
 * 	 The value to write into the CMD register
 *
 ******************************************************************************/

void leuart_cmd_write(LEUART_TypeDef *leuart, uint32_t cmd_update){

	leuart->CMD = cmd_update;
	while(leuart->SYNCBUSY);
}

/***************************************************************************//**
 * @brief
 *   LEUART IF Reset resets all interrupt flag bits that can be cleared
 *   through the Interrupt Flag Clear register
 *
 * @details
 * 	 This function is used by the TDD test program to clear interrupts before
 * 	 the TDD tests and to reset the LEUART interrupts before the TDD
 * 	 exits
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 ******************************************************************************/

void leuart_if_reset(LEUART_TypeDef *leuart){
	leuart->IFC = 0xffffffff;
}

/***************************************************************************//**
 * @brief
 *   LEUART App Transmit Byte transmits a byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a transmit byte, a while statement checking for the TXBL
 *   bit in the Interrupt Flag register is required before writing the
 *   TXDATA register.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] data_out
 *   Byte to be transmitted by the LEUART peripheral
 *
 ******************************************************************************/

void leuart_app_transmit_byte(LEUART_TypeDef *leuart, uint8_t data_out){
	while (!(leuart->IF & LEUART_IF_TXBL));
	leuart->TXDATA = data_out;
}


/***************************************************************************//**
 * @brief
 *   LEUART App Receive Byte polls a receive byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a receive byte, a while statement checking for the RXDATAV
 *   bit in the Interrupt Flag register is required before reading the
 *   RXDATA register.
 *
 * @param[in] leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the byte read from the LEUART peripheral
 *
 ******************************************************************************/

uint8_t leuart_app_receive_byte(LEUART_TypeDef *leuart){
	uint8_t leuart_data;
	while (!(leuart->IF & LEUART_IF_RXDATAV));
	leuart_data = leuart->RXDATA;
	return leuart_data;
}


