/**
 * @file
 *  spi.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  April 24, 2022
 * @brief
 *  Setting up device for SPI communication and the state machine
 *
 */

#include "spi.h"

//local state machine
static SPI_STATE_MACHINE *local_state_machine;
static SPI_STATE_MACHINE state_1;
static uint32_t junk_store = 2;

/***************************************************************************//**
 * @brief
 *   The spi_open() function starts up the SPI module clock, initializes
 *   the local structures, enables certain interrupts and registers
 * @details
 *   The spi_open function enables the cmuClock_USART3 for SPI communication,
 *   enables structure variables such as the frequency, databits, and so on.
 *   We also complete routing here for the chip select, RX and TX, and CSLK.
 *   Finally we enable the necessary interrupts
 *
 * @note
 *   Called once to initialize SPI
 *
 * @param[in] USART_TypeDef *usart,
 *   Specific USART being utilized for SPI communication
 * @param[in] SPI_OPEN_STRUCT *usart_setup
 *   Struct containing data on how to initialize the USART SPI communication. All
 *   info should have been passed by the devices open function
 *
 ******************************************************************************/
void spi_open(USART_TypeDef *usart, SPI_OPEN_STRUCT *usart_setup){
  USART_InitSync_TypeDef local_spi;

  //Enable the spi clock based on the USART connection used
  if(usart == USART3){
      CMU_ClockEnable(cmuClock_USART3,true);
  }
  else{
      EFM_ASSERT(false);
  }


  EFM_ASSERT(cmuClock_USART3); //Using 3 for transmit and receive operations
  local_spi.enable = usart_setup -> enable;
  local_spi.master = usart_setup -> master;
  local_spi.databits = usart_setup -> databits;
  local_spi.autoCsEnable = usart_setup -> autoCsEnable;
  local_spi.autoCsHold = usart_setup -> autoCsHold;
  local_spi.autoCsSetup = usart_setup -> autoCsSetup;
  local_spi.autoTx =  usart_setup -> autoTx;
  local_spi.baudrate = usart_setup -> baudrate;
  local_spi.clockMode = usart_setup -> clockMode;
  local_spi.msbf = usart_setup -> msbf;
  local_spi.prsRxCh = usart_setup -> prsRxCh;
  local_spi.prsRxEnable = usart_setup -> prsRxEnable;
  local_spi.refFreq = usart_setup -> refFreq;
  USART_InitSync(usart, &local_spi);

  usart->ROUTELOC0 = SPI_TX_ROUTE | SPI_RX_ROUTE | SPI_CS_ROUTE | SPI_CLK_ROUTE ;
  usart->ROUTEPEN = (USART_ROUTEPEN_CLKPEN * usart_setup -> out_pin_clk);
  usart->ROUTEPEN |= (USART_ROUTEPEN_CSPEN * usart_setup -> out_pin_cs);
  usart->ROUTEPEN |= (USART_ROUTEPEN_RXPEN * usart_setup -> in_pin_rx);
  usart->ROUTEPEN |= (USART_ROUTEPEN_TXPEN * usart_setup -> out_pin_tx);

  usart -> CMD |= (USART_CMD_CLEARRX | USART_CMD_CLEARTX);

  GPIO_PinOutSet(ICM20648_CS_PORT, ICM20648_CS_PIN);
  USART_Enable(usart, usart_setup->enable);

  NVIC_EnableIRQ(USART3_RX_IRQn);
  NVIC_EnableIRQ(USART3_TX_IRQn);

}

/***************************************************************************//**
 * @brief
 *   Receiving data interrupt handler
 *
 * @details
 *   The RX_IRQHandler is called whenever the RXDATA interrupt is raised
 *   due to data being in the RXDATA register
 *
 *
 *
 * @param[in]none
 ******************************************************************************/
void USART3_RX_IRQHandler(){
  uint32_t int_flag;

  int_flag = USART3 -> IF & USART3 -> IEN;
  USART3->IFC = int_flag;

  if(int_flag & USART_IF_RXDATAV){
      spi_rxdatav(USART3);
  }
  else{

  }
}

/***************************************************************************//**
 * @brief
 *   Transmitting data interrupt handler
 *
 * @details
 *   The TX_IRQHandler is called whenever the TXBL interrupt is raised
 *   due to the transmit buffer being empty
 *
 *
 *
 * @param[in]none
 ******************************************************************************/
void USART3_TX_IRQHandler(){
  uint32_t int_flag;

  int_flag = USART3 -> IF & USART3 -> IEN;
  USART3->IFC = int_flag;

  if(int_flag &USART_IF_TXBL){
      spi_state_machine();
  }

  if(int_flag & USART_IF_TXC){
      usart_end(USART3);
  }

}

/***************************************************************************//**
 * @brief
 *   State machine for SPI communication, not currently complete.
 * @param[in] USART_TypeDef *usart
 *    The USART device we are using for sending data
 * @param[in] uint32_t *result_address
 *  The address we want to write our result to (this is specifically for writing)
 * @param[in] uint32_t register_address
 *  The register address of the register we are reading or writing
 * @param[in] uint32_t data_to_write
 *  Specifically for writing, the data we want to send to the register
 * @param[in] uint32_t callback_value
 *  callback used
 * @param[in] uint32_t read_write
 *  read_write determines whether we are reading or writing, 0 is write, 1 is read
 *
 ******************************************************************************/
void spi_start(USART_TypeDef *usart, uint32_t *result_address, uint32_t register_address, uint32_t data_to_write,
               uint32_t callback_value, uint32_t read_write){

  if(usart!=USART3){
      EFM_ASSERT(false);
  }
  local_state_machine = &state_1;

  while(local_state_machine->busy_bit);

  //Atomicity Commands
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  //Fill local state machine based on the spi command (read or write)
  local_state_machine->busy_bit = true;
  local_state_machine->bytes_to_read = 1;
  local_state_machine->bytes_to_write = sizeof(data_to_write);
  local_state_machine->data_string = data_to_write;
  local_state_machine->device_address = DEVICE_ADDRESS;
  local_state_machine->register_address = register_address;
  local_state_machine->rw_bit = read_write;
  local_state_machine->callback = callback_value;
  local_state_machine->pointer_to_result = result_address;
  local_state_machine->usart_value = USART3;
  local_state_machine->state = initialize_spi;
  //Block The Sleep Mode
  sleep_block_mode(SPI_EM_BLOCK);

  //Enable Interrupts
  usart->IEN |= USART_IEN_TXBL;
  usart->IEN |= USART_IEN_RXDATAV;
  usart->IFC |= USART_IEN_RXDATAV; //note
  usart->IFC |= USART_IEN_TXBL; //note

  //Clear The Chip Select again
  GPIO_PinOutClear(ICM20648_CS_PORT, ICM20648_CS_PIN);
  CORE_EXIT_CRITICAL();

}

/***************************************************************************//**
 * @brief
 *   spi_state_machine for handling reading and writing
 * @details
 *   This state machine handles the processes for TXBL interrupts being raised. The
 *   state machine sends and recieves data (currently not working). It also handles junk data being
 *   entered after every transfmit
 *
 * @param[in]none
 ******************************************************************************/
void spi_state_machine(){
  switch(local_state_machine->state){
    case(initialize_spi):
        local_state_machine->usart_value->TXDATA = local_state_machine->register_address | (local_state_machine->rw_bit << 7);
        local_state_machine->state = handle_junk;
        break;
    case(handle_junk):
        //EFM_ASSERT(false);
        break;
    case(receive_send_data):
        if(local_state_machine->rw_bit == 0){
            local_state_machine->usart_value->TXDATA |= local_state_machine->data_string;
            local_state_machine->state = send_junk;
        }
        else if(local_state_machine->rw_bit == 1){
            local_state_machine->usart_value->TXDATA = local_state_machine->data_string;  //sending junk data for read so that I receieve a rx back
            local_state_machine->state = receive;
        }
        break;
    case(receive):
        //EFM_ASSERT(false);
        break;
    case(send_junk):
        //EFM_ASSERT(false);
        break;
    case(finish):
        local_state_machine->usart_value->IEN &= ~USART_IEN_TXBL;
        local_state_machine->usart_value->IEN &= ~USART_IEN_RXDATAV;
        local_state_machine->usart_value->IEN |= USART_IEN_TXC;
        break;
  }
}
/***************************************************************************//**
 * @brief
 *   The spi_rxdatav state machine handles processing for when RXDATAV is raised
 * @details
 *   This state machine handles the processes for RXDATAV interrupt being raised.
 *   It reads in the junk data passed after a transmit if needed, and then for
 *   actual data is records it into a integer variable outside of the SPI function.
 *   Currently doesn't work for reading..
 *
 * @param[in]none
 ******************************************************************************/
void spi_rxdatav(USART_TypeDef *usart){
  switch(local_state_machine->state){
    case(initialize_spi):
        break;
    case(handle_junk):
        junk_store = (local_state_machine->usart_value->RXDATA);
        local_state_machine->state = receive_send_data;
        break;
    case(receive_send_data):
        //EFM_ASSERT(false);
        break;
    case(receive):
        if(local_state_machine->bytes_to_read > NIL_VAL){
            local_state_machine->bytes_to_read--;

            *(local_state_machine->pointer_to_result) &= ~(0xff << 8 *local_state_machine->bytes_to_read);

            *local_state_machine->pointer_to_result |= (usart->RXDATA) << (8 * local_state_machine->bytes_to_read);

            if(local_state_machine->bytes_to_read){

            }

            if(local_state_machine->bytes_to_read == NIL_VAL){
                local_state_machine->state = finish;
            }
        }



        break;
    case(send_junk):
        junk_store = (local_state_machine->usart_value->RXDATA);
        local_state_machine->state = finish;
        break;
    case(finish):
        //EFM_ASSERT(false);
        break;
    default:
        EFM_ASSERT(false);
  }



}

/***************************************************************************//**
 * @brief
 *   USART END handles the TXC interrupt and closes down SPI since the write or read is complete
 * @details
 *   The currently blocked sleep mode is unblocked, we disable our TXC interrupt. CS is configured,
 *   the event is scheduled, and the busy bit is set to false to allow more reads and writes using spi
 *
 * @param[in]none
 ******************************************************************************/
void usart_end(USART_TypeDef *usart){
  //Unblock Sleep
  sleep_unblock_mode(SPI_EM_BLOCK);
  //Disable interrupts for TXC
  usart->IEN &= ~(USART_IEN_TXC);
  //Pinout
  GPIO_PinOutSet(ICM20648_CS_PORT, ICM20648_CS_PIN);
  //Add the scheduled event
  add_scheduled_event(USART_DONE_CB);
  //Not busy anymore
  local_state_machine->busy_bit = false;

}
