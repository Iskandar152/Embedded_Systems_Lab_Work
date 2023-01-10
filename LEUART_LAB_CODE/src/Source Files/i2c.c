/**
 * @file
 *  i2c.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  February 11, 2022
 * @brief
 *  Setting up I2C peripheral functions and structures
 *
 */
#include "i2c.h"
#include "cmu.h"
#include "scheduler.h"
#include "app.h"
static I2C_STATE_MACHINE state_machine0;
static I2C_STATE_MACHINE state_machine1;
static I2C_STATE_MACHINE *local_state;

/***************************************************************************//**
 * @brief
 *   Starting up I2C module clock, initializing I2C based on our structs, enabling interrupts,
 *   doing proper routing, and enabling NVIC
 * @details
 *   The Si1133.c module open function calls our I2C function to allow our I2C peripheral to be
 *   properly set up to receive or transmit data. This involves enabling the interrupts through
 *   routelock0 and routepen. We also allow specifically the ACK, RXDATAV, and MSTOP
 *   interrupts to be raised. In my code however, there is some kind of strange issue which
 *   doesn't allow IRQ handlers to be called. Reasoning unknown for now
 *
 * @note
 *   Called once to initialize I2c
 *
 * @param[in] I2C_TypeDef *i2c,
 *   Pointer to our I2C peripheral, differentiates between I2C0 and I2C1
 * @param[in] I2C_OPEN_STRUCT *i2c_setup
 *   The struct that contains the data we need to initialize a local
 *   I2C struct with
 *
 ******************************************************************************/
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *i2c_setup){
  //Local I2C_Init_TypeDef defined
  I2C_Init_TypeDef local_I2C;

  //Enable the clock based on what I2C peripheral is entered
  if(i2c == I2C0){
      CMU_ClockEnable(cmuClock_I2C0, true);
  }
  else if(i2c == I2C1){
      CMU_ClockEnable(cmuClock_I2C1, true);
  }
  //Test to verify proper I2C clock operation
  EFM_ASSERT(cmuClock_I2C1);
  if((i2c->IF & 0x01) == 0){
      i2c->IFS = 0x01;
      EFM_ASSERT(i2c->IF & 0x01);
      i2c->IFC = 0x01;
  }
  else{
      i2c->IFC = 0x01;
      EFM_ASSERT(!(i2c->IF & 0x01));
  }



  //Not sure if I need to set up the values?
  local_I2C.enable = i2c_setup->enable;
  local_I2C.master = i2c_setup->master;
  local_I2C.refFreq = i2c_setup->refFreq;
  local_I2C.freq = i2c_setup->freq;
  local_I2C.clhr = i2c_setup->clhr;
  //Initializing the local I2C_TypeDef
  I2C_Init(i2c,&local_I2C);  //Since local_I2C is a struct, it's passed into init using its address

  //Routing based on the i2c
  if(i2c == I2C0){
      i2c->ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC11 | I2C_ROUTELOC0_SCLLOC_LOC10;
  }
  else if(i2c == I2C1){
      //i2c->ROUTELOC0 = i2c_setup-> out_pin_scl | i2c_setup->out_pin_sda;
      i2c->ROUTELOC0 |= i2c_setup-> out_pin_sda | i2c_setup-> out_pin_scl;
  }
  i2c->ROUTEPEN = ( I2C_ROUTEPEN_SCLPEN * i2c_setup->out_pin_scl_en);
  i2c->ROUTEPEN |= (I2C_ROUTEPEN_SDAPEN * i2c_setup->out_pin_sda_en);
  i2c_bus_reset(i2c);

  i2c->IFC = I2C_IFC_MSTOP | I2C_IFC_ACK;
  i2c->IEN = I2C_IF_ACK | I2C_IF_RXDATAV | I2C_IF_MSTOP;

  //Need to enable interrupts
  if(i2c==I2C0){
      NVIC_EnableIRQ(I2C0_IRQn);
  }
  else if(i2c==I2C1){
      NVIC_EnableIRQ(I2C1_IRQn);
  }


}
/***************************************************************************//**
 * @brief
 *   Clears interrupt flags and makes sure peripherals are ready for new interrupts
 * @details
 *   The i2c_bus_reset function saves the state of our IEN register, disables
 *   interrupts using the IEN register (sets it equal to 0). Clears the interrupt flag
 *   by writing to it, clears the transmit buffer, and performs the simultaneous start and stop
 *   bits until an MSTOP is triggered. Afterwards, the saved IEN state is put back
 *
 * @note
 *   Called once in I2C_open to reset interrupts
 *
 * @param[in] I2C_TypeDef *i2c,
 *   Specifies which i2c (I2C0 or I2C1) is going to be affected
 *
 ******************************************************************************/
void i2c_bus_reset(I2C_TypeDef *i2c){

  //Send an abort command to the I2C peripheral
  i2c->CMD = I2C_CMD_ABORT;

  //Save state of the I2C->IEN register
  //The I2C->IEN register is of type uint32_t
  uint32_t i2c_state = i2c->IEN;

  //Disabling all interrupts using the I2C->IEN register
  i2c->IEN = 0;

  //Clear Interrupt Flag, will do so by writing to it
  i2c->IFC = i2c->IF;

  //Clear I2C transmit buffer
  i2c->CMD = I2C_CMD_CLEARTX;

  //Perform the simultaneous setting of the START and STOP bits
  i2c->CMD = I2C_CMD_START | I2C_CMD_STOP;
  //Stall until the STOP has been completed by checking the MSTOP bit
  while(!(i2c->IF & I2C_IF_MSTOP));

  //Clear all interrupts that may be generated
  i2c->IFC = i2c->IF;

  //Reset micro-controller I2C peripheral
  i2c->CMD = I2C_CMD_ABORT;

  //Put the state back of the interrupt enable register???
  i2c->IEN = i2c_state;
}

/***************************************************************************//**
 * @brief
 *   I2C0_IRQHandler is supposed to be called when an interrupt relating to the I2C peripheral is
 *   raised. Based on the interrupt, separate functions are called
 * @details
 *   If the ACK interrupt is raised on I2C0, then the i2c_ack function is called, with the proper state machine
 *   If RXDATAV is raised, that means there is data in the RX buffer, and the rx_data function is called
 *   If MSTOP is raised, then we call the ending() function to stop the process
 *
 * @param[in] none
 *
 ******************************************************************************/
void I2C0_IRQHandler(void){
  uint32_t int_flag;
  local_state = &state_machine0;
  //Interrupts include the ACK, NACK, RXDATA, and IC0 flag

  //Set flag to I2C0 flag and Interrupt enable
  int_flag = I2C0 -> IF & I2C0 -> IEN;
  I2C0->IFC = int_flag;
  if(int_flag & I2C_IF_ACK){
      i2c_ack(local_state);
  }
  if(int_flag & I2C_IF_RXDATAV){
      i2c_rx_data(I2C0);
  }
  if(int_flag & I2C_IF_MSTOP){
      ending();
  }
}
/***************************************************************************//**
 * @brief
 *   I2C1_IRQHandler is supposed to be called when an interrupt relating to the I2C peripheral is
 *   raised. Based on the interrupt, separate functions are called
 * @details
 *   If the ACK interrupt is raised on I2C1, then the i2c_ack function is called, with the proper state machine
 *   If RXDATAV is raised, that means there is data in the RX buffer, and the rx_data function is called
 *   If MSTOP is raised, then we call the ending() function to stop the process
 *
 * @param[in] none
 *
 ******************************************************************************/
void I2C1_IRQHandler(void){
  uint32_t int_flag;
  local_state = &state_machine1;
  //Interrupts include the ACK, RXDATA, and IC1 flag

  //Set flag to I2C1 flag and Interrupt enable
  int_flag = I2C1->IF & I2C1->IEN;
  I2C1->IFC = int_flag;
  if(int_flag & I2C_IF_ACK){
      i2c_ack(local_state);
  }
  if(int_flag & I2C_IEN_RXDATAV){
      i2c_rx_data(I2C1);
  }
  if(int_flag & I2C_IEN_MSTOP){
      ending(local_state);
  }
}
/***************************************************************************//**
 * @brief
 *   i2c_start begins our state machine at intitialization, and goes through the process
 *   of reading/writing from/to the Si1133 peripheral. It relies on the IRQ handler to proceed from there
 * @details
 *   Initializes the local state machine that goes along with the I2C perpheral
 *   being affected. Sends a start command and a shifted write message to
 *   begin the sending of the device address and register address to read from
 *
 * @param[in] I2C_TypeDef *i2c,
 *   Specifies which I2C we are going to use
 * @param[in] uint32_t *result_address,
 *    Specifies where to store read values from the RX buffer
 * @param[in] uint32_t rw,
 *    Specifies whether the function is going to initiate a read or write
 * @param[in] uint32_t register_address,
 *    Address to read from
 * @param[in] uint32_t callback_value,
 *    Callback value for our timing
 ******************************************************************************/
void i2c_start(I2C_TypeDef *i2c,uint32_t *result_address, uint32_t rw, uint32_t register_address,uint32_t callback_value, uint32_t data_to_write){

  if(i2c == I2C0){
      local_state = &state_machine0;
  }
  else if(i2c == I2C1){
      local_state = &state_machine1;

  }
  else{
      EFM_ASSERT(false);
  }

  while(local_state->busy);
  EFM_ASSERT((local_state->current_state & _I2C_STATE_STATE_MASK) == I2C_STATE_STATE_IDLE);
  sleep_block_mode(I2C_EM_BLOCK);
  local_state->busy = true;

  if(rw == (SHIFTED_ADDRESS_WRITE)){
      local_state->writing_data = data_to_write;
  }

  if(register_address == SI1133_HOSTOUT0){
      local_state->bytes_to_read = BYTE_GIVEN + 1;
  }
  else{
      local_state->bytes_to_read = BYTE_GIVEN;
  }
  local_state-> i2c_used = i2c;
  local_state->device_address = SI1133_ADDRESS;
  local_state->register_address = register_address;
  local_state->pointer_to_result = result_address;
  local_state->rw_type = rw;
  local_state->callback = callback_value;
  local_state->bytes_to_write = BYTE_GIVEN;

  //Start transferring the address of sensor based whether we are reading or writing
  if(data_to_write){
      local_state->current_state = sending_address;
      local_state->i2c_used-> CMD = I2C_CMD_START;
      local_state->i2c_used->TXDATA = SHIFTED_ADDRESS_WRITE;
  }
  else{
      local_state->current_state = sending_address;
      local_state->i2c_used->CMD = I2C_CMD_START;
      local_state->i2c_used->TXDATA = SHIFTED_ADDRESS_WRITE;
  }
}
/***************************************************************************//**
 * @brief
 *   i2c_ack is called when an ACK interrupt is raised. Based on the location we are
 *   in within the state machine, we have different responses
 * @details
 *   At initialization, ack shouldn't be raised
 *
 *   When it's at the state when it should be sending an address, it sends the
 *   register address to read from and advances the state
 *
 *   In the re-sending data state it sends another start command, and the address
 *   to read from
 *
 *   All other states should not be entered for ACK
 *
 *
 * @param[in] I2C_STATE_MACHINE *local_state,
 *   Local state machine
 *
 ******************************************************************************/
void i2c_ack(I2C_STATE_MACHINE *local_state){
  switch(local_state->current_state){
    case initialize:
      EFM_ASSERT(false);
      break;
    case sending_address:
      if(local_state->writing_data){
          local_state->i2c_used->TXDATA = local_state->register_address;
          local_state->current_state = writing;
      }
      else if(!local_state->writing_data){
          local_state->i2c_used->TXDATA = local_state->register_address;
          local_state->current_state = resending_data;
      }

      break;

    case resending_data:
        local_state->i2c_used->CMD = I2C_CMD_START; //double start
        local_state->i2c_used->TXDATA = SHIFTED_ADDRESS_READ;
        local_state->current_state = reading;
      break;
    case reading:
      break;
    case writing:
      if(local_state->bytes_to_write > 0){
          local_state-> bytes_to_write -= 1;
          local_state->i2c_used->TXDATA = local_state->writing_data >> (8 * local_state->bytes_to_write);

      }
      else if(local_state->bytes_to_write == ZERO_VALUE){
          local_state->current_state = end;
          local_state->i2c_used->CMD = I2C_CMD_STOP;
      }
      break;
    case end:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
      break;

  }
}
/***************************************************************************//**
 * @brief
 *   i2c_rx_data is called when the rx buffer has data in it that the
 *   device needs to fetch.
 * @details
 *   The only state that requires fetching is during the reading state, where
 *   we store the value to a result in the state machines local variable, and then
 *   shift it so we are ready to accept more data if needed
 *
 * @param[in] I2C_TypeDef *i2c,
 *   Specifies which i2c (I2C0 or I2C1) is going to be affected
 *
 ******************************************************************************/
void i2c_rx_data(I2C_TypeDef *i2c){
  switch(local_state->current_state){
    case initialize:
      EFM_ASSERT(false);
      break;
    case sending_address:
      EFM_ASSERT(false);
      break;
    case resending_data:
      EFM_ASSERT(false);
      break;
    case reading:
      if(local_state->bytes_to_read > ZERO_VALUE){

          local_state->bytes_to_read -= 1;
          *(local_state->pointer_to_result) &= ~(0xff <<8 *local_state->bytes_to_read);

          *local_state->pointer_to_result |= (i2c->RXDATA) << (8 * local_state->bytes_to_read);
          if(local_state->bytes_to_read){
              i2c->CMD = I2C_CMD_ACK;
          }

          if(local_state->bytes_to_read == ZERO_VALUE){
              i2c->CMD = I2C_CMD_NACK;
              i2c->CMD = I2C_CMD_STOP;
              local_state->current_state = end;
          }
      }
      break;
    case writing:
      EFM_ASSERT(false);
      break;
    case end:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
      break;
  }
}
/***************************************************************************//**
 * @brief
 *   If we recieve MSTOP, we eventually will call ending() to finish the reading process
 * @details
 *   The busy bit of the state machine is set to not busy, the sleep block is unblocked
 *   and we add the call back to the scheduled events essentially telling our
 *   device that we are done
 *
 * @note
 *   The writing->data is set to 0 because we base our comparison of whether
 *   the start i2c function is doing a write or read on it.
 *
 * @param[in] none,
 *
 ******************************************************************************/
void ending(){
  if(local_state->current_state == end){
      local_state->busy = false;
      local_state->writing_data = ZERO_VALUE;
      sleep_unblock_mode(I2C_EM_BLOCK);
      add_scheduled_event(SI1133_REG_READ_CB);
  }
  else{
      EFM_ASSERT(false);
  }
}

/***************************************************************************//**
 * @brief
 *   Checks whether the current i2c peripheral is busy with a different function
 * @details
 *   A simple if else statement is used to check if our peripheral is I2C1 or I2C0,
 *   and based on that, we check the busy bit in the state machine
 *
 *
 * @param[in] I2C_TypeDef *i2c
 *   The peripheral whose state machine will be checked.
 *
 ******************************************************************************/
bool check_busy(I2C_TypeDef *i2c){

  if(i2c == I2C0){
      return state_machine0.busy;
  }

  else if(i2c == I2C1){
      return state_machine1.busy;
  }

  else{
      EFM_ASSERT(false);
      return false;
  }


}
