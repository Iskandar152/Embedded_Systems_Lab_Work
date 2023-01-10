/*****************************************************
 * @file sleep_routines.c
 *****************************************************
 *@section License
 *<b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *****************************************************
 *
 *Permission is granted to anyone to use this software for any purpose,
 *including commercial applications, and to alter it and redistribute it
 *freely, subject to the following restrictions
 *
 *1. The origin of this software must not be misrepresented; you must not
 *   claim that you wrote the original software
 *2. Altered source versions must be plainly marked as such, and must not be
 *   misrepresented as being the original software
 *3. This notice may not be removed or altered from any source distribution
 *
 *DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 *obligation to support this Software. Silicon Labs is providing the
 *Software "AS IS", with no express or implied warranties of any kind,
 *including,but not limited to, any implied warranties of merchantability
 *or fitness for any particular purpose or warranties against infringement
 *of any proprietary rights of a third party
 *
 *Silicon Labs will not be liable for any consequential, incidental, or
 *special damages, or any other relief, or for any claim by any third party,
 *arising from your use of this Software
 *****************************************************/


#include "sleep_routines.h"

static int lowest_energy_mode[MAX_ENERGY_MODES];
/***************************************************************************//**
 * @brief
 * Initializing the array that stores the current value of energy modes
 * @details
 * Using a for loop, the lowest_energy_mode array has each of its members
 * set to 0
 * @note
 * @param[in] None
 *
 ******************************************************************************/
void sleep_open(){
  for(int i = 0; i < 5; i++){
      lowest_energy_mode[i] = 0;
  }
}
/***************************************************************************//**
 * @brief
 * A specific energy mode is blocked so we don't reach that state in our peripheral
 * @details
 * Atomically increments the energy mode EM in the lowest_energy_mode array
 * @note
 * @param[in] EM
 * EM is the energy mode that will be incremented to be selected as blocked
 *
 ******************************************************************************/
void sleep_block_mode(uint32_t EM){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  EFM_ASSERT(lowest_energy_mode[EM] < 5);
  lowest_energy_mode[EM] += 1;

  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 * A specific energy mode is unblocked so we can reach that state again in our peripheral
 * @details
 * Atomically decrements the energy mode EM in the lowest_energy_mode array
 * @note
 * @param[in] EM
 * EM is the energy mode that will be decremented to be selected as unblocked
 *
 ******************************************************************************/
void sleep_unblock_mode(uint32_t EM){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  EFM_ASSERT(lowest_energy_mode[EM] >= 0);
  lowest_energy_mode[EM] -= 1;

  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 * The sleep mode is determined by the lowest energy mode that is enabled
 * @details
 * For energy modes 0 and 1, we don't enter any functional sleep mode
 * For energy mode 2, we enter EM1
 * For energy mode 3, we enter EM2
 * @note
 * @param[in] None
 *
 ******************************************************************************/
void enter_sleep(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  if(lowest_energy_mode[0] > 0){
      CORE_EXIT_CRITICAL();
      return;
  }
  else if(lowest_energy_mode[1] > 0){
      CORE_EXIT_CRITICAL();
      return;
  }
  else if(lowest_energy_mode[2] > 0){
      EMU_EnterEM1();
      CORE_EXIT_CRITICAL();
      return;
  }
  else if(lowest_energy_mode[3] > 0){
      EMU_EnterEM2(true);
      CORE_EXIT_CRITICAL();
      return;
  }
  else{

      EMU_EnterEM3(true);
      CORE_EXIT_CRITICAL();
      return;
  }
}
/***************************************************************************//**
 * @brief
 * The current lowest energy mode that is blocked is returned
 * @details
 * Iterating through the lowest energy mode array until the first blocked mode is reached
 * @note
 * @param[in] None
 * @return i
 * signifies the lowest energy mode that was blocked. Returns a uint32_t value
 *
 ******************************************************************************/
uint32_t current_block_energy_mode(){
    for(int i = 0; i < 5; i++){
        if(lowest_energy_mode[i] > 0){
            return i;
        }
    }
}
