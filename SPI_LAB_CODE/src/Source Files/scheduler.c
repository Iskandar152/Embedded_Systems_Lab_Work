/**
 * @file
 *  scheduler.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  February 13, 2022
 * @brief
 *  Defining and initializing functions dealing with the scheduler and scheduling events
 */
#include "scheduler.h"
#include "em_assert.h"
#include "em_core.h"
#include "em_emu.h"
static uint32_t event_scheduled;

/***************************************************************************//**
 * @brief
 * Initializes the event scheduled private static variable
 * @details
 * Atomically sets the event_scheduled variable to 0
 * @note
 * @param[in] None
 *
 ******************************************************************************/
void scheduler_open(void){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  event_scheduled = 0;
  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 * An event is added to the scheduler
 * @details
 * Atomically adds an event to the event_scheduled private static scheduler
 * @note
 * @param[in] uint32_t event
 * This is the event that we are adding to the scheduler
 *
 *
 *
 ******************************************************************************/
void add_scheduled_event(uint32_t event){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled |= event;
  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 * Remove an event from scheduler
 * @details
 * Atomically removes an event from the scheduler
 * @note
 *
 * @param[in] uint32_t event
 * The event that needs to be removed from the scheduler
 *
 *
 ******************************************************************************/
void remove_scheduled_event(uint32_t event){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled &= ~(event);
  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 * Shows scheduled events
 * @details
 * The event_scheduled private variable is returned
 * @note
 * @param[in] None
 * @return
 * The scheduler with all the events is returned as a uint32_t
 *
 *
 *
 ******************************************************************************/
uint32_t get_scheduled_events(void){
  return event_scheduled;
}

