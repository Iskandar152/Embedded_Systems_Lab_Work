/**
 * @file
 *  gpio.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  January 30, 2022
 * @brief
 *  Initializing GPIO pins
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Enabling GPIO clocks and setting the GPIO pins
 *
 * @details
 *  We enable the GPIO Clock, set the drive strength of the port and then
 *  The GPIO pins for the LED's are set
 *
 *
 * @note
 *
 *
 ******************************************************************************/

void gpio_open(void){

  CMU_ClockEnable(cmuClock_GPIO, true);

	// Configure LED pins
	GPIO_DriveStrengthSet(LED_RED_PORT, LED_RED_DRIVE_STRENGTH);
	GPIO_PinModeSet(LED_RED_PORT, LED_RED_PIN, LED_RED_GPIOMODE, LED_RED_DEFAULT);

	GPIO_DriveStrengthSet(LED_RED_PORT, LED_GREEN_DRIVE_STRENGTH);
	GPIO_PinModeSet(LED_RED_PORT, LED_GREEN_PIN, LED_GREEN_GPIOMODE, LED_GREEN_DEFAULT);

}
