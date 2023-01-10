/*
 * ble.h
 *
 *  Created on:
 *      Author:
 */
//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef	BLE_GUARD_H
#define	BLE_GUARD_H

//** Standard Libraries
#include <stdbool.h>
#include <stdint.h>

// Driver functions
#include "leuart.h"
#include "gpio.h"
#include "brd_config.h"
#include <stdio.h>


//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void ble_open(); //uint32_t tx_event, uint32_t rx_event removed due to warnings and not being used
void ble_write(char *string);
bool ble_test(char *mod_name);

#endif
