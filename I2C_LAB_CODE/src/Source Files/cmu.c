/**
 * @file
 *  cmu.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  January 30, 2022
 * @brief
 *  Enabling/Disabling the clocks and oscillators
 *
 */
//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************


//***********************************************************************************
// Private functions
//***********************************************************************************


//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * Enabling CMU clocks and Oscillators that will be needed for the low power application
 * of the LETIMER.
 *
 * @details
 *  THE HFPER clock is enabled, CORELE clock is enabled , the LFRCO Oscillator is disabled, the LFXO Oscillator
 *  is disabled
 *
 *  We also route the LF clock to the LF clock tree. ULFRCO is routed to
 *  The low frequency clock tree
 *
 *
 * @note
 *
 *
 ******************************************************************************/

void cmu_open(void){

    CMU_ClockEnable(cmuClock_HFPER, true);

    // By default, LFRCO is enabled, disable the LFRCO oscillator
    // Disable the LFRCO oscillator
    // What is the enumeration required for LFRCO?
    // It can be found in the online HAL documentation
    CMU_OscillatorEnable(cmuOsc_LFRCO , false, false);

    // Disable the LFXO oscillator
    // What is the enumeration required for LFXO?
    // It can be found in the online HAL documentation
    CMU_OscillatorEnable(cmuOsc_LFXO , false, false);

    // No requirement to enable the ULFRCO oscillator.  It is always enabled in EM0-4H1

    // Route LF clock to the LF clock tree
    // What is the enumeration required to placed the ULFRCO onto the proper clock branch?
    // It can be found in the online HAL documentation
    CMU_ClockSelectSet(cmuClock_LFA , cmuSelect_ULFRCO);    // routing ULFRCO to proper Low Freq clock tree

    // What is the proper enumeration to enable the clock tree onto the LE clock branches?
    // It can be found in the Assignment 2 documentation
    CMU_ClockEnable(cmuClock_CORELE, true);

}

