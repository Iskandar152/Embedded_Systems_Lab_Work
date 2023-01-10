#ifndef BRD_CONFIG_HG
#define BRD_CONFIG_HG

//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
#include "em_cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED 0 pin is
#define LED_RED_PORT          gpioPortD
#define LED_RED_PIN           8
#define LED_RED_DEFAULT       false   // Default false (0) = off, true (1) = on
#define LED_RED_GPIOMODE      gpioModePushPull

// LED 1 pin is
#define LED_GREEN_PORT        gpioPortD
#define LED_GREEN_PIN         9
#define LED_GREEN_DEFAULT     false // Default false (0) = off, true (1) = on
#define LED_GREEN_GPIOMODE    gpioModePushPull

#define MCU_HFXO_FREQ			    cmuHFRCOFreq_32M0Hz

// GPIO pin setup
//#define STRONG_DRIVE

#ifdef STRONG_DRIVE
	#define LED_RED_DRIVE_STRENGTH		gpioDriveStrengthStrongAlternateStrong
  #define LED_GREEN_DRIVE_STRENGTH    gpioDriveStrengthStrongAlternateStrong
#else
	#define LED_RED_DRIVE_STRENGTH		gpioDriveStrengthWeakAlternateWeak
  #define LED_GREEN_DRIVE_STRENGTH    gpioDriveStrengthWeakAlternateWeak
#endif

// LETIMER PWM Configuration

#define   PWM_ROUTE_0     LETIMER_ROUTELOC0_OUT0LOC_LOC17
#define   PWM_ROUTE_1     LETIMER_ROUTELOC0_OUT1LOC_LOC16


//***********************************************************************************
// function prototypes
//***********************************************************************************

#endif
