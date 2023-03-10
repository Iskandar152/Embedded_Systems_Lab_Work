//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef APP_HG
#define APP_HG

/* System include statements */


/* Silicon Labs include statements */
#include "em_cmu.h"
#include "em_assert.h"

/* The developer's include statements */
#include "cmu.h"
#include "gpio.h"
#include "letimer.h"
#include "brd_config.h"
#include "LEDs_thunderboard.h"
#include "scheduler.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define   PWM_PER         3.0   // PWM period in seconds
#define   PWM_ACT_PER     0.001  // PWM active period in seconds
#define   LETIMER0_COMP0_CB    0x00000001 //0b0001
#define   LETIMER0_COMP1_CB    0x00000002 //0b0010
#define   LETIMER0_UF_CB       0x00000004 //0b0100



//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void app_peripheral_setup(void);
void scheduled_letimer0_uf_cb(void);
void scheduled_letimer0_comp0_cb(void);
void scheduled_letimer0_comp1_cb(void);
void initialize_led(void);
#endif
