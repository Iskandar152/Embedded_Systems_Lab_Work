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
#include "ble.h"
#include "HW_Delay.h"
#include "icm20648.h"
#include <stdio.h>
#include <string.h>
//***********************************************************************************
// defined files
//***********************************************************************************
#define   PWM_PER         2.0   // PWM period in seconds
#define   PWM_ACT_PER     0.002  // PWM active period in seconds
#define   LETIMER0_COMP0_CB    0x00000001 //0b0001
#define   LETIMER0_COMP1_CB    0x00000002 //0b0010
#define   LETIMER0_UF_CB       0x00000004 //0b0100
#define   SI1133_REG_READ_CB   0x00000008 //0b1000
#define   BOOT_UP_CB           0x00000010 //0b10000
#define   TX_EVENT_CB          0x00000020 //0b100000
#define   RX_EVENT_CB          0x00000040 //0b1000000
#define   BLE_TX_DONE_CB       0x00000080 //0b10000000
#define   USART_DONE_CB        0x00000100 //0b100000000
#define   READVALUETOCOMPARETO    0x33
#define   WHITE_SENSOR_COMPARISON 20
#define   SYSTEM_BLOCK_EM      EM3
//#define BLE_TEST_ENABLED


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
void si1133_test(void);
void uart_handler(void);

void scheduled_boot_up_cb(void);
#endif
