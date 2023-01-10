/**
 * @file
 *  app.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  January 30th, 2022
 * @brief
 *  Getting Peripherals set up and initializing the structure of the letimer
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"


//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************


//***********************************************************************************
// Private functions
//***********************************************************************************

static void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route);

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Initializes app_letimer_pwm and gets the timer started
 *
 *
 * @details
 *  Calls the cmu_open() and gpio_open function to initialize them,
 *  and then gets the app_letimer_pwm initialized. LETIMER0 is then started
 *
 * @note
 *  The app_letimer_pwm_open is initialized with the period, active period, and
 *  route locations
 *
 ******************************************************************************/

void app_peripheral_setup(void){
  cmu_open();
  gpio_open();
  app_letimer_pwm_open(PWM_PER, PWM_ACT_PER, PWM_ROUTE_0, PWM_ROUTE_1);
  letimer_start(LETIMER0, true);  //This command will initiate the start of the LETIMER0
}

/***************************************************************************//**
 * @brief
 *  Opening the letimer which is essentially initializing it's values
 *
 * @details
 *  The letimer_pwm_struct is specifically of type APP_LETIMER_PWM
 *  which is a struct that has a set of variables that need to be
 *  assigned to
 *
 * @note
 *  The values of the routes, period, and active period are passed into the function
 *
 *
 * @param[in]
 *  period: The entire period of the LETIMER. Essentially how long between
 *          the blinks of the LEd
 *
 * @param[in]
 *  act_period: The active period is the amount of time the LED will be on for
 *
 * @param[in]
 *  out0_route: Pin location for OUT0
 *
 * @param[in]
 *  out1_route: Pin location for OUT1
 *
 ******************************************************************************/

void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route){
  // Initializing LETIMER0 for PWM operation by creating the
  // letimer_pwm_struct and initializing all of its elements
  // APP_LETIMER_PWM_TypeDef is defined in letimer.h
  APP_LETIMER_PWM_TypeDef   letimer_pwm_struct;

  letimer_pwm_struct.active_period = act_period;
  letimer_pwm_struct.debugRun = false;
  letimer_pwm_struct.enable = false;
  letimer_pwm_struct.out_pin_0_en =  true;
  letimer_pwm_struct.out_pin_1_en = true;
  letimer_pwm_struct.out_pin_route0 = out0_route;
  letimer_pwm_struct.out_pin_route1 = out1_route;
  letimer_pwm_struct.period = period;

  letimer_pwm_open(LETIMER0, &letimer_pwm_struct);
}


