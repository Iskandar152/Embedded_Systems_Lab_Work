/**
 * @file
 *  app.c
 * @author
 *  Iskandar Shoyusupov
 * @date
 *  February 13th, 2022
 * @brief
 *  Getting Peripherals set up and initializing the structure of the letimer
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"
#include "LEDs_thunderboard.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************
static int led_color;

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
  scheduler_open();
  sleep_open();
  initialize_led();
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
  letimer_pwm_struct.comp0_irq_enable = false;
  letimer_pwm_struct.comp0_cb = LETIMER0_COMP0_CB; //?
  letimer_pwm_struct.comp1_irq_enable = true;
  letimer_pwm_struct.comp1_cb = LETIMER0_COMP1_CB; //?
  letimer_pwm_struct.uf_irq_enable = true;
  letimer_pwm_struct.uf_cb = LETIMER0_UF_CB;       //?

  letimer_pwm_open(LETIMER0, &letimer_pwm_struct);
}
/***************************************************************************//**
 * @brief
 * Based on the UF callback, this function goes through and disables the current blinking
 * color on the LED.
 * @details
 * Based on the value in led_color which keeps track of the current color of the LED, we
 * determine which color to turn off, and then we  increment to the next LED
 * @note The led_color is set back to 0 when we reach led_color having a value of 2,
 * so we can have a loop from Red->Green->Blue
 * @param[in] None
 * @return None
 *
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void){
  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_UF_CB));
  if(led_color == 0){
      leds_enabled(RGB_LED_1,COLOR_RED,false);
      led_color++;
  }
  else if(led_color == 1){
      leds_enabled(RGB_LED_1,COLOR_GREEN,false);
      led_color++;
  }
  else if(led_color == 2){
      leds_enabled(RGB_LED_1,COLOR_BLUE,false);
      led_color = 0;
  }
}
/***************************************************************************//**
 * @brief
 * Initializing static variables and rgb LEDs
 * @details
 * The led_color private static variable is set to 0, and the rgb_init()
 * function is called to set the GPIO pins for the LED
 * @note
 * @param[in] None
 * @return None
 ******************************************************************************/
void initialize_led(void){
  led_color = 0;
  rgb_init();
}
/***************************************************************************//**
 * @brief
 * Based on the comp1 callback, this function is called in main, and it turns on specific
 * RGB colors in our LED
 * @details Based on the led_color private static variable, we determine which color to turn on
 * in the LED. This will later be turned off in the uf callback
 * @note
 * @param[in] None
 * @return None
 *
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void){
  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));
  if(led_color == 0){
      leds_enabled(RGB_LED_1,COLOR_RED,true);
  }
  else if(led_color == 1){
      leds_enabled(RGB_LED_1,COLOR_GREEN,true);
  }
  else if(led_color == 2){
      leds_enabled(RGB_LED_1,COLOR_BLUE,true);
  }
}

