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
#include "Si1133.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************
static int led_color;
static uint32_t x = 3;
static uint32_t y = 0;
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
  Si1133_open();

  sleep_block_mode(SYSTEM_BLOCK_EM);
  ble_open(TX_EVENT_CB, RX_EVENT_CB);
  add_scheduled_event(BOOT_UP_CB); //Scheduling UART event
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
 * We do a return command to read the sensing data that our force command gave.
 * We also algorithmically manipulate specific numbers to a solution that gets
 * transmitted to the leuart device using our ble_write function
 * @param[in] None
 * @return None
 *
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void){
  //floats are seemingly incredibly problematic for some reason in printf commands on
  //my simplicity studio, have to manually do the division
  Si1133_RETURN();
  float z;
  x += 3;
  y += 1;
  z = (float)x/y;
  char str[10];
  sprintf(str,"z=%3.1f\n",z);
  ble_write(str);

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
 * Sends a Si1133_FORCE() command to read data from our white sensor
 * @details The force command will ask for a single sensing value
 * @note
 * @param[in] None
 * @return None
 *
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void){
  //EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));
  Si1133_FORCE();
  /*
  if(led_color == 0){
      leds_enabled(RGB_LED_1,COLOR_RED,true);
  }
  else if(led_color == 1){
      leds_enabled(RGB_LED_1,COLOR_GREEN,true);
  }
  else if(led_color == 2){
      leds_enabled(RGB_LED_1,COLOR_BLUE,true);
  }
  */
}
/***************************************************************************//**
 * @brief
 *   The si1133_test function test function checks the value recieved form the white sensor
 *   and if the value is below a certain amount, we turn on the blue led, otherwise it will be off
 * @details
 *   The values from the sensor is a direct consequence of calling the FORCE and RETURN si1133 functions
 *   in our callback functions. These request and retrieve the data from the sensor, which
 *   we then can compare to here.
 *
 * @param[in] none
 *
 ******************************************************************************/
void si1133_test(){
  EFM_ASSERT((get_scheduled_events() & SI1133_REG_READ_CB));
  int data_value = (int)(short)Si1133_return_data();
  char str[50];
  if(data_value < WHITE_SENSOR_COMPARISON){
      leds_enabled(RGB_LED_1, COLOR_BLUE, true);
      sprintf(str,"It's Dark = %d\n",data_value);
      ble_write(str);
  }
  else{
      leds_enabled(RGB_LED_1, COLOR_BLUE, false);
      sprintf(str,"It's Light Outside = %d\n",data_value);
      ble_write(str);
  }


}

/***************************************************************************//**
 * @brief
 *   If the bluetooth device is being used for the first time, we set up
 *   the device name to "Donut" based on the BLE_TEST_ENABLED call back .
 * @details
 *   We also print hello work to test out the ble_write functin and start up the
 *   letimer0
 *
 * @param[in] none
 *
 ******************************************************************************/
void scheduled_boot_up_cb(void){
#ifdef BLE_TEST_ENABLED
  bool ble_return = ble_test("Donut");
  EFM_ASSERT(ble_return);
  timer_delay(2000);
#endif
  letimer_start(LETIMER0, true);  //This command will initiate the start of the LETIMER0
  ble_write("\nHello World\n");
}

/***************************************************************************//**
 * @brief
 *   not used for now
 *
 ******************************************************************************/
void uart_handler(void){

}
