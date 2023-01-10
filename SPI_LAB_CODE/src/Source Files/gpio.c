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


	// Set RGB LED gpio pin configurations
	GPIO_PinModeSet(RGB_ENABLE_PORT, RGB_ENABLE_PIN, gpioModePushPull,RGB_DEFAULT_OFF);
	GPIO_PinModeSet(RGB0_PORT, RGB0_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	GPIO_PinModeSet(RGB1_PORT, RGB1_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	GPIO_PinModeSet(RGB2_PORT, RGB2_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	GPIO_PinModeSet(RGB3_PORT, RGB3_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	GPIO_PinModeSet(RGB_RED_PORT, RGB_RED_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);
	GPIO_PinModeSet(RGB_GREEN_PORT, RGB_GREEN_PIN, gpioModePushPull,COLOR_DEFAULT_OFF);
	GPIO_PinModeSet(RGB_BLUE_PORT, RGB_BLUE_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);

	//Set I2C GPIO Pins
	GPIO_DriveStrengthSet(SI1133_SENSOR_EN_PORT,SI1133_DRIVE_STRENGTH);
	GPIO_PinModeSet(SI1133_SENSOR_EN_PORT, SI1133_SENSOR_EN_PIN, SI1133_GPIOMODE,1);
	GPIO_PinModeSet(SI1133_SCL_PORT, SI1133_SCL_PIN, gpioModeWiredAnd, SI1133_SCL_DEFAULT);
	GPIO_PinModeSet(SI1133_SDA_PORT, SI1133_SDA_PIN, gpioModeWiredAnd, SI1133_SDA_DEFAULT);

	//Set UART Pins for Bluetooth
	GPIO_DriveStrengthSet(UART_TX_PORT, UART_TX_DRIVE_STRENGTH);
	GPIO_PinModeSet(UART_TX_PORT, UART_TX_PIN, gpioModePushPull, UART_TX_DEFAULT);
	GPIO_PinModeSet(UART_RX_PORT, UART_RX_PIN, gpioModeInput, UART_RX_DEFAULT);

	//Set SPI PINS FOR ICM20648
	//GPIO_DriveStrengthSet(ICM20648_MOSI_PORT,SPI_TX_DRIVE_STRENGTH);
	//GPIO_DriveStrengthSet(ICM20648_MISO_PORT,SPI_RX_DRIVE_STRENGTH);
	GPIO_DriveStrengthSet(ICM20648_SENSOR_EN_PORT,gpioDriveStrengthWeakAlternateWeak);
	//Need to finish drive strength of SCLK
	//Need to finish drive strength of EN
	//Need to finish drive strength of Chip Select
	GPIO_PinModeSet(ICM20648_SENSOR_EN_PORT, ICM20648_SENSOR_EN_PIN, gpioModePushPull, 1);
	//GPIO_PinModeSet(ICM20648_INT_PORT, ICM20648_INT_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(ICM20648_CS_PORT, ICM20648_CS_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(ICM20648_MOSI_PORT,ICM20648_MOSI_PIN,gpioModePushPull, 0);
	GPIO_PinModeSet(ICM20648_MISO_PORT,ICM20648_MISO_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(ICM20648_SCLK_PORT,ICM20648_SCLK_PIN, gpioModePushPull, 0);
}

