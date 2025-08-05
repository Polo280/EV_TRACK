#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"
#include "Globals.h"

#include "MT3333_GPS.h"

// RGB status LED
extern uint8_t RGB_LED_GPIOs[3];

// CONFIGURATION FUNCTIONS 
void GPIO_Config(void *);
void UART_Config(void *);