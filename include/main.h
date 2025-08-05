#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Custom files 
#include "WIFI_Manager.h"
#include "PeriphConfig.h"
#include "Globals.h"
#include "MT3333_GPS.h"

// CONTROL MACROS
#define ADC_AVG_NUM_SAMPLES 8

// Telemetry Struct to store current Data
extern TelemetryData telemetry_data;

/////// PERIPHERAL CONFIG ////////
void uartInit();

//////// TASK DECLARATIONS ////////
void LED_status(void*);
void readOneShot_ADC(void*);