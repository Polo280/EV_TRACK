#include "main.h"
#include "driver/i2c.h"

uint8_t current_status_code = STATUS_OFFLINE;

// Default telemetry data on start
TelemetryData telemetry_data = {
    .battery_voltage = 0.0f,
    .current_amps    = 0.0f,
    .latitude        = 0.0f,
    .longitude       = 0.0f,
    .accel_x         = 0.0f,
    .accel_y         = 0.0f,
    .accel_z         = 0.0f,
    .orient_x        = 0.0f,
    .orient_y        = 0.0f,
    .orient_z        = 0.0f,
    .rpms            = 0,
    .velocity_x      = 0.0f,
    .velocity_y      = 0.0f,
    .ambient_temp    = 0.0f,
    .altitude_m      = 0.0f,
    .num_sats        = 0,
    .air_speed       = 0.0f
};

SemaphoreHandle_t telemetry_mutex;
SemaphoreHandle_t i2c_mutex;


void app_main(void)
{
    // Create mutex for telemetry data struct 
    telemetry_mutex = xSemaphoreCreateMutex();
    configASSERT(telemetry_mutex);
    i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(i2c_mutex);

    // Configuration 
    NVS_Init();
    xTaskCreate(Peripheral_Config, "Peripheral_Config", 4096, NULL, 15, NULL);

    // Communication 
    xTaskCreatePinnedToCore(foc_uart_test_task, "foc_uart_test_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(GPS_parse_task, "gps_parse", 4096, &telemetry_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(pitot_task, "pitot_task", 4096, &telemetry_data, 6, NULL, 1);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx_task", 4096, NULL, 5, NULL, 1);

    // Data acquisition 
    //  xTaskCreate(post_data, "post_data", 8192, &telemetry_data, 5, NULL);
    xTaskCreate(SD_manager_task, "SD_manager", 4096, &telemetry_data, 5, NULL);

    // Debugging
    xTaskCreate(status_LED_task, "status_LED", 2048, NULL, 15, NULL);
    xTaskCreate(telemetry_print_task, "telemetry_print_task", 4096, NULL, 4, NULL);
}