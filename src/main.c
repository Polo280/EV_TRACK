#include "main.h"

uint8_t current_status_code = STATUS_OFFLINE;  // Default status at start 
// Default telemetry data on start
TelemetryData telemetry_data = {
    .battery_voltage = 24.0,
    .current_amps =    2.5,
    .latitude =        0.0,
    .longitude =       0.0,
    .accel_x =         0.0,
    .accel_y =         34.0,
    .accel_z =         90.0,
    .orient_x =        23.0,
    .orient_y =        0.0,
    .orient_z =        0.0, 
    .rpms =            0,
    .velocity_x =      0.0,
    .velocity_y =      0.0,
    .ambient_temp =    0.0
};


// MAIN PROGRAM ENTRY POINT
void app_main() {
    esp_err_t ret = init_sdcard();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "SD init failed: %s", esp_err_to_name(ret));
        return; // Don't try to write if mount failed
    }


    sd_append_line("/sdcard/test.txt", "Holaa");
    // NVS which stores some WIFI configurations 
    NVS_Init();
    // Initialize GPIO's
    // xTaskCreate(&GPIO_Config, "GPIO_Config", 2048, NULL, 5, NULL);
    // Connect to WIFI 
    // xTaskCreate(&WIFI_Connect, "WIFI_Connect", 4096, NULL, 2, NULL);
    // Posting data to database
    // xTaskCreate(&post_data, "post_data", 4096, (void *)&telemetry_data, 2, NULL);
    // ADC Oneshot task
    //xTaskCreate(&readOneShot_ADC, "readOneShot", 4096, NULL, 3, NULL);
    // UART Test|
    // xTaskCreate(&UART_Config, "UART_Config", 4096, NULL, 3, NULL);
    // I2C Test
    // xTaskCreate(&I2C_Config, "I2C_Config", 4096, NULL, 3, NULL);
    // Blink task 
    // xTaskCreate(&LED_status, "LED_status", 2048, (void *)&current_status_code, 1, NULL);
}
  