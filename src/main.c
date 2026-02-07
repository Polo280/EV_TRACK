#include "main.h"
#include "driver/i2c.h"
#include "esp_random.h"


uint8_t current_status_code = STATUS_OFFLINE;

// static const char *TAG = "I2C_SCAN";

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


void status_LED_task(void *arg){
    bool led_state = false;

    while(1){
        if(!test_mode_enabled){
            led_state = !led_state;
            gpio_set_level(LED_PIN, led_state);

        }
        if(current_status_code == STATUS_ONLINE){
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}


static const char *TAG = "TELEM";

static void telemetry_print_task(void *arg)
{
    (void)arg;

    while (1)
    {
        /* If telemetry_data is written by other tasks/ISRs,
           you should protect it with a mutex.
           For now we just read it directly. */

        ESP_LOGI(TAG,
                 "\n"
                 "---------------- TELEMETRY ----------------\n"
                 "Battery voltage : %.2f V\n"
                 "Current         : %.2f A\n"
                 "Latitude        : %.7f\n"
                 "Longitude       : %.7f\n"
                 "Accel X         : %.3f m/s2\n"
                 "Accel Y         : %.3f m/s2\n"
                 "Accel Z         : %.3f m/s2\n"
                 "Orient X        : %.2f deg\n"
                 "Orient Y        : %.2f deg\n"
                 "Orient Z        : %.2f deg\n"
                 "RPM             : %u\n"
                 "Velocity X      : %.2f m/s\n"
                 "Velocity Y      : %.2f m/s\n"
                 "Ambient Temp    : %.2f C\n"
                 "Altitude        : %.2f m\n"
                 "Satellites      : %u\n"
                 "Air speed       : %.2f m/s\n"
                 "-------------------------------------------",
                 telemetry_data.battery_voltage,
                 telemetry_data.current_amps,
                 telemetry_data.latitude,
                 telemetry_data.longitude,
                 telemetry_data.accel_x,
                 telemetry_data.accel_y,
                 telemetry_data.accel_z,
                 telemetry_data.orient_x,
                 telemetry_data.orient_y,
                 telemetry_data.orient_z,
                 telemetry_data.rpms,
                 telemetry_data.velocity_x,
                 telemetry_data.velocity_y,
                 telemetry_data.ambient_temp,
                 telemetry_data.altitude_m,
                 telemetry_data.num_sats,
                 telemetry_data.air_speed);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void bno055_task(void *arg)
{
    TelemetryData *telem = (TelemetryData *)arg;

    double acc[3];
    double euler[3];
    int8_t temp;

    ESP_LOGI("BNO055", "Starting BNO055...");

    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_NDOF);
    if (err != ESP_OK)
    {
        ESP_LOGE("BNO055", "bno055_begin_i2c failed");
        vTaskDelete(NULL);
    }

    /* optional but recommended */
    set_external_crystal(true);

    while (1)
    {
        bool ok_acc  = false;
        bool ok_ori  = false;

        if (get_vector(VECTOR_LINEARACCEL, acc) == ESP_OK)
            ok_acc = true;

        if (get_vector(VECTOR_EULER, euler) == ESP_OK)
            ok_ori = true;

        temp = get_temp();

        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);

        if (ok_acc)
        {
            telem->accel_x = acc[0];   // m/s²
            telem->accel_y = acc[1];
            telem->accel_z = acc[2];
        }

        if (ok_ori)
        {
            telem->orient_x = euler[0];   // degrees
            telem->orient_y = euler[1];
            telem->orient_z = euler[2];
        }

        telem->ambient_temp = (float)temp;

        xSemaphoreGive(telemetry_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void)
{
    // Create mutex for telemetry data struct 
    telemetry_mutex = xSemaphoreCreateMutex();
    configASSERT(telemetry_mutex);
    i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(i2c_mutex);

    // Config and setup
    NVS_Init();
    
    xTaskCreate(Peripheral_Config, "Peripheral_Config", 4096, NULL, 15, NULL);

    // Periodic get data tasks
    xTaskCreate(SD_monitor_task, "SD_monitor", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(GPS_parse_task, "gps_parse", 4096, &telemetry_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(pitot_task, "pitot_task", 4096, &telemetry_data, 6, NULL, 1);
    xTaskCreate(foc_uart_test_task, "foc_uart_test_task", 4096, NULL, 5, NULL);
    // xTaskCreate(bno055_task, "bno055_task", 4096, &telemetry_data, 4, NULL);

     xTaskCreate(post_data, "post_data", 8192, &telemetry_data, 5, NULL);
     xTaskCreatePinnedToCore(can_tx_task, "can_tx_task", 4096, NULL, 5, NULL, 1);

    // Status LED
    xTaskCreate(status_LED_task, "status_LED", 2048, NULL, 15, NULL);

    // xTaskCreate(telemetry_print_task, "telemetry_print_task", 4096, NULL, 4, NULL);
}