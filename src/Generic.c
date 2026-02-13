#include "Generic.h"

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


void telemetry_print_task(void *arg)
{
    (void)arg;

    while (1)
    {
        /* If telemetry_data is written by other tasks/ISRs,
           you should protect it with a mutex.
           For now we just read it directly. */

        ESP_LOGI("TELEM",
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
