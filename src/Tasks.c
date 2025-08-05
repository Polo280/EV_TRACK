#include "main.h"

void LED_status(void* pvParameter) {
    bool blink_aux = true;

    while (true) {
        uint8_t *status = (uint8_t *)pvParameter;

        switch (*status)
        {
        case STATUS_ONLINE:
            // GREEN 
            gpio_set_level(GREEN_LED_GPIO, 1);
            gpio_set_level(BLUE_LED_GPIO, 0);
            gpio_set_level(RED_LED_GPIO, 0);
            break;
        
        case STATUS_OFFLINE:
            // YELLOW 
            gpio_set_level(GREEN_LED_GPIO, 1);
            gpio_set_level(BLUE_LED_GPIO, 1);
            gpio_set_level(RED_LED_GPIO, 0);
            break;

        case STATUS_TRANSMITTING:
            // GREEN BLINK 
            gpio_set_level(GREEN_LED_GPIO, blink_aux);
            gpio_set_level(BLUE_LED_GPIO, 0);
            gpio_set_level(RED_LED_GPIO, 0);
            blink_aux = !blink_aux;
            break;

        case STATUS_ERROR:
            // RED
            gpio_set_level(GREEN_LED_GPIO, 0);
            gpio_set_level(BLUE_LED_GPIO, 0);
            gpio_set_level(RED_LED_GPIO, 1);
            break;

        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void readOneShot_ADC(void* pvParameter){
    // Configure ADC1
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc1_config_struct = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config_struct, &adc1_handle));

    // Currently using GPIO 32 (ADC1 CHANN4) for battery voltage measurements 
    adc_oneshot_chan_cfg_t adc_chann_config_struct = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc_chann_config_struct));

    while(true){
        // Perform an average of N oneshot samples 
        int adc_raw = 0, adc_total = 0;
        for(uint8_t i=0; i < ADC_AVG_NUM_SAMPLES; i++){
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw));    
            adc_total += adc_raw;        
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        // ADC Average reading, map as required 
        adc_total = adc_total / ADC_AVG_NUM_SAMPLES;
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
