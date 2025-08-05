#include "PeriphConfig.h"

uint8_t RGB_LED_GPIOs[3] = {RED_LED_GPIO, BLUE_LED_GPIO, GREEN_LED_GPIO};

bool WIFI_transmit_enable = true;

/**
 * @brief Interrupt handler for enable transmission push button 
 * @return None
 */
static void IRAM_ATTR gpio_isr_handler(void* arg){
    WIFI_transmit_enable = !WIFI_transmit_enable;
    if(WIFI_transmit_enable){
        current_status_code = STATUS_TRANSMITTING;
    }else{
        current_status_code = STATUS_ONLINE;
    }
}


/**
 * @brief Initializes on-start platform GPIO's 
 * @return None
 */
void GPIO_Config(void* pvParameter){
    // RGB LED GPIOs
    for(uint8_t i=0; i < sizeof(RGB_LED_GPIOs); i++){
        esp_rom_gpio_pad_select_gpio(RGB_LED_GPIOs[i]);
        gpio_set_direction(RGB_LED_GPIOs[i], GPIO_MODE_OUTPUT);
    }

    // Transmit enable push button, configure interrupt with pullup
    esp_rom_gpio_pad_select_gpio(TRANSMIT_ENABLE_GPIO);
    gpio_set_direction(TRANSMIT_ENABLE_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TRANSMIT_ENABLE_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(TRANSMIT_ENABLE_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TRANSMIT_ENABLE_GPIO, gpio_isr_handler, NULL);
    gpio_intr_enable(TRANSMIT_ENABLE_GPIO);

    // Avoid returning 
    vTaskDelete(NULL);
}


/**
 * @brief Configures the required UART channels
 * @note  UART0 - Console Debug / UART2 - GPS module (MT3333)
 * @return None
 */
void UART_Config(void* pvParameter){
    // Config params and buffer 
    const int buff_size = 2048;
    QueueHandle_t uart_queue;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, buff_size, buff_size, 10, &uart_queue, 0));
    // Configure UART
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_CHANNEL, &uart_config));
    // Configure GPIOs for UART 
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPS_TX_GPIO, GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // char* test_str = "$PMTK605*31\r\n";
    // uart_write_bytes(GPS_UART_CHANNEL, (const char*)test_str, strlen(test_str));

    GPS_Init(GPS_UART_CHANNEL);
    // char* test_str = "$PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
    // uart_write_bytes(GPS_UART_CHANNEL, (const char*)test_str, strlen(test_str));

    while(true){
        //printf("UART Message sent\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}