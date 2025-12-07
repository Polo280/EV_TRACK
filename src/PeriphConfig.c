#include "PeriphConfig.h"
#include "BNO055.h"

// Pinout check - https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/

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
 * @brief Configures required UART channels
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
    // GPS TESTING
    GPS_Init(GPS_UART_CHANNEL);

    while(true){
        // Do something constantly
        //printf("UART Message sent\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}



/**
 * @brief Configures I2C Buses
 * 
 * @note Only one I2C interface (from 2 available) being used in this board.
 * GPIO21 - SDA 
 * GPIO22 - SCL
 * 
 * @return None
 */
void I2C_Config(void* pvParameter){
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_INSTANCE, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_INSTANCE, i2c_config.mode, 0, 0, 0));
    // BNO055 Test
    // bno055_set_i2c_handler(I2C_INSTANCE);
    // bno055_init(&default_bno055_config, &default_bno055_verification);

    uint8_t device_address = 0x28;
    bno055_euler_t gyro_euler;

    uint8_t addr = 0x28;
    uint8_t reg = 0x07;
    uint8_t data = 0x42;

    printf("I2C_Config task started!\n");

    printf("Starting I2C write loop...\n");

    while (true) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        printf("Command link created\n");

        // i2c_master_start(cmd);
        // printf("Start sent\n");

        // esp_err_t err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        // printf("Address write ret: %s\n", esp_err_to_name(err));

        // err = i2c_master_write_byte(cmd, reg, true);
        // printf("Reg write ret: %s\n", esp_err_to_name(err));

        // err = i2c_master_write_byte(cmd, data, true);
        // printf("Data write ret: %s\n", esp_err_to_name(err));

        // i2c_master_stop(cmd);
        // printf("Stop sent\n");

        // esp_err_t ret = i2c_master_cmd_begin(I2C_INSTANCE, cmd, pdMS_TO_TICKS(100));
        // printf("Command result: %s\n", esp_err_to_name(ret));

        // i2c_cmd_link_delete(cmd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}