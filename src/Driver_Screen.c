#include "Driver_Screen.h"

uint8_t payload_base = 1;

static bool recovery_started = false;

/* ---------------- CAN INIT ---------------- */
void can_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG_CAN, "TWAI driver started");
}

/* ---------------- CAN TX TASK ---------------- */
void can_tx_task(void *arg)
{
    twai_message_t tx_msg = {
        .identifier = CAN_TEST_ID,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE
    };

    can_init();

    TickType_t last_wake = xTaskGetTickCount();
    twai_status_info_t status;

    while (1) {
        twai_get_status_info(&status);        

        if (status.state == TWAI_STATE_RUNNING) {
            recovery_started = false;

            for (int i = 0; i < 8; i++) {
                tx_msg.data[i] = payload_base + i;
            }

            esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));

            if (err == ESP_OK) {
                payload_base++;
            } else {
                ESP_LOGE(TAG_CAN, "TX failed (%s)", esp_err_to_name(err));
            }
        }
        else if (status.state == TWAI_STATE_BUS_OFF) {
            twai_stop();
            twai_driver_uninstall();

            twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

            twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
            twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
            ESP_ERROR_CHECK(twai_start());

            ESP_LOGI(TAG_CAN, "TWAI driver recover start");
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(3000));
        }
        else if (status.state == TWAI_STATE_RECOVERING) {

            // absolutely do nothing
        }

        // Precise 1 second period
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
}