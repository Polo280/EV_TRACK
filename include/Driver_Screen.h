#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

#define TAG_CAN "CAN_TX"

#define CAN_ERROR_STATE   2
#define CAN_RUNNING_STATE 1

// CAN GPIOs
#define CAN_TX_GPIO GPIO_NUM_16
#define CAN_RX_GPIO GPIO_NUM_17

// CAN ID
#define CAN_TEST_ID 0x100

extern uint8_t payload_base;

void can_init(void);
void can_tx_task(void *arg);