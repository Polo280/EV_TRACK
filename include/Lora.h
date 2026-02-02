#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "Globals.h"

typedef struct
{
    spi_device_handle_t spi;
    int cs_pin;
    int reset_pin;
} lora_t;


/**
 * @brief Initialize RFM95W (SX1276) in LoRa TX mode
 */
esp_err_t lora_init(lora_t *dev,
                    spi_device_handle_t spi,
                    int cs_pin,
                    int reset_pin);


/**
 * @brief Send a raw buffer over LoRa
 */
esp_err_t lora_send(lora_t *dev,
                    const uint8_t *data,
                    uint16_t len);


/**
 * @brief Serialize and send TelemetryData over LoRa
 */
esp_err_t lora_send_telemetry(lora_t *dev,
                              const TelemetryData *telem);

#endif
