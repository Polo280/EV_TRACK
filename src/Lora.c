#include "Lora.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/* SX1276 / RFM95 registers */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_PAYLOAD_LENGTH  0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_IRQ_FLAGS            0x12

#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03

#define PA_BOOST                 0x80

/* 868 MHz (EU – Shell Eco typically uses EU bands) */
#define LORA_FREQ_HZ  868000000UL


static inline void lora_cs_low(lora_t *dev)
{
    gpio_set_level(dev->cs_pin, 0);
}

static inline void lora_cs_high(lora_t *dev)
{
    gpio_set_level(dev->cs_pin, 1);
}


static esp_err_t lora_write_reg(lora_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { (uint8_t)(reg | 0x80), val };

    spi_transaction_t t = {
        .length = 8 * sizeof(buf),
        .tx_buffer = buf,
        .rx_buffer = NULL
    };

    lora_cs_low(dev);
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    lora_cs_high(dev);

    return ret;
}


static esp_err_t lora_read_reg(lora_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2];

    spi_transaction_t t = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    lora_cs_low(dev);
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    lora_cs_high(dev);

    if (ret != ESP_OK)
        return ret;

    *val = rx[1];
    return ESP_OK;
}


static void lora_reset(lora_t *dev)
{
    gpio_set_level(dev->reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(dev->reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}


static void lora_set_frequency(lora_t *dev, uint32_t freq_hz)
{
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000;

    lora_write_reg(dev, REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(dev, REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(dev, REG_FRF_LSB, (uint8_t)(frf >> 0));
}


esp_err_t lora_init(lora_t *dev,
                    spi_device_handle_t spi,
                    int cs_pin,
                    int reset_pin)
{
    dev->spi = spi;
    dev->cs_pin = cs_pin;
    dev->reset_pin = reset_pin;

    gpio_reset_pin(reset_pin);
    gpio_set_direction(reset_pin, GPIO_MODE_OUTPUT);

    gpio_reset_pin(cs_pin);
    gpio_set_direction(cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(cs_pin, 1);

    lora_reset(dev);
    return ESP_OK;
}


esp_err_t lora_send(lora_t *dev,
                    const uint8_t *data,
                    uint16_t len)
{
    if (len == 0 || len > 255)
        return ESP_ERR_INVALID_ARG;

    /* Standby */
    lora_write_reg(dev, REG_OP_MODE,
                    MODE_LONG_RANGE_MODE | MODE_STDBY);

    /* Reset FIFO pointer */
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0x00);

    /* Write payload */
    for (uint8_t i = 0; i < len; i++) {
        lora_write_reg(dev, REG_FIFO, data[i]);
    }

    lora_write_reg(dev, REG_FIFO_PAYLOAD_LENGTH, len);

    /* Clear IRQs */
    lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF);

    /* TX */
    lora_write_reg(dev, REG_OP_MODE,
                    MODE_LONG_RANGE_MODE | MODE_TX);

    /* Wait for TX done */
    uint8_t irq;

    do {
        lora_read_reg(dev, REG_IRQ_FLAGS, &irq);
        vTaskDelay(pdMS_TO_TICKS(2));
    } while ((irq & 0x08) == 0);

    /* Clear TX done */
    lora_write_reg(dev, REG_IRQ_FLAGS, 0x08);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Telemetry packet                                                           */
/* -------------------------------------------------------------------------- */
typedef struct __attribute__((packed))
{
    float   battery_voltage;
    float   current_amps;
    float   air_speed;
    double  latitude;
    double  longitude;
    uint16_t rpms;
} lora_telem_packet_t;


/*
 * Keep the RF payload small and deterministic.
 * Do NOT dump the entire TelemetryData struct over RF.
 */
esp_err_t lora_send_telemetry(lora_t *dev,
                              const TelemetryData *telem)
{
    lora_telem_packet_t p;

    p.battery_voltage = telem->battery_voltage;
    p.current_amps    = telem->current_amps;
    p.air_speed       = telem->air_speed;
    p.latitude        = telem->latitude;
    p.longitude       = telem->longitude;
    p.rpms            = telem->rpms;

    return lora_send(dev,
                     (const uint8_t *)&p,
                     sizeof(p));
}
