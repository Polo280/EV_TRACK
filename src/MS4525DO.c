#include "MS4525DO.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MS4525DO_COUNTS_MIN   1638.0f
#define MS4525DO_COUNTS_MAX   14745.0f

/* MS4525DO-001DP */
#define MS4525DO_P_MIN_PSI   (-1.0f)
#define MS4525DO_P_MAX_PSI   ( 1.0f)

#define PSI_TO_PA  6894.757f


static float ms4525do_raw_to_pa(uint16_t raw)
{
    float psi =
        ((float)raw - MS4525DO_COUNTS_MIN) *
        (MS4525DO_P_MAX_PSI - MS4525DO_P_MIN_PSI) /
        (MS4525DO_COUNTS_MAX - MS4525DO_COUNTS_MIN) +
        MS4525DO_P_MIN_PSI;

    return psi * PSI_TO_PA;
}


static float ms4525do_pa_to_airspeed(float dp_pa)
{
    const float rho = 1.225f;   // air density at sea level

    if (dp_pa <= 0.0f)
        return 0.0f;

    return sqrtf(2.0f * dp_pa / rho);
}


void ms4525do_init(ms4525do_t *dev,
                    i2c_port_t i2c_port,
                    uint8_t i2c_addr)
{
    dev->i2c_port      = i2c_port;
    dev->i2c_addr      = i2c_addr;
    dev->zero_offset_pa = 0.0f;
}


esp_err_t ms4525do_read_raw(ms4525do_t *dev,
                            uint16_t *raw_pressure,
                            uint16_t *raw_temp)
{
    uint8_t buf[4];

    esp_err_t ret = i2c_master_read_from_device(
                        dev->i2c_port,
                        dev->i2c_addr,
                        buf,
                        4,
                        pdMS_TO_TICKS(50));

    if (ret != ESP_OK)
        return ret;

    uint8_t status = (buf[0] >> 6) & 0x03;

    /* 0 = normal, 2 = stale, 3 = fault */
    if (status != 0)
        return ESP_FAIL;

    if (raw_pressure)
        *raw_pressure = ((uint16_t)(buf[0] & 0x3F) << 8) | buf[1];

    if (raw_temp)
        *raw_temp = ((uint16_t)buf[2] << 3) | (buf[3] >> 5);

    return ESP_OK;
}


esp_err_t ms4525do_read(ms4525do_t *dev,
                        float *diff_pa,
                        float *airspeed_mps)
{
    uint16_t raw_p, raw_t;

    esp_err_t ret = ms4525do_read_raw(dev, &raw_p, &raw_t);
    if (ret != ESP_OK)
        return ret;

    float pa = ms4525do_raw_to_pa(raw_p) - dev->zero_offset_pa;

    if (diff_pa)
        *diff_pa = pa;

    if (airspeed_mps)
        *airspeed_mps = ms4525do_pa_to_airspeed(pa);

    return ESP_OK;
}


esp_err_t ms4525do_calibrate_zero(ms4525do_t *dev, uint32_t samples)
{
    if (samples == 0)
        return ESP_ERR_INVALID_ARG;

    float sum = 0.0f;
    uint32_t valid = 0;

    for (uint32_t i = 0; i < samples; i++) {

        uint16_t rp, rt;

        if (ms4525do_read_raw(dev, &rp, &rt) == ESP_OK) {
            sum += ms4525do_raw_to_pa(rp);
            valid++;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (valid == 0)
        return ESP_FAIL;

    dev->zero_offset_pa = sum / (float)valid;

    return ESP_OK;
}


void pitot_task(void *arg)
{
    TelemetryData *telem = (TelemetryData *)arg;

    ms4525do_t pitot;

    /* Init device */
    ms4525do_init(&pitot, I2C_NUM_0, 0x28);

    /* Let sensor settle */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Zero calibration (car must be stopped) */
    ms4525do_calibrate_zero(&pitot, 300);

    while (1) {

        float dp_pa;
        float airspeed;

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        if (ms4525do_read(&pitot, &dp_pa, &airspeed) == ESP_OK) {

            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            telem->air_speed = airspeed;   // m/s
            xSemaphoreGive(telemetry_mutex);
        }
        xSemaphoreGive(i2c_mutex);

        // printf("The air speed is: %f\n", telem->air_speed);
        vTaskDelay(pdMS_TO_TICKS(1000));   // 10 Hz pitot update
    }
}