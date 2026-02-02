#ifndef MS4525DO_H
#define MS4525DO_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "Globals.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MS4525DO device descriptor
 */
typedef struct {
    i2c_port_t i2c_port;
    uint8_t    i2c_addr;
    float      zero_offset_pa;
} ms4525do_t;


/**
 * @brief Initialize MS4525DO driver structure
 *
 * @param dev        Pointer to device descriptor
 * @param i2c_port   ESP-IDF I2C port
 * @param i2c_addr   7-bit I2C address (usually 0x28)
 */
void ms4525do_init(ms4525do_t *dev,
                    i2c_port_t i2c_port,
                    uint8_t i2c_addr);


/**
 * @brief Read raw pressure and temperature values
 *
 * @param dev           Device handle
 * @param raw_pressure  14-bit raw pressure
 * @param raw_temp      11-bit raw temperature
 *
 * @return esp_err_t
 */
esp_err_t ms4525do_read_raw(ms4525do_t *dev,
                            uint16_t *raw_pressure,
                            uint16_t *raw_temp);


/**
 * @brief Read differential pressure and airspeed
 *
 * Pressure is returned in pascals and already compensated by
 * the zero offset obtained with ms4525do_calibrate_zero().
 *
 * @param dev             Device handle
 * @param diff_pa         Differential pressure [Pa]
 * @param airspeed_mps    Airspeed [m/s]
 *
 * @return esp_err_t
 */
esp_err_t ms4525do_read(ms4525do_t *dev,
                        float *diff_pa,
                        float *airspeed_mps);


/**
 * @brief Perform zero calibration (car stopped, no airflow)
 *
 * Stores internal zero offset used by ms4525do_read().
 *
 * @param dev       Device handle
 * @param samples   Number of samples to average
 *
 * @return esp_err_t
 */
esp_err_t ms4525do_calibrate_zero(ms4525do_t *dev, uint32_t samples);


void pitot_task(void *arg);


#ifdef __cplusplus
}
#endif

#endif
