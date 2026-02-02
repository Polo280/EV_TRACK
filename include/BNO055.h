#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "Globals.h"


bool bno055_init(void);

/* Reads Euler orientation (degrees) */
bool bno055_read_euler(float *heading_deg,
                        float *roll_deg,
                        float *pitch_deg);

/* Reads linear acceleration (m/s^2) */
bool bno055_read_linear_accel(float *ax,
                               float *ay,
                               float *az);

/* NEW: calibration status */
bool bno055_get_calib_status(uint8_t *sys,
                             uint8_t *gyro,
                             uint8_t *accel,
                             uint8_t *mag);

/* NEW: temperature (°C) */
bool bno055_read_temperature(float *temp_c);

void bno055_task(void *arg) ;
