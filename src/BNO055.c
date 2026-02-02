#include "BNO055.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>

/* --------------------------------------------------------- */
/* BNO055 definitions                                       */
/* --------------------------------------------------------- */

#define BNO055_I2C_ADDR          0x29

#define BNO055_PAGE_ID   0x07

#define BNO055_CHIP_ID           0x00
#define BNO055_OPR_MODE          0x3D
#define BNO055_PWR_MODE          0x3E
#define BNO055_SYS_TRIGGER       0x3F
#define BNO055_UNIT_SEL          0x3B

#define BNO055_EULER_H_LSB       0x1A
#define BNO055_LIA_DATA_X_LSB    0x28

#define BNO055_CHIP_ID_VAL       0xA0

#define BNO055_TEMP                0x34
#define BNO055_CALIB_STAT          0x35

/* Operation modes */
#define BNO055_MODE_CONFIG       0x00
#define BNO055_MODE_NDOF         0x0C

#define BNO055_SYS_STATUS  0x39
#define BNO055_SYS_ERR     0x3A

int fail_count = 0;


static bool bno055_write_reg(uint8_t reg, uint8_t data)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                           (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                           true);
    i2c_master_write_byte(cmd, reg,  true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_INSTANCE,
                                         cmd,
                                         pdMS_TO_TICKS(200));

    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);

    return (ret == ESP_OK);
}


static bool bno055_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    bool ok;

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    /* write register address */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);

    ok = (i2c_master_cmd_begin(I2C_INSTANCE, cmd,
                               pdMS_TO_TICKS(400)) == ESP_OK);

    i2c_cmd_link_delete(cmd);

    if (!ok) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    /* small delay like your STM code */
    vTaskDelay(pdMS_TO_TICKS(2));

    /* now read */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (BNO055_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);

    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ok = (i2c_master_cmd_begin(I2C_INSTANCE, cmd,
                               pdMS_TO_TICKS(400)) == ESP_OK);

    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);

    return ok;
}


static bool bno055_write_retry(uint8_t reg, uint8_t val)
{
    for (int i = 0; i < 5; i++)
    {
        if (bno055_write_reg(reg, val))
            return true;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}


static bool bno055_read_retry(uint8_t reg, uint8_t *val)
{
    for (int i = 0; i < 5; i++)
    {
        if (bno055_read_reg(reg, val, 1))
            return true;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}


static bool bno055_force_page0(void)
{
    return bno055_write_retry(0x07, 0x00);   // PAGE_ID
}


static bool bno055_read_system_status(uint8_t *stat, uint8_t *err)
{
    if (!bno055_force_page0())
        return false;

    if (!bno055_read_reg(BNO055_SYS_STATUS, stat, 1))
        return false;

    if (!bno055_read_reg(BNO055_SYS_ERR, err, 1))
        return false;

    return true;
}


static bool bno055_soft_reset(void)
{
    /* must be on page 0 */
    if (!bno055_force_page0())
        return false;

    /* go to config mode before reset */
    bno055_write_retry(BNO055_OPR_MODE, BNO055_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* trigger reset */
    if (!bno055_write_retry(BNO055_SYS_TRIGGER, 0x20))
        return false;

    /* datasheet: reset takes up to 650 ms */
    vTaskDelay(pdMS_TO_TICKS(700));

    return true;
}



/* --------------------------------------------------------- */

bool bno055_init(void)
{
    uint8_t id = 0;

    vTaskDelay(pdMS_TO_TICKS(700));

    for (int i = 0; i < 10; i++)
    {
        if (bno055_read_retry(BNO055_CHIP_ID, &id) &&
            id == BNO055_CHIP_ID_VAL)
            break;

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (id != BNO055_CHIP_ID_VAL)
        return false;

    if (!bno055_write_retry(BNO055_PAGE_ID, 0x00))
        return false;

    if (!bno055_write_retry(BNO055_OPR_MODE, BNO055_MODE_CONFIG))
        return false;
    vTaskDelay(pdMS_TO_TICKS(30));

    if (!bno055_write_retry(BNO055_PWR_MODE, 0x00))
        return false;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!bno055_write_retry(BNO055_UNIT_SEL, 0x00))
        return false;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!bno055_write_retry(BNO055_SYS_TRIGGER, 0x00))
        return false;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!bno055_write_retry(BNO055_OPR_MODE, BNO055_MODE_NDOF))
        return false;

    vTaskDelay(pdMS_TO_TICKS(100));


    /* wait for fusion engine to start */
    uint8_t sys, g, a, m;

    for (int i = 0; i < 50; i++)   // up to ~1 second
    {
        if (bno055_get_calib_status(&sys, &g, &a, &m))
        {
            if (sys != 0)   // fusion running
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}


/* --------------------------------------------------------- */

bool bno055_read_euler(float *heading_deg,
                        float *roll_deg,
                        float *pitch_deg)
{
    uint8_t buf[6];

    if (!bno055_force_page0())
        return false;

    if (!bno055_read_reg(BNO055_EULER_H_LSB, buf, 6))
        return false;

    int16_t h = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t p = (int16_t)((buf[5] << 8) | buf[4]);

    /* 1 LSB = 1/16 degree */
    *heading_deg = (float)h / 16.0f;
    *roll_deg    = (float)r / 16.0f;
    *pitch_deg   = (float)p / 16.0f;

    return true;
}

/* --------------------------------------------------------- */

bool bno055_read_linear_accel(float *ax,
                               float *ay,
                               float *az)
{
    uint8_t buf[6];

    if (!bno055_force_page0())
        return false;

    if (!bno055_read_reg(BNO055_LIA_DATA_X_LSB, buf, 6))
        return false;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    /*
     * Linear acceleration scale:
     * 1 LSB = 1 mg = 0.01 m/s^2   (per datasheet default units)
     */
    const float LSB_TO_MS2 = 0.00980665f;

    *ax = (float)x * LSB_TO_MS2;
    *ay = (float)y * LSB_TO_MS2;
    *az = (float)z * LSB_TO_MS2;

    return true;
}


bool bno055_get_calib_status(uint8_t *sys,
                             uint8_t *gyro,
                             uint8_t *accel,
                             uint8_t *mag)
{
    uint8_t cal;

    if (!bno055_force_page0())
        return false;

    if (!bno055_read_reg(BNO055_CALIB_STAT, &cal, 1))
        return false;

    /*
     * Bits:
     * [7:6] = SYS
     * [5:4] = GYR
     * [3:2] = ACC
     * [1:0] = MAG
     *
     * Each value: 0..3
     */

    if (sys)   *sys   = (cal >> 6) & 0x03;
    if (gyro)  *gyro  = (cal >> 4) & 0x03;
    if (accel) *accel = (cal >> 2) & 0x03;
    if (mag)   *mag   =  cal       & 0x03;

    return true;
}


bool bno055_read_temperature(float *temp_c)
{
    int8_t t;

    if (!bno055_force_page0())
        return false;

    if (!bno055_read_reg(BNO055_TEMP, (uint8_t *)&t, 1))
        return false;

    /*
     * Datasheet:
     * 1 LSB = 1 °C
     * signed value
     */
    if (temp_c)
        *temp_c = (float)t;

    return true;
}


// static bool i2c_probe_old(uint8_t addr)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_stop(cmd);

//     esp_err_t ret = i2c_master_cmd_begin(I2C_INSTANCE, cmd,
//                                          pdMS_TO_TICKS(200));

//     i2c_cmd_link_delete(cmd);

//     return (ret == ESP_OK);
// }


static const char *TAG = "BNO055_TASK";


void bno055_task(void *arg)
{
    TelemetryData *telem = (TelemetryData *)arg;

    float ax, ay, az;
    float heading, roll, pitch;
    float temp_c;

    uint8_t sys, gyro, accel, mag;

    // /* ---------- First: old-style probe (known working) ---------- */

    // vTaskDelay(pdMS_TO_TICKS(50));   // small settle time

    // if (!i2c_probe_old(0x29))
    // {
    //     ESP_LOGE(TAG, "Old probe failed (0x29). I2C not ready for BNO055.");
    //     vTaskDelete(NULL);
    //     return;
    // }

    // ESP_LOGI(TAG, "Old probe OK (0x29)");

    /* ---------- Now try normal init ---------- */

    ESP_LOGI(TAG, "Initializing BNO055...");

    if (!bno055_init())
    {
        ESP_LOGE(TAG, "BNO055 init failed");
        vTaskDelete(NULL);
        return;
    }


    ESP_LOGI(TAG, "BNO055 initialized");

    while (1)
    {
        bool ok_acc  = false;
        bool ok_ori  = false;
        bool ok_temp = false;

        /* --------- Calibration status --------- */

        if (bno055_get_calib_status(&sys, &gyro, &accel, &mag))
        {
            ESP_LOGD(TAG,
                     "CAL: sys=%u gyro=%u accel=%u mag=%u",
                     sys, gyro, accel, mag);
        }

        /* --------- Linear acceleration --------- */

        if (bno055_read_linear_accel(&ax, &ay, &az))
            ok_acc = true;

        /* --------- Orientation --------- */

        if (bno055_read_euler(&heading, &roll, &pitch))
            ok_ori = true;

        /* --------- Temperature --------- */

        if (bno055_read_temperature(&temp_c)){
            ok_temp = true;
        }

        
        if (!ok_acc && !ok_ori && !ok_temp)
        {
            fail_count++;

            uint8_t st, er;
            if (bno055_read_system_status(&st, &er))
            {
                ESP_LOGW(TAG, "BNO055 fail #%d  SYS_STATUS=%u  SYS_ERR=%u",
                        fail_count, st, er);
            }

            if (fail_count >= 5)
            {
                ESP_LOGE(TAG, "BNO055 seems stuck - resetting sensor");

                /* real recovery */
                if (!bno055_soft_reset())
                {
                    ESP_LOGE(TAG, "BNO055 soft reset failed");
                }

                if (!bno055_init())
                {
                    ESP_LOGE(TAG, "BNO055 re-init failed");
                }
                else
                {
                    ESP_LOGI(TAG, "BNO055 recovered");
                }

                fail_count = 0;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

    else
    {
        fail_count = 0;
    }

        /* --------- Store into telemetry --------- */

        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);

        if (ok_acc)
        {
            telem->accel_x = ax;
            telem->accel_y = ay;
            telem->accel_z = az;
        }

        if (ok_ori)
        {
            telem->orient_x = heading;
            telem->orient_y = roll;
            telem->orient_z = pitch;
        }

        if (ok_temp)
        {
            telem->ambient_temp = temp_c;
        }

        xSemaphoreGive(telemetry_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
