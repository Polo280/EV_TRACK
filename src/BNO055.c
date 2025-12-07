// ESP-IDF translation of your BNO055 HAL code
#include "BNO055.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- Config ----------
#ifndef BNO055_I2C_ADDR_LO
// Define if not defined by your header. ESP-IDF APIs expect 7-bit addr.
#define BNO055_I2C_ADDR_LO  (0x28)   // ADR low (0x28) / high (0x29)
#endif

static const char *TAG = "BNO055";

static i2c_port_t s_i2c_port = I2C_NUM_0;

bno055_conf_t default_bno055_config = {
    .pwr_mode = POWER_MODE_NORMAL,
    .op_mode = OPERATION_MODE_NDOF,
    .axis_remap_conf = AXIS_REMAP_CONFIG_P0,
    .axis_remap_sign = AXIS_REMAP_SIGN_P0,
    .acc_g_range = ACC_CONFIG_4G,
    .acc_bandwidth = ACC_CONFIG_62_5Hz,
    .acc_operation_mode = ACC_CONFIG_NORMAL,
    .gyr_range = GYR_CONFIG_250DPS,
    .gyr_bandwidth = GYR_CONFIG_116Hz,
    .gyr_op_mode = GYR_CONFIG_NORMAL,
    .mag_data_rate = MAG_CONFIG_10Hz,
    .mag_op_mode = MAG_CONFIG_REGULAR,
    .mag_pwr_mode = MAG_CONFIG_NORMAL,
    .unit_sel = ACCELERATION_M_S2 | ANGULAR_RATE_DPS | EULER_ANGLES_DEG | TEMPERATURE_C
};

bno055_verification_t default_bno055_verification = {
    .chip_id = 0,
    .sw_rev_id = 0,
    .page_id = 0,
    .acc_id = 0,
    .mag_id = 0,
    .gyr_id = 0,
    .bl_rev_id = 0
};

// ---------- Port selector ----------
void bno055_set_i2c_handler(i2c_port_t port) {
    s_i2c_port = port;
}

// ---------- Utility ----------
static inline TickType_t _to_ticks(uint32_t timeout_ms) {
    return (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
}

// ---------- ESP-IDF I2C wrappers  ----------
uint8_t bno055_writeData_len(const uint8_t *txdata, size_t len, uint32_t timeout_ms) {
    if (!txdata || len == 0) return 1;

    esp_err_t err = i2c_master_write_to_device(
        s_i2c_port,
        BNO055_I2C_ADDR_LO,      // 7-bit address, no shifting
        txdata,
        len,
        _to_ticks(timeout_ms)
    );

    if (err == ESP_OK) return 0;

    ESP_LOGE(TAG, "i2c_master_write_to_device err=%d (%s)", err, esp_err_to_name(err));
    return 1;
}

// Backward-compatible wrapper with default timeout = 10ms and "2-byte register write" expectation
uint8_t bno055_writeData(uint8_t *txdata) {
    return bno055_writeData_len(txdata, 2, 10);
}

uint8_t bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
    if (!data || len == 0) return 1;

    esp_err_t err = i2c_master_write_read_device(
        s_i2c_port,
        BNO055_I2C_ADDR_LO,
        &reg,
        1,
        data,
        len,
        _to_ticks(20)    // small combined timeout
    );

    if (err == ESP_OK) {
        return 0;
    } else {
        ESP_LOGE(TAG, "i2c_master_write_read_device err=%d (%s), reg=0x%02X len=%u",
                 err, esp_err_to_name(err), reg, (unsigned)len);
        return 1;
    }
}

void bno055_delay(uint32_t ms) {
    // Works from tasks. If you need it before scheduler starts, use ets_delay_us(ms*1000).
    vTaskDelay(pdMS_TO_TICKS(ms));
}


BNO055_FUNC_RETURN bno055_init(bno055_conf_t *bno055_conf, bno055_verification_t *bno055_verification){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;

    uint8_t conf_page0 [2] = {BNO055_PAGE_ID, 0x00};
    uint8_t op_mode_conf [2] = {BNO055_OPR_MODE, OPERATION_MODE_CONFIG};

    ret += bno055_writeData_len(conf_page0, sizeof(conf_page0), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(op_mode_conf, sizeof(op_mode_conf), 10);
    bno055_delay(550);

    uint8_t conf_page1 [2] = {BNO055_PAGE_ID, 0x01};
    uint8_t acc_conf  [2] = {BNO055_ACC_CONFIG,    (uint8_t)((bno055_conf->acc_operation_mode << 5) | (bno055_conf->acc_bandwidth << 2) | bno055_conf->acc_g_range)};
    uint8_t gyr_conf0 [2] = {BNO055_GYRO_CONFIG_0, (uint8_t)((bno055_conf->gyr_bandwidth << 3) | bno055_conf->gyr_range)};
    uint8_t gyr_conf1 [2] = {BNO055_GYRO_CONFIG_1, bno055_conf->gyr_op_mode};
    uint8_t mag_conf  [2] = {BNO055_MAG_CONFIG,    (uint8_t)((bno055_conf->mag_pwr_mode << 5) | (bno055_conf->mag_op_mode << 3) | bno055_conf->mag_data_rate)};

    ret += bno055_writeData_len(conf_page1, sizeof(conf_page1), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(acc_conf,  sizeof(acc_conf), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(gyr_conf0, sizeof(gyr_conf0), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(gyr_conf1, sizeof(gyr_conf1), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(mag_conf,  sizeof(mag_conf), 10);
    bno055_delay(10);

    uint8_t pwr_mode        [2] = {BNO055_PWR_MODE,       bno055_conf->pwr_mode};
    uint8_t op_mode         [2] = {BNO055_OPR_MODE,       bno055_conf->op_mode};
    uint8_t axis_remap_conf [2] = {BNO055_AXIS_MAP_CONFIG,bno055_conf->axis_remap_conf};
    uint8_t axis_remap_sign [2] = {BNO055_AXIS_MAP_SIGN,  bno055_conf->axis_remap_sign};
    uint8_t unit_sel        [2] = {BNO055_UNIT_SEL,       bno055_conf->unit_sel};

    ret += bno055_writeData_len(conf_page0, sizeof(conf_page0), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(pwr_mode, sizeof(pwr_mode), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(axis_remap_conf, sizeof(axis_remap_conf), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(axis_remap_sign, sizeof(axis_remap_sign), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(unit_sel, sizeof(unit_sel), 10);
    bno055_delay(10);

    ret += bno055_writeData_len(op_mode, sizeof(op_mode), 10);
    bno055_delay(10);

    uint8_t sw_id[2] = {0, 0};
    uint8_t data = 0;

    ret += bno055_readData(BNO055_CHIP_ID, &data, 1);
    bno055_verification->chip_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_ACC_ID, &data, 1);
    bno055_verification->acc_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_MAG_ID, &data, 1);
    bno055_verification->mag_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_GYR_ID, &data, 1);
    bno055_verification->gyr_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_BL_REV_ID, &data, 1);
    bno055_verification->bl_rev_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_SW_REV_ID_LSB, sw_id, 2);
    bno055_verification->sw_rev_id = (uint16_t)((sw_id[1] << 8)|(sw_id[0]));
    bno055_delay(10);

    ret += bno055_readData(BNO055_PAGE_ID, &data, 1);
    bno055_verification->page_id = data;
    bno055_delay(100);

    return ret;
}

// --- The rest of your read helpers stay the same, calling bno055_readData() ---
// I’ll keep your original logic, just using ESP-IDF I2C under the hood.

BNO055_FUNC_RETURN bno055_read_acc_x(uint16_t* acc_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_X_LSB, data, 2);
    *acc_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_y(uint16_t* acc_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_Y_LSB, data, 2);
    *acc_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_z(uint16_t* acc_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_Z_LSB, data, 2);
    *acc_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_xyz(bno055_acc_t* acc_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_ACC_DATA_X_LSB, data, 6);
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
    const float accScale = 1.0f / 100.0f;
    acc_xyz->x = rawX * accScale;
    acc_xyz->y = rawY * accScale;
    acc_xyz->z = rawZ * accScale;
    return ret;
}

BNO055_FUNC_RETURN bno055_read_mag_x(uint16_t* mag_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_X_LSB, data, 2);
    *mag_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_y(uint16_t* mag_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Y_LSB, data, 2);
    *mag_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_z(uint16_t* mag_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *mag_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_xyz(bno055_mag_t* mag_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_MAG_DATA_X_LSB, data, 6);
    mag_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    mag_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    mag_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_gyr_x(uint16_t* gyr_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GYR_DATA_X_LSB, data, 2);
    *gyr_x = (uint16_t)((data[1] << 8)|(data[0])); // note: your original had "+=", likely unintended
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_y(uint16_t* gyr_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GYR_DATA_Y_LSB, data, 2);
    *gyr_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_z(uint16_t* gyr_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GYR_DATA_Z_LSB, data, 2);
    *gyr_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_xyz(bno055_gyr_t* gyr_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_GYR_DATA_X_LSB, data, 6);
    gyr_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    gyr_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    gyr_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_euler_h(uint16_t* euler_h){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 2);
    *euler_h = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_r(uint16_t* euler_r){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_ROLL_LSB, data, 2);
    *euler_r = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_p(uint16_t* euler_p){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_PITCH_LSB, data, 2);
    *euler_p = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_hrp(bno055_euler_t* euler_hrp){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 6);
    int16_t rawHeading = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawRoll    = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawPitch   = (int16_t)((data[5] << 8) | data[4]);
    float scaleFactor = 1.0f / 16.0f;
    euler_hrp->h = rawHeading * scaleFactor;
    euler_hrp->r = rawRoll * scaleFactor;
    euler_hrp->p = rawPitch * scaleFactor;
    return ret;
}

BNO055_FUNC_RETURN bno055_read_quaternion_w(uint16_t* quaternion_w){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_W_LSB, data, 2);
    *quaternion_w = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_x(uint16_t* quaternion_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_X_LSB, data, 2);
    *quaternion_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_y(uint16_t* quaternion_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_Y_LSB, data, 2);
    *quaternion_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_z(uint16_t* quaternion_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_Z_LSB, data, 2);
    *quaternion_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_wxyz(bno055_quaternion_t* quaternion_wxyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    ret += bno055_readData(BNO055_QUA_DATA_W_LSB, data, 8);
    quaternion_wxyz->w = (float)(uint16_t)((data[1] << 8)|(data[0]));
    quaternion_wxyz->x = (float)(uint16_t)((data[3] << 8)|(data[2]));
    quaternion_wxyz->y = (float)(uint16_t)((data[5] << 8)|(data[4]));
    quaternion_wxyz->z = (float)(uint16_t)((data[7] << 8)|(data[6]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_linear_acc_x(uint16_t* linear_acc_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_X_LSB, data, 2);
    *linear_acc_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_y(uint16_t* linear_acc_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_Y_LSB, data, 2);
    *linear_acc_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_z(uint16_t* linear_acc_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_Z_LSB, data, 2);
    *linear_acc_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_xyz(bno055_linear_acc_t* linear_acc_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_LIA_DATA_X_LSB, data, 6);
    linear_acc_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    linear_acc_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    linear_acc_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_gravity_x(uint16_t* gravity_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_X_LSB, data, 2);
    *gravity_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_y(uint16_t* gravity_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_Y_LSB, data, 2);
    *gravity_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_z(uint16_t* gravity_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_Z_LSB, data, 2);
    *gravity_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_xyz(bno055_gravity_t* gravity_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_GRV_DATA_X_LSB, data, 6);
    gravity_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    gravity_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    gravity_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_temperature(int8_t* temp){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data = 0;
    ret += bno055_readData(BNO055_TEMP, &data, 1);
    *temp = (int8_t)data;
    return ret;
}

BNO055_FUNC_RETURN bno055_get_acc_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}
BNO055_FUNC_RETURN bno055_get_mag_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}
BNO055_FUNC_RETURN bno055_get_gyr_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}
