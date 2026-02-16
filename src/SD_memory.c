#include "../include/SD_memory.h"

const char *TAG_SD = "SD_WRITE";

sdmmc_card_t *card;


esp_err_t mount_sdcard(void)
{
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_SPI_CS_PIN;
    slot_config.host_id = (spi_host_device_t)host.slot;

    // SD Mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const char mount_point[] = "/sdcard";

    // 5) Mount the SD card
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to mount card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("SD", "SD card mounted");
    return ret;
}


/**
 * @brief Unmount the SD card filesystem.
 *
 * This function unmounts the FAT filesystem from the SD card that was
 * previously mounted at the "/sdcard" mount point using
 * esp_vfs_fat_sdspi_mount() or esp_vfs_fat_sdmmc_mount().
 *
 * It should be called when the SD card is physically removed or when the
 * application no longer needs access to the SD card, in order to:
 *  - Flush pending writes
 *  - Release the VFS resources
 *  - Safely detach the card from the system
 *
 * @return
 *  - ESP_OK on successful unmount
 *  - An error code if unmounting fails (e.g., invalid handle or not mounted)
 */
esp_err_t unmount_sdcard(void){
    esp_err_t ret;
    ret = esp_vfs_fat_sdcard_unmount("/sdcard", card);
    return ret;
}


/**
 * @brief Append a line of text to a file on the SD card.
 *
 * @param path Full path to file, e.g. "/sdcard/log.txt"
 * @param text Text to write (without newline)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sd_append_line(const char *path, const char *text)
{
    if(!SD_card_detected){
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    // Open file in append mode
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Failed to open file for appending: %s", path);
        return ESP_FAIL;
    }

    // Write text + newline
    if (fprintf(f, "%s\n", text) < 0) {
        ESP_LOGE(TAG_SD, "Failed to write to file: %s", path);
        fclose(f);
        return ESP_FAIL;
    }

    // Make sure data is actually written
    fflush(f);
    fclose(f);

    ESP_LOGI(TAG_SD, "Wrote line to: %s", path);
    return ESP_OK;
}


/**
 * @brief Task that monitors SD card insertion/removal.
 *
 * Watches the SD_card_detected flag and mounts the SD card on insertion
 * and unmounts it on removal.
 *
 * @param arg Unused.
 */
void SD_manager_task(void *arg){
    TelemetryData *telemetry_data = (TelemetryData *)arg;
    bool last_state = false;

    while (1) {
        if (SD_card_detected != last_state) {
            last_state = SD_card_detected;

            if (last_state) {
                ESP_LOGI("SD", "Card inserted");
                esp_err_t ret = mount_sdcard();
                if(ret == ESP_OK){
                    sd_write_csv_header("/sdcard/telem.csv");
                }
            } else {
                ESP_LOGI("SD", "Card removed");
                unmount_sdcard();
            }
        }

        if(SD_card_detected){
            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            sd_append_telemetry_csv("/sdcard/telem.csv", telemetry_data);
            xSemaphoreGive(telemetry_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/**
 * @brief Create a CSV file and write the telemetry header line.
 *
 * This function creates (or overwrites) a CSV file at the given path and
 * writes the column headers corresponding to the TelemetryData fields.
 * It should be called once before appending telemetry samples.
 *
 * @param path Full path to the CSV file (e.g., "/sdcard/telemetry.csv").
 * @return
 *  - ESP_OK on success
 *  - ESP_FAIL if the file could not be created or written
 */
esp_err_t sd_write_csv_header(const char *path)
{
    if(!SD_card_detected){
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    const char *header =
        "battery_voltage,current_amps,latitude,longitude,"
        "accel_x,accel_y,accel_z,"
        "orient_x,orient_y,orient_z,"
        "rpms,velocity_x,velocity_y,ambient_temp,altitude_m,num_sats,air_speed,throttle_raw";

    FILE *f = fopen(path, "w");   // "w" creates/overwrites and writes header once
    if (!f) {
        ESP_LOGE("SD", "Failed to create CSV file: %s", path);
        return ESP_FAIL;
    }

    fprintf(f, "%s\n", header);
    fflush(f);
    fclose(f);

    ESP_LOGI("SD", "CSV header written to %s", path);
    return ESP_OK;
}


/**
 * @brief Append one TelemetryData sample as a CSV row.
 *
 * Writes the fields of a TelemetryData structure to a CSV file in the same
 * order as the header created by sd_write_csv_header().
 *
 * @param path Full path to the CSV file (e.g. "/sdcard/telem.csv").
 * @param data Pointer to the TelemetryData structure to log.
 *
 * @return
 *  - ESP_OK on success
 *  - ESP_FAIL if the SD card is not present or the write fails
 */
esp_err_t sd_append_telemetry_csv(const char *path, const TelemetryData *data)
{
    if (!SD_card_detected) {
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Failed to open CSV file for appending: %s", path);
        return ESP_FAIL;
    }

    int ret = fprintf(f,
        "%.3f,%.3f,%.8lf,%.8lf,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%u,%.3f,%.3f,%.2f,%.2f,%u,%.2f,%d\n",
        data->battery_voltage,
        data->current_amps,
        data->latitude,
        data->longitude,
        data->accel_x,
        data->accel_y,
        data->accel_z,
        data->orient_x,
        data->orient_y,
        data->orient_z,
        data->rpms,
        data->velocity_x,
        data->velocity_y,
        data->ambient_temp,
        data->altitude_m,
        data->num_sats, 
        data->air_speed,
        data->throttle_raw
    );

    if (ret < 0) {
        ESP_LOGE(TAG_SD, "Failed to write telemetry row");
        fclose(f);
        return ESP_FAIL;
    }

    fflush(f);   // ensure physical write
    fclose(f);

    return ESP_OK;
}


void SD_append_data_task(void *arg){
    TelemetryData *telem = (TelemetryData *)arg;  // cast

    while (1) {
        sd_append_telemetry_csv("/sdcard/telem.csv", telem);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}