#include "../include/SD_memory.h"

const char *TAG_SD = "SD_WRITE";

esp_err_t init_sdcard(void)
{
    esp_err_t ret;

    // 1) Host config for SDSPI
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // 2) SPI bus config
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    host.max_freq_khz = 400; 

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3) Device/slot config (THIS is your “slot_config”)
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    // 4) Mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
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
 * @brief Append a line of text to a file on the SD card.
 *
 * @param path Full path to file, e.g. "/sdcard/log.txt"
 * @param text Text to write (without newline)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sd_append_line(const char *path, const char *text)
{
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