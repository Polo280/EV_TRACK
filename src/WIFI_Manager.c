#include "WIFI_Manager.h"
#include <stdlib.h>

//////// WIFI Settings //////// 
// const char *ssid = "55 HOME";
// const char *pass = "bienvenidos55";

const char *ssid = "POCO";
const char *pass = "12345678";

const bool allow_to_reconnect = true;
bool wifi_connected_flag = false;

char ip_address[20];

//////////////////////////////// 

/**
 * @brief Initialize the Non-Volatile Storage (NVS).
 *
 * This function initializes the NVS flash partition. If the NVS
 * partition is found to be full, corrupted, or incompatible with
 * the current ESP-IDF version, it erases the partition and
 * re-initializes it.
 *
 * This ensures that NVS is always in a usable state for storing
 * persistent data such as Wi-Fi credentials, device settings,
 * and calibration values.
 */
void NVS_Init(void){
    esp_err_t nvs_status = nvs_flash_init();
    if(nvs_status == ESP_ERR_NVS_NO_FREE_PAGES || nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND){
        printf("ERROR NVS was thrown\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
        ESP_ERROR_CHECK(nvs_status);
    }
}


void WIFI_Event_Handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_START:
            // ESP_LOGD("WIFI", "WIFI connecting ...");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            // ESP_LOGI("WIFI", "WIFI Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            // ESP_LOGW("WIFI", "WIFI disconnected");
            wifi_connected_flag = false;
            current_status_code = STATUS_OFFLINE;
            // Manage reconnection after losing signal
            if (allow_to_reconnect){
                esp_wifi_connect();
            }
            break;

        case IP_EVENT_STA_GOT_IP:
            // ESP_LOGI("WIFI", "WIFI got IP address");
            current_status_code = STATUS_ONLINE;
            wifi_connected_flag = true; 
            break;

        default:
            // printf("An unrecognized WiFi event with id %ld occured\n", event_id);
            break;
    }
}


void WIFI_Connect(void){
    esp_netif_init();
    esp_event_loop_create_default(); 
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    // Register event handler for WiFi
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WIFI_Event_Handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, WIFI_Event_Handler, NULL);

    // WiFi configuration struct
    wifi_config_t wifi_configuration = { 
        .sta = {
            .ssid = "",
            .password= ""
        }
    };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);

    // Start  configured WiFi and connect 
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_connect();  // Will trigger event handler
}


void  post_data(void *pvParameter)
{
    TelemetryData *data = (TelemetryData *)pvParameter;

    WIFI_Connect();

    while(true)
    {
        esp_http_client_config_t config = {
            .url = "http://192.168.68.107:8080/telemetry",
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);

        char post_data[640];

        snprintf(post_data, sizeof(post_data),
            "{"
            "\"voltage_battery\": %.2f, "
            "\"current\": %.2f, "
            "\"latitude\": %.6f, "
            "\"longitude\": %.6f, "
            "\"acceleration_x\": %.2f, "
            "\"acceleration_y\": %.2f, "
            "\"acceleration_z\": %.2f, "
            "\"orientation_x\": %.2f, "
            "\"orientation_y\": %.2f, "
            "\"orientation_z\": %.2f, "
            "\"rpm_motor\": %d, "
            "\"velocity_x\": %.2f, "
            "\"velocity_y\": %.2f, "
            "\"ambient_temp\": %.2f, "
            "\"altitude_m\": %.2f, "
            "\"num_sats\": %d, "
            "\"air_speed\": %.2f, "
            "\"steering_angle\": 0.0"
            "}",
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
            data->air_speed
        );

        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_post_field(client, post_data, strlen(post_data));
        esp_http_client_set_header(client, "Content-Type", "application/json");

        esp_err_t err = esp_http_client_perform(client);

        // if (err == ESP_OK) {
        //     ESP_LOGI("HTTP", "Telemetry sent");
        // } else {
        //     ESP_LOGE("HTTP", "POST failed: %s", esp_err_to_name(err));
        // }

        esp_http_client_cleanup(client);

        vTaskDelay(pdMS_TO_TICKS(500));
}
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
