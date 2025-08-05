#include "WIFI_Manager.h"
#include <stdlib.h>

//////// WIFI Settings //////// 
// const char *ssid = "55 HOME";
// const char *pass = "bienvenidos55";

const char *ssid = "CASAGUERO";
const char *pass = "AMADALC1212";

const bool allow_to_reconnect = true;
bool wifi_connected_flag = false;

char ip_address[20];

//////////////////////////////// 

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
            ESP_LOGD("WIFI", "WIFI connecting ...");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("WIFI", "WIFI Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW("WIFI", "WIFI disconnected");
            wifi_connected_flag = false;
            current_status_code = STATUS_OFFLINE;
            // Manage reconnection after losing signal
            if (allow_to_reconnect){
                esp_wifi_connect();
            }
            break;

        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI("WIFI", "WIFI got IP address");
            current_status_code = STATUS_ONLINE;
            wifi_connected_flag = true; 
            break;

        default:
            printf("An unrecognized WiFi event with id %ld occured\n", event_id);
            break;
    }
}


void WIFI_Connect(void *pvParameter){
    esp_netif_init();
    esp_event_loop_create_default(); 
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    // Event handlers 
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
    esp_wifi_connect();

    // Block this task to avoid returning since this triggers RTOS error
    vTaskDelete(NULL);
}

// Simulated random float generator between min and max
float randf(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

// Simulated TelemetryData population
void generate_random_telemetry(TelemetryData *data) {
    data->battery_voltage = randf(20.0f, 28.0f);  // volts
    data->current_amps    = randf(0.0f, 10.0f);   // amps
    data->latitude        = randf(-90.0f, 90.0f); // degrees
    data->longitude       = randf(-180.0f, 180.0f);
    data->accel_x         = randf(-50.0f, 50.0f); // m/s^2
    data->accel_y         = randf(-50.0f, 50.0f);
    data->accel_z         = randf(-50.0f, 50.0f);
    data->orient_x        = randf(0.0f, 360.0f);  // degrees
    data->orient_y        = randf(0.0f, 360.0f);
    data->orient_z        = randf(0.0f, 360.0f);
    data->rpms            = rand() % 10000;       // 0–9999
    data->velocity_x      = randf(-20.0f, 20.0f);  // m/s
    data->velocity_y      = randf(-20.0f, 20.0f);
    data->ambient_temp    = randf(-10.0f, 50.0f);  // °C
}

void post_data(void *pvParameter) {
    while(true){
        if(WIFI_transmit_enable){
            TelemetryData *data = (TelemetryData *)pvParameter;
            generate_random_telemetry(data);

            esp_http_client_config_t config = {
                .url = "http://274db8d0d3ce.ngrok-free.app/api/lectures" ,
            };

            esp_http_client_handle_t client = esp_http_client_init(&config);

            char post_data[512];
            snprintf(post_data, sizeof(post_data), 
            "{\"voltage_battery\": %.2f, \"current\": %.2f, \"latitude\": %.4f, \"longitude\": %.4f, \"acceleration_x\": %.2f, \"acceleration_y\": %.2f, \"acceleration_z\": %.2f, \"orientation_x\": %.2f, \"orientation_y\": %.2f, \"orientation_z\": %.2f, \"rpm_motor\": %d, \"velocity_x\": %.2f, \"velocity_y\": %.2f, \"ambient_temp\": %.2f, \"steering_angle\": 0.0}",
            data->battery_voltage, data->current_amps, data->latitude, data->longitude, data->accel_x, data->accel_y, data->accel_z, data->orient_x, data->orient_y, data->orient_z, data->rpms, data->velocity_x, data->velocity_y, data->ambient_temp);
            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_post_field(client, post_data, strlen(post_data));
            esp_http_client_set_header(client, "Content-Type", "application/json");

            esp_err_t err = esp_http_client_perform(client);

            esp_http_client_cleanup(client);
            printf("Data was posted\n");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CONNECTION TEC

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI("WIFI", "WIFI started, connecting...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI("WIFI", "Connected to WPA2 Enterprise network");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW("WIFI", "Disconnected. Reconnecting...");
                wifi_connected_flag = false;
                esp_wifi_connect();  // Optional: auto-reconnect
                break;

            default:
                ESP_LOGW("WIFI", "Unhandled event id: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        esp_netif_ip_info_t ip_info = event->ip_info;

        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&ip_info.ip));
        wifi_connected_flag = true;

        ESP_LOGI("WIFI", "Got IP: %s", ip_address);
    }
}


void wifi_init_enterprise(void *pvParameter) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL);


    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, "Tec", sizeof(wifi_config.sta.ssid));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // WPA2 Enterprise configuration
    esp_wifi_sta_wpa2_ent_enable();  // Enable WPA2 Enterprise
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)"A00344893", strlen("A00344893"));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)"A00344893", strlen("A00344893"));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)"", strlen(""));

    esp_wifi_start();
    esp_wifi_connect();

    while (!wifi_connected_flag) {
        ESP_LOGI("WIFI", "Waiting for IP...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Block this task to avoid returning s ince this triggers RTOS error
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}