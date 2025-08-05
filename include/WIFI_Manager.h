#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_netif_ip_addr.h"
#include "nvs_flash.h"
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps
#include "esp_log.h"
#include "esp_wpa2.h"
#include "Globals.h"

///////// WIFI SETTINGS /////////
extern const char *ssid;
extern const char *pass;
extern bool wifi_connected_flag;
extern const bool allow_to_reconnect;

extern char ip_address[20];
/////////////////////////////////

void NVS_Init(void);
void WIFI_Event_Handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
void post_data(void *);
void WIFI_Connect(void *pvParameter);
void wifi_init_enterprise(void *pvParameter);
