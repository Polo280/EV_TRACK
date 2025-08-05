#ifndef GLOBALS_H
#define GLOBALS_H

/////////////////////////////////////////////////////////
///////////////////// BOARD PINOUT //////////////////////
/////////////////////////////////////////////////////////

// Status RGB LED
#define RED_LED_GPIO           GPIO_NUM_2    
#define GREEN_LED_GPIO         GPIO_NUM_0
#define BLUE_LED_GPIO          GPIO_NUM_4

// GPS UART 
#define GPS_UART_CHANNEL       UART_NUM_2
#define GPS_RX_GPIO            GPIO_NUM_16   
#define GPS_TX_GPIO            GPIO_NUM_17

// ADC Measurements

#define VBAT_ADC_PIN GPIO_NUM_32
 
#define TRANSMIT_ENABLE_GPIO   GPIO_NUM_15   // Enable WIFI data transmission push button pin

//////////////////////////////////////////////////////////

// PLATFORM STATUS CODES
typedef enum {
    STATUS_ONLINE,
    STATUS_OFFLINE,
    STATUS_TRANSMITTING,
    STATUS_ERROR
} STATUS_CODES;

extern uint8_t current_status_code;
extern bool WIFI_transmit_enable;

// TELEMETRY DATA TEMPLATE
typedef struct {
    float battery_voltage;
    float current_amps;
    double latitude;
    double longitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float orient_x;
    float orient_y;
    float orient_z;
    uint16_t rpms;
    float velocity_x;
    float velocity_y;
    float ambient_temp;
} TelemetryData;

#endif /* GLOBALS_H */