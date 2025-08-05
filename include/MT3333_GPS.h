#ifndef MT3333_GPS 
#define MT3333_GPS 

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/uart.h"

// Return options for PMTK commands
#define COMMAND_OK     0
#define COMMAND_ERROR  -1 

// PMTK Command String Buffer
extern char PMTK_command_buff[256];

// NMEA RX String Buffer 
extern char NMEA_rx_buff[256];

// Auxiliar buffer 
extern char aux_buff[128];

// NMEA sentence indexes
enum nmea_sentence_index {
    NMEA_SEN_GLL,
    NMEA_SEN_RMC,
    NMEA_SEN_VTG, 
    NMEA_SEN_GGA,
    NMEA_SEN_GSA,
    NMEA_SEN_GSV,
    NMEA_SEN_ZDA, 
    NMEA_SEN_MCHN,
    NMEA_SEN_NONE 
};

// Output frequency values
enum output_frequencies {
    OUTPUT_DISABLED,
    OUTPUT_ONE_FIX_CYCLE,
    OUTPUT_TWO_FIX_CYCLE,
    OUTPUT_THREE_FIX_CYCLE, 
    OUTPUT_FOUR_FIX_CYCLE,
    OUTPUT_FIVE_FIX_CYCLE
};

// Auxiliary struct for NMEA sentence frequency selection
typedef struct {
    uint8_t GPGLL_freq;
    uint8_t GPRMC_freq;
    uint8_t GPVTG_freq;
    uint8_t GPGGA_freq;
    uint8_t GPGSA_freq;
    uint8_t GPGSV_freq;
    uint8_t GPZDA_freq;
    uint8_t GPMCHN_freq;
} NMEA_config_struct;

extern NMEA_config_struct NMEA_cfg;

// UART Instance GPS 
extern int UART_instance_GPS;

// GPS standby flag
extern bool is_GPS_standby;

////////////// FUNCTIONS //////////////

void GPS_Init(int);
void set_UART_Instance(int);
void send_GPS_Command(void);

int set_GPS_Update_Rate(uint8_t);
int set_GPS_Standby(void);
int set_Satellite_System(uint8_t);
int set_Output_Frequency(uint8_t, uint8_t);

int get_GPS_Firmware_Version(void);
int get_GPS_Update_Rate(void);
int get_UTC_Time(void);

void generate_nmea_sentence(const char *payload, char *output, size_t output_size);

//////////////////////////////////////

#endif /* MT3333_GPS */