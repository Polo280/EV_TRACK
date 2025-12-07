#include "MT3333_GPS.h"

char PMTK_command_buff[256];
char aux_buff[128];
int UART_instance_GPS = 0;
bool is_GPS_standby = false;

// NMEA sentence frequency config struct (by default is started with these values)
NMEA_config_struct NMEA_cfg = {
    .GPGLL_freq = OUTPUT_DISABLED,
    .GPRMC_freq = OUTPUT_ONE_FIX_CYCLE,
    .GPVTG_freq = OUTPUT_DISABLED,
    .GPGGA_freq = OUTPUT_DISABLED,
    .GPGSA_freq = OUTPUT_DISABLED,
    .GPGSV_freq = OUTPUT_DISABLED,
    .GPZDA_freq = OUTPUT_DISABLED,
    .GPMCHN_freq = OUTPUT_DISABLED
};


/**
 * @brief Starts the GPS with the given UART instance and the default configurations
 * 
 * @param uart_num: UART instance that the GPS will use 
 * 
 * @return None 
 * 
 */
void GPS_Init(int uart_num){
    set_UART_Instance(uart_num);
    // Set the GPS update rate 
    set_GPS_Update_Rate(GPS_UPDATE_1HZ);
    // Start with default frequency values
    set_Output_Frequency(NMEA_SEN_NONE, OUTPUT_DISABLED);
    // Set baud rate (test)
    set_GPS_Baud_Rate(GPS_BAUD_RATE_9600);
}


void set_UART_Instance(int uart_num){ 
    strcpy(PMTK_command_buff, "");
    UART_instance_GPS = uart_num;
}


void send_GPS_Command(void){
    uart_write_bytes(UART_instance_GPS, (const char*)PMTK_command_buff, strlen(PMTK_command_buff));
}


/////////////// COMMANDS ///////////////

/**
 * @brief Sets GPS update rate.
 *
 * This function configures the MT3333 GPS module to output NMEA sentences
 * at a specific update rate based on the selected index.
 * - 0: 1 Hz
 * - 1: 5 Hz
 * - 2: 10 Hz
 *
 * @param rate_index Index representing the desired update rate:
 * - 0: 1 Hz
 * - 1: 5 Hz
 * - 2: 10 Hz
 *
 * @return 0 if index is valid, -1 if index is not valid
 */
int set_GPS_Update_Rate(uint8_t rate_index){
    bool send_command = true; 
    switch (rate_index)
    {
    case GPS_UPDATE_1HZ:
        // 1 Hz Updates
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK220,1000");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_UPDATE_5HZ:
        // 5 Hz Updates
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK220,200");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_UPDATE_10HZ:
        // 10 Hz Updates
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK220,100");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    default:
        send_command = false;
        break;
    }

    // Validate command 
    if(!send_command){
        return COMMAND_ERROR;
    }

    send_GPS_Command();
    return COMMAND_OK;
}

/**
 * @brief Select baud rate for GPS communication
 * 
 * @param rate_index: Baud Rate Index (GPS baud rate enum values)
 * 
 * @return COMMAND_OK if a valid baud rate was selected, COMMAND_ERROR if not
 * 
 */
int set_GPS_Baud_Rate(uint8_t rate_index){
    bool send_command = true;
    switch (rate_index)
    {
    case GPS_BAUD_RATE_4800:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,4800");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_9600:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,9600");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_14400:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,14400");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_19200:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,19200");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_38400:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,38400");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_57600:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,57600");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    case GPS_BAUD_RATE_115200:
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK251,115200");
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        break;
    default:
        send_command = false;
        break;
    }

    // Validate command 
    if(!send_command){
        return COMMAND_ERROR;
    }

    send_GPS_Command();
    return COMMAND_OK;
}


/**
 * @brief Getter for current GPS update rate 
 * 
 * @note 1000 = 1 Hz, 200 = 5 Hz, 100 = 10 Hz
 */
int get_GPS_Update_Rate(void){
    snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "$PMTK400*36\r\n");
    send_GPS_Command();
    return COMMAND_OK;
}


/**
 * @brief Get current UTC time from RTC 
 * 
 * @param None
 * 
 * @note After the command is sent, a rewturn PMTK package is transmitted from the GPS,
 * the return command is given in the following format: PMTK535,Year,Month,Day,Hour,Min,Sec
 * 
 */
int get_UTC_Time(void){
    snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "$PMTK435*30\r\n");
    send_GPS_Command();
    return COMMAND_OK;
}


/**
 * @brief Sets GPS in standby mode for power saving. 
 *
 * @param None
 *
 * @return COMMAND_OK if GPS got turned off, COMMAND_ERROR if GPS was already standby.
 */
int set_GPS_Standby(void){
    if(!is_GPS_standby){
        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "$PMTK161,0*28\r\n");
        send_GPS_Command();
        is_GPS_standby = true;
        return COMMAND_OK;
    }
    return COMMAND_ERROR;
}


/**
 * @brief Select output frequencies for each NMEA sentence through this command
 * 
 * @param sentence_selection: Integer representing which NMEA sentence is being configured
 * @param freq_selection: Integer representing how frequently is printed, use values in output_frequencies enum
 * 
 * @return COMMAND_OK
 */
int set_Output_Frequency(uint8_t sentence_selection, uint8_t freq_selection){
    if(sentence_selection > NMEA_SEN_NONE || freq_selection > OUTPUT_FIVE_FIX_CYCLE){
        return COMMAND_ERROR;
    }
    // Update the NMEA sentence struct 
    switch(sentence_selection){
        case NMEA_SEN_GLL:
            NMEA_cfg.GPGLL_freq = freq_selection;
            break;
        case NMEA_SEN_RMC:
            NMEA_cfg.GPRMC_freq = freq_selection;
            break;
        case NMEA_SEN_VTG:
            NMEA_cfg.GPVTG_freq = freq_selection;
            break;
        case NMEA_SEN_GGA:
            NMEA_cfg.GPGGA_freq = freq_selection;
            break;
        case NMEA_SEN_GSA:
            NMEA_cfg.GPGSA_freq = freq_selection;
            break;
        case NMEA_SEN_GSV:
            NMEA_cfg.GPGSV_freq = freq_selection;
            break;
        case NMEA_SEN_ZDA:
            NMEA_cfg.GPZDA_freq = freq_selection;
            break;
        case NMEA_SEN_MCHN:
            NMEA_cfg.GPMCHN_freq = freq_selection;
            break;
        case NMEA_SEN_NONE:
            break;
        default:
            return COMMAND_ERROR;
    }
    // Format and send command
    snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK314,%d,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,%d,%d",
    NMEA_cfg.GPGLL_freq, NMEA_cfg.GPRMC_freq, NMEA_cfg.GPVTG_freq, NMEA_cfg.GPGGA_freq, NMEA_cfg.GPGSA_freq, NMEA_cfg.GPGSV_freq,
    NMEA_cfg.GPZDA_freq, NMEA_cfg.GPMCHN_freq);
    generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
    strcpy(PMTK_command_buff, aux_buff);
    send_GPS_Command();
    return COMMAND_OK;
}


/**
 * @brief Selects a satellite system to perform GPS triangulation 
 * 
 * @param gp_index: Selection index ranging from 0 - 31
 * 
 * @note Given selection index is decomposed into binary to enable/disable following options 
 * MSB - GPS_Enable (USA)
 * 2   - GLONASS Enable (RUSSIA)
 * 3   - Galileo_Enable (European just search)
 * 4   - Galileo_Full_Enable (European perform fixes also)
 * LSB - Beidou Enable (CHINA)
 * 
 * @return COMMAND_OK if selection index is valid, COMMAND_ERROR if invalid
 */
int set_Satellite_System(uint8_t gp_index){
    if(gp_index <= 31){
        bool gps_en          = (gp_index >> 4) & 0x01;  
        bool glonass_en      = (gp_index >> 3) & 0x01;
        bool galileo_en      = (gp_index >> 2) & 0x01;
        // bool galileo_full_en = (gp_index >> 1) & 0x01;
        bool beidou_en       = (gp_index) & 0x01;

        snprintf(PMTK_command_buff, sizeof(PMTK_command_buff), "PMTK353,%d,%d,%d,0,%d", gps_en? 1 : 0, glonass_en? 1 : 0, galileo_en? 1 : 0, beidou_en? 1 : 0);
        generate_nmea_sentence(PMTK_command_buff, aux_buff, sizeof(aux_buff));
        strcpy(PMTK_command_buff, aux_buff);
        send_GPS_Command();
        return COMMAND_OK;
    }
    return COMMAND_ERROR;
}


/**
 * @brief Generate configuration sentence with the checksum 
 */
void generate_nmea_sentence(const char *payload, char *output, size_t output_size) {
    uint8_t checksum = 0;

    // Compute checksum (XOR of all characters in payload)
    for (size_t i = 0; i < strlen(payload); ++i) {
        checksum ^= (uint8_t)payload[i];
    }

    // Format: $ + payload + * + 2-digit checksum + CRLF
    snprintf(output, output_size, "$%s*%02X\r\n", payload, checksum);
}


/**
 * @brief Gets the current firmware version of the GPS module 
 * 
 * @param None 
 * 
 */
int get_GPS_Firmware_Version(void){
    snprintf(PMTK_command_buff, sizeof( PMTK_command_buff), "$PMTK605*31\r\n");
    send_GPS_Command();
    return COMMAND_OK;
}