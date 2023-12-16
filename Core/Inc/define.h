#ifndef DEFINE_H
#define DEFINE_H

#define VERSION     "v2.0"
#define CONFIG_FILE "config.txt"

#define DEBUG
// #define LOGGER_RESET
#ifdef DEBUG
    #warning "You are running a debug build!"
#endif

/*************************************************FRAM ADDRESS CONFIG*****************************************************/
// External FRAM locations for user settings
#define DEVICE_ID_LOCATION 0x02 // Each logger will be assigned a unique ID

#define LOCATION_FILE_NUMBER_LSB 0x03 // 16-bit value LSB for file location number
#define LOCATION_FILE_NUMBER_MSB 0x04 // 16-bit value MSB for file location number

#define BAUD_LSB_LOCATION 0x05 // 16-bit value LSB for Logger baudrate
#define BAUD_MSB_LOCATION 0x06 // 16-bit value MSB for Logger baudrate

#define LOGGER_STAT_LOCATION 0x07 // Logger status location

/*---------------24 bit value of the FRAM Iterator address from the last logging ----------------*/
#define SYNC_BUFFER_MSB  0x08 // First 8 MSB of the Iterator value
#define SYNC_BUFFER_MSB2 0x09 // Second 8 bits of the Iterator
#define SYNC_BUFFER_LSB  0x0A // Last 8 LSB of the Iterator

/*-----------------------------------------------------------------------------------------------*/
// #define LOCATION_BAUD_SETTING_HIGH 0x0B
// #define LOCATION_BAUD_SETTING_MID 0x0C
// #define LOCATION_BAUD_SETTING_LOW 0x0D

// #define LOCATION_LOGGER_RESTORE 0x0E

#define LOCATION_BUFFER_START 20u      // Address of start position when buffering
#define LOCATION_BUFFER_END   0x3ff70u // Address of end position when buffering (260000)

/*************************************************USER DEFINE BUFFER SIZE, ITERATOR LOCAION AND TIMEOUT**********************************/
#ifdef DEBUG
#define BUFFER_MAX 100u // User define max buffer 100 characters for debugging
#define MAX_IDLE_TIME_MSEC UINT32_MAX // User define timeout before going low power
#else
#define BUFFER_MAX 30000 // User define max buffer from 20 to 262000
#define MAX_IDLE_TIME_MSEC 5000 // User define timeout before going low power
#endif



#endif /* DEFINE_H */
