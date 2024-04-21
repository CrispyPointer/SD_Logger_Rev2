#ifndef _LOGGER_APP_H_
#define _LOGGER_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "define.h"
#include "main.h"

/**< LOGGER STATUS & ERROR */
typedef enum
{
    INITIALIZATION = 0u,
    CONFIG_PROCESS,
    IDLE,
    FRAM_PROCESS,
    SD_PROCESS,
    USB_MS,
} LOGGER_STATE_T;

typedef enum
{
    ERROR_SD_INIT = 2u, // blink LED 2 times a second if SD card is mounted fail.
    SD_ERROR,
    FRAM_ERROR,
    LOG_RESET,
    CONFIG_ERROR
} LOGGER_ERROR_T;

typedef enum
{
    LINEAR_BUFFER = 0u,
    RING_BUFFER,
} LOGGER_MODE_T;

typedef struct
{
    bool usb_ms; /**< USB mass storage function */
    uint32_t baudrate;
    LOGGER_MODE_T mode;
} LOGGER_CONFIG_T;

typedef struct
{
    LOGGER_STATE_T state;
    LOGGER_CONFIG_T config;
    uint32_t ram_addr; /**< Address of the current position of FRAM */
    uint8_t rx_buffer[2048u];
    uint32_t timer; /**< logger timer */
} LOGGER_DATA_T;

void logger_app_task_entry(void);

#endif // !_LOGGER_APP_H_
