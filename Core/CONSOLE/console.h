#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h> //for va_list var arg functions
#include <stdbool.h>

#define RX_BUFFER_SIZE 1024u
#define TX_BUFFER_SIZE 1024u

typedef struct
{
    bool is_dma;
    uint8_t rx;
    uint32_t rx_ptr;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];
} CONSOLE_DATA_T;

/**
 * @brief Custom console log information function using UART
 *
 * @param fmt arguments
 * @param ...
 */
void log_info(const char* fmt, ...);

/**
 * @brief Custom console log warning function using UART
 *
 * @param fmt arguments
 * @param ...
 */
void log_warning(const char* fmt, ...);

/**
 * @brief Custom console log error function using UART
 *
 * @param fmt arguments
 * @param ...
 */
void log_error(const char* fmt, ...);

void console_task_entry(void);

#endif // !CONSOLE_H
