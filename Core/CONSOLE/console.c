#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"
#include "console.h"
#include "define.h"
#include "main.h"
#include "queue.h"
#include "timer.h"

extern UART_HandleTypeDef USART_HANDLE;
static CONSOLE_DATA_T console_data;

/* Serial color code */
#define YELLOW_CODE "\033[33m"
#define RED_CODE    "\033[91m"

/* Log information special character */
#define END_LINE      "\n\r"
#define WARNING_COLOR YELLOW_CODE
#define ERROR_COLOR   RED_CODE
#define COLOR_RESET   "\033[0m"

/* Log type defines */
typedef enum
{
    LOG_INFO = 0u,
    LOG_WARNING,
    LOG_ERROR,
} LOG_TYPE;

static void log_transmit(LOG_TYPE type, uint8_t* pData, uint32_t length)
{
    uint8_t tx_buffer[length];
    memcpy(tx_buffer, pData, length); // Utilize UART DMA for fastest transmit
    if (type == LOG_WARNING)
    {
        HAL_UART_Transmit(&huart2, WARNING_COLOR, sizeof(WARNING_COLOR), -1);
    }
    else if (type == LOG_ERROR)
    {
        HAL_UART_Transmit(&huart2, ERROR_COLOR, sizeof(ERROR_COLOR), -1);
    }
    else
    {
        // Keep it white text!
    }

    HAL_UART_Transmit(&huart2, tx_buffer, length, -1);
    HAL_UART_Transmit(&huart2, COLOR_RESET, sizeof(COLOR_RESET), -1);
    HAL_UART_Transmit(&huart2, END_LINE, sizeof(END_LINE), -1);
}

void log_info(const char* fmt, ...)
{
    static char buffer[150u];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uint32_t len = strlen(buffer);
    log_transmit(LOG_INFO, buffer, len);
}

void log_warning(const char* fmt, ...)
{
#ifdef LOGGER_DEBUG
    static char buffer[150u];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    log_transmit(LOG_WARNING, buffer, len);
#endif
}

void log_error(const char* fmt, ...)
{
#ifdef LOGGER_ERROR
    static char buffer[150u];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    log_transmit(LOG_ERROR, buffer, len);
#endif
}

static bool console_init(void)
{
    // log_info("LOGGER CONSOLE");
    memset(&console_data, 0, sizeof(console_data));
    HAL_UART_Receive_IT(&USART_HANDLE, &console_data.rx, sizeof(console_data.rx));
    return true;
}

static void console_enq_data(const uint32_t data)
{
    if (console_data.rx_ptr < RX_BUFFER_SIZE)
    {
        console_data.rx_buffer[console_data.rx_ptr++] = data;
    }
    else
    {
        // Reset buffer
        console_data.rx_ptr = 0u;
    }
}

static void console_proc(void)
{
    static uint32_t tx_timer = 0u;
    static uint32_t prev_ptr = 0u;

    if (!console_data.is_dma)
    {
        if (prev_ptr != console_data.rx_ptr)
        {
            uint32_t length = console_data.rx_ptr - prev_ptr;
#ifdef CONSOLE_ECHO
            memcpy(&console_data.tx_buffer, &console_data.rx_buffer[prev_ptr], length);
            HAL_UART_Transmit_DMA(&USART_HANDLE, &console_data.tx_buffer, length);
            console_data.is_dma = true;
#endif /* CONSOLE_ECHO */
            prev_ptr = console_data.rx_ptr;
        }
        else
        {
            // Do nothing
        }
    }
}

static void console_task(void* arg)
{
    console_init();
    while (true)
    {
        console_proc();
        task_delay(50u); // 50ms so that all characters are received
    }
}

void console_task_entry(void)
{
    BaseType_t xReturned;
    TaskHandle_t console_handle = NULL;
    xReturned = xTaskCreate(console_task, "console", CONSOLE_STACK, NULL, CONSOLE_PRIORITY, &console_handle);

    // if (xReturned != pdPASS)
    // {
    //     log_info("Console task initialization failed");
    // }
    // else
    // {
    //     log_info("Console task initialization successfully");
    // }

    int my_variable = 0x12345678;
    int* my_pointer = &my_variable;
    log_info("%d", my_pointer);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    console_enq_data(console_data.rx);
    HAL_UART_Receive_IT(&USART_HANDLE, &console_data.rx, sizeof(console_data.rx));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    console_data.is_dma = false;
}
