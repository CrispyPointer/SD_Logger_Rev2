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
#include "timer.h"

void log_info(const char* fmt, ...)
{
    static char buffer[500];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}

static void console_task(void* arg)
{
    log_info("LOGGER CONSOLE\r\n");

    while (true)
    {
        task_delay(1u);
    }
}

void console_task_entry(void)
{
    BaseType_t xReturned;
    TaskHandle_t console_handle = NULL;
    xReturned = xTaskCreate(console_task, "console", CONSOLE_STACK, NULL, CONSOLE_PRIORITY, &console_handle);

    if (xReturned != pdPASS)
    {
        log_info("Console task initialization failed\r\n");
    }
}