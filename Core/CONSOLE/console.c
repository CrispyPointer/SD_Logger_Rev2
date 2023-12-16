#include <stdio.h>
#include <string.h>

#include "console.h"
#include "main.h"
#include "define.h"

void console_log(const char* fmt, ...)
{
    static char buffer[500];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}
