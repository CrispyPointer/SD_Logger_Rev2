#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h> //for va_list var arg functions

/**
 * @brief Custom console log function using UART
 *
 * @param fmt arguments
 * @param ...
 */
void log_info(const char* fmt, ...);

void console_task_entry(void);

#endif // !CONSOLE_H
