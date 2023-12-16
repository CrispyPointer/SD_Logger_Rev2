#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h> //for va_list var arg functions

/**
 * @brief Custom console log function using UART 
 * 
 * @param fmt arguments
 * @param ... 
 */
void console_log(const char* fmt, ...);

#endif // !CONSOLE_H
