#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>

/**
 * @brief Get elapses (delta ms) time from xTaskGetTickCount
 *
 * @param timer timer from a module
 * @return uint32_t elapsed time
 */
uint32_t timer_get_elapsed_time(const uint32_t timer);

/**
 * @brief Reset timer (Update to current timestamp based on systick)
 *
 * @param timestamp timestamp of module
 */
void timer_reset(uint32_t* timestamp);

#endif // !_TIMER_H_
