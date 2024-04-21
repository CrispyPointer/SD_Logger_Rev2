#include "timer.h"

uint32_t timer_get_elapsed_time(const uint32_t timer)
{
    const uint32_t current_timestamp = (uint32_t)HAL_GetTick();

    return timer >= 0u ? current_timestamp - timer : 0u;
}

void timer_reset(uint32_t* timestamp)
{
    *timestamp = HAL_GetTick();
}
