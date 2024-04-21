#include "timer.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

uint32_t timer_get_elapsed_time(const uint32_t timer)
{
    const TickType_t current_timestamp = xTaskGetTickCount();

    return timer >= 0u ? current_timestamp - timer : 0u;
}

void timer_reset(uint32_t* timestamp)
{
    *timestamp = xTaskGetTickCount();
}
