#include "logger_app.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"
#include "console.h"
#include "define.h"
#include "gpio_config.h"
#include "sd.h"
#include "spi_fram.h"
#include "timer.h"

static LOGGER_DATA_T logger_data;

static const LOGGER_DATA_T initial_logger_data = 
{
    .state = INITIALIZATION,
    .ram_addr = 0u,
    .rx_buffer = {0u},
    .config = 
    {
        .usb_ms = false,
        .mode = LINEAR_BUFFER,    
        .baudrate = 115200u,
    },
};

/*Blink LEDs when system receive error based on error type*/
static void system_error(LOGGER_ERROR_T Error_type)
{
    while (true)
    {
        for (uint8_t index = 0; index < Error_type; index++)
        {
            LED_TOGGLE;
            HAL_Delay(200u);
        }
        HAL_Delay(2000u);
    }
}

static void data_received(bool is_received)
{
    if (is_received)
    {
        logger_data.state = FRAM_PROCESS;
    }
    else
    {
        // Not sure yet
    }
}

static bool logger_app_init(void)
{
    bool ret = true;

    memset(&logger_data, &initial_logger_data, sizeof(initial_logger_data));

    if (fram_init())
    {
        log_info("SD Logger by PL v.%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR);
        log_info("Current RAM addr: %d\r\n", logger_data.ram_addr);
    }
    else
    {
        log_info("SPI FRAM initialization failed.\r\n");
    }

    return true;
}

static void logger_app_proc(void)
{
    switch (logger_data.state)
    {
        case INITIALIZATION:
            /* Should not be in here */
            break;
        case FRAM_PROCESS:

        default:
            break;
    }
}

static void logger_app_task(void* arg)
{
    while (true)
    {
        task_delay(1u);
    }
}

void logger_app_task_entry(void)
{
    BaseType_t xReturned;
    TaskHandle_t logger_app_handle = NULL;
    xReturned = xTaskCreate(logger_app_task, "main task", LOGGER_APP_STACK, NULL, LOGGER_APP_PRIORITY, &logger_app_handle);

    if (xReturned != pdPASS)
    {
        log_info("logger main task initialization failed\r\n");
    }
}
