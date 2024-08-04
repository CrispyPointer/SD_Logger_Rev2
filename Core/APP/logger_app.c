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
#include "main.h"
#include "queue.h"
#include "sd.h"
#include "spi_fram.h"
#include "timer.h"

static LOGGER_DATA_T logger_data;

static QueueHandle_t fram_queue;

static const LOGGER_DATA_T default_logger_data = 
{
    .state = INITIALIZATION,
    .addr = 0u,
    .buffer = {0u, 1u},
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

static void logger_app_init(void)
{
    memcpy(&logger_data, &default_logger_data, sizeof(logger_data));
    logger_data.state = WAITING;
}

static void logger_input_proc(void)
{
    if (logger_data.addr == (BUFFER_MAX - 1u))
    {
        logger_data.addr = 0u;
        if (fram_queue != 0)
        {
            if (xQueueSend(fram_queue, &logger_data.buffer, (TickType_t)10) != pdPASS)
            {
                log_info("Failed to send queue");
            }
            else
            {
                // memset(&logger_data.buffer, 0u, sizeof(logger_data.buffer));
            }
        }
        else
        {
            log_info("Unable to create queue");
        }
    }

    LED_OFF;
    logger_data.state = WAITING;
}

static void logger_app_proc(void)
{
    switch (logger_data.state)
    {
        case INITIALIZATION:
            /* Should not be in here */
            break;
        case WAITING:
            task_delay(5u); // wait until other task done process
            break;
        case RX_PROCESS:
            logger_input_proc();
            break;
        default:
            break;
    }
}

static void fram_proc(void)
{
    FRAM_STATE_T fram_state = fram_get_state();
    static uint32_t fram_index = 0u;
    static LOGGER_BUFFER_T fram_buffer = { 0u };

    if (fram_state == FRAM_ERROR)
    {
        log_info("FRAM error: ");
    }
    else if (fram_state == FRAM_IDLE)
    {
        if (uxQueueMessagesWaiting(fram_queue) != 0u)
        {
            fram_index = fram_get_index();
            if (xQueueReceive(fram_queue, &fram_buffer, (TickType_t)50u) == pdPASS)
            {
                if (fram_write(fram_index, fram_buffer.data, fram_buffer.size) != true)
                {
                    log_info("FRAM write error: ");
                }
                else
                {
                    fram_set_state(FRAM_BUSY);
                }
            }
        }
    }
    else
    {
        // FRAM is processing? Leave it
    }
}

static void fram_receive(void)
{
    static uint8_t read_buffer[120u];
    uint32_t fram_index = fram_get_index();
    if (fram_get_state() == FRAM_DONE_READ)
    {
        HAL_UART_Transmit_DMA(&USART_HANDLE, read_buffer, sizeof(read_buffer));
        fram_set_state(FRAM_IDLE);
    }
    if (fram_index >= 120u)
    {
        log_warning("fram_index: %d", fram_index);
        fram_set_index(0u);
        if (fram_read(0u, read_buffer, sizeof(read_buffer)) != true)
        {
            log_info("FRAM read error: ");
        }
        else
        {
            fram_set_state(FRAM_BUSY);
        }
    }
}

static void fram_task(void* argument)
{
    if (!fram_init())
    {
        log_info("FRAM initialization failed.");
    }
    task_delay(5u); // wait until other task done initialization

    while (true)
    {
        fram_proc();
        fram_receive();
    }
}

static void logger_app_task(void* arg)
{
    logger_app_init();
    fram_queue = xQueueCreate(10u, sizeof(logger_data.buffer));

    while (true)
    {
        logger_app_proc();
    }
}

void logger_app_task_entry(void)
{
    BaseType_t xReturned;
    TaskHandle_t logger_app_handle = NULL;
    TaskHandle_t fram_handle = NULL;

    xReturned = xTaskCreate(logger_app_task, "main task", LOGGER_APP_STACK, NULL, LOGGER_APP_PRIORITY, &logger_app_handle);

    if (xReturned != pdPASS)
    {
        log_info("Logger main task initialization failed");
    }
    else
    {
        log_info("Logger main task initialization successfully");
    }

    xReturned = xTaskCreate(fram_task, "fram task", FRAM_STACK, NULL, FRAM_PRIORITY, &fram_handle);
    if (xReturned != pdPASS)
    {
        log_info("Fram task initialization failed");
    }
    else
    {
        log_info("Fram task initialization successfully");
    }
}
