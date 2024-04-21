#include <stdbool.h>
#include <stdint.h>

#include "console.h"
#include "fatfs.h"
#include "gpio_config.h"
#include "main.h"
#include "sd.h"

// Some variables for FatFs
static FATFS FatFs;                   // Fatfs handle
static FIL newFile, workingFile, fil; // File handle
static char FATPATH[4];

void sd_power_on(void)
{
    SD_PWR_ON;
}

void sd_power_off(void)
{
    HAL_SPI_DeInit(&hspi2);
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /*Configure SPI pins Output Level */
    SPI_SD_OFF;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Power down SD module */
    SD_PWR_OFF;
}

bool sd_init(void)
{
    bool ret = true;

    FRESULT fres;

    fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now

    // TODO: Look for a safe way to do this
    // while (fres != FR_OK) {
    //     if (FATFS_UnLinkDriver(&USERPath)) {
    //         // NVIC_SystemReset();
    //     }
    //     else
    //         sd_power_off();
    //     HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
    //     HAL_Delay(500);

    //     /*Re-init SD*/
    //     HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
    //     HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin);
    //     MX_GPIO_Init();
    //     MX_SPI2_Init();
    //     MX_FATFS_Init();
    //     HAL_Delay(500);
    //     fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now
    // }

    if (fres == FR_OK)
    {
        log_info("SD mounted\r\n");
        ret = true;
    }

    return ret;
}