#ifndef _SD_H_
#define _SD_H_

#include <stdbool.h>
#include "fatfs.h"

bool sd_init(FATFS* hfat);

/*!
 *  @brief  Turn off SD NAND chip by disable 3V3 power supply and put the GPIOs to analog mode
 *  @param  None
 *  @retval None
 */
void sd_power_off(void);

/*!
 *  @brief  Turn on SD NAND chip by enable 3V3 power supply
 *  @param  None
 *  @retval None
 */
void sd_power_on(void);

#endif // !_SD_H_
