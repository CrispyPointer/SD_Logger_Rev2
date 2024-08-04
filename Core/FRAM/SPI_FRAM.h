/*!
 *  @file   SPI_FRAM.h
 *  @brief  This is a redistribution of the SPI_FRAM chip driver implemented on STM32L412xx MCU using HAL Driver.
 *  Phuoc K. Ly (SODAQ)
 ******************************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Portions copyright (c) 2013, Adafruit Industries, All rights reserved.
 *  Portioins copyright (c) 2022, phuocly2304, All rights reserved
 *
 *  This software is a free software and there is NO WARRANTY.
 *  No restriction on use. You can use, modify and redistribute it for
 *  personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 *  Redistributions of source code must retain the above copyright notice.
 */

/*
    This code was ported by Phuoc K. Ly (SODAQ) from a copywrited (C) library written by K.Townsend (Adafruit Industries)
    available at https://github.com/adafruit/Adafruit_FRAM_SPI
*/

#ifndef _SPI_FRAM_H_
#define _SPI_FRAM_H_

#include <stdbool.h>
#include <string.h>

/** Operation Codes **/
typedef enum
{
    OPCODE_WRSR = 1u,    /* Write Status Register D1*/
    OPCODE_WRITE,        /* Write Memory D2*/
    OPCODE_READ,         /* Read Memory D3*/
    OPCODE_WRDI,         /* Reset Write Enable Latch D4*/
    OPCODE_RDSR,         /* Read Status Register D5*/
    OPCODE_WREN,         /* Write Enable Latch D6*/
    OPCODE_RDID = 159u,  /* Read Device ID D159*/
    OPCODE_SLEEP = 185u, /*Sleep Mode 185 apply to MB85RS2MTA chip*/
} opcodes_t;

typedef enum
{
    FRAM_IDLE = 0u,
    FRAM_BUSY,
    FRAM_ERROR,
    FRAM_DONE_READ,
} FRAM_STATE_T;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          FRAM SPI
 */

typedef struct FRAM_DATA_T
{
    uint8_t _nAddressSizeBytes;
    FRAM_STATE_T state;
    uint32_t current_index;
    uint32_t error;
} FRAM_DATA_T;

/**
 *  @brief  Initializes SPI and configures the chip (call this function before
 *          doing anything else)
 *  @return true if successful
 */
bool fram_init(void);

bool fram_write(const uint32_t addr, const uint8_t* pData, uint8_t pLength);

bool fram_read(const uint32_t addr, uint8_t* pData, uint8_t pLength);

uint8_t getStatusRegister(void);

FRAM_STATE_T fram_get_state(void);

uint32_t fram_get_index(void);

void fram_set_index(uint32_t index);

void fram_set_state(FRAM_STATE_T state);

void setStatusRegister(uint8_t value);

void fram_task_entry(void);

#endif
