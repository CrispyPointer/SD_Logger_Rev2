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

#define FRAM_OK 0
/** Operation Codes **/
typedef enum opcodes_e
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

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          FRAM SPI
 */

typedef struct SPI_FRAM
{
    uint8_t _nAddressSizeBytes;
} SPI_FRAM;

/**
 *  @brief  Initializes SPI and configures the chip (call this function before
 *          doing anything else)
 *  @return true if successful
 */
bool fram_init(void);

/**
    @brief  Enables or disables writing to the SPI dram
    @param enable
            True enables writes, false disables writes
*/
bool fram_write_enable(bool enable);

/**
 *  @brief  Writes 8-bit at the specific FRAM address
 *  @param addr
 *         The 32-bit address to write to in FRAM memory
 *  @param value
 *         The 8-bit value to write at framAddr
 */
bool fram_write_8b(uint32_t addr, uint8_t value);

/**
 *   @brief  Writes string starting at the specific FRAM address
 *   @param addr
 *           The 32-bit address to write to in FRAM memory
 *   @param values
 *           The pointer to an array of 8-bit values to write starting at addr
 */
bool fram_write(const uint32_t addr, const uint8_t* pData, uint8_t pLength);

/**
 *   @brief  Reads an 8-bit value from the specified FRAM address
 *   @param  addr
 *           The 32-bit address to read from in FRAM memory
 *   @return The 8-bit value retrieved at framAddr
 */
uint8_t fram_read_8b(uint32_t addr);

/**
 *   @brief  Read 'string' starting at the specific FRAM address
 *   @param  addr
 *           The 32-bit address to write to in FRAM memory
 *   @param  values
 *           The pointer to an array of 8-bit values to read starting at addr
 *   @param length
 *           The number of bytes to read
 */
bool fram_read(const uint32_t addr, uint8_t* pData, uint8_t pLength);

uint8_t getStatusRegister(void);

void setStatusRegister(uint8_t value);

/**
 * @brief Enables or disables sleep mode of the FRAM
 * @param enable
 *              True enables sleep, flash disables sleep
 */
bool fram_sleep_enable(bool enable);

void fram_task_entry(void);

#endif
