/*!
 *  @file   SPI_FRAM.c
 *  @brief  This is a redistribution of the SPI_FRAM chip driver implemented on STM32L412xx MCU using HAL Driver.
 *  @author K. Ly (SODAQ)
 *  @version 0.1
 ******************************************************************************
 * @attention
 * Software License Agreement (BSD License)
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
    This code was derived by Phuoc K. Ly (SODAQ) from a copywrited (C) library written by K.Townsend (Adafruit Industries)
    available at https://github.com/adafruit/Adafruit_FRAM_SPI
    This file provides the FRAM driver functions and SPI code required to manage
    an SPI-connected to the MB85RS2MTA FRAM
*/

#include <stdint.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"
#include "console.h"
#include "define.h"
#include "main.h"
#include "spi_fram.h"

extern SPI_HandleTypeDef SPI_FRAM_HANDLE;
static FRAM_DATA_T fram_data = { 0u };

/* Function prototype*/
#define SPI1_CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)
#define SPI1_CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)

/* Supported flash devices */
const struct
{
    uint8_t manufID; ///< Manufacture ID
    uint16_t prodID; ///< Product ID
    uint32_t size;   ///< Size in bytes
} _supported_devices[] = {
    // Sorted in numerical order
    // Fujitsu
    { 0x04, 0x0101, 2 * 1024UL },   // MB85RS16
    { 0x04, 0x0302, 8 * 1024UL },   // MB85RS64V
    { 0x04, 0x2303, 8 * 1024UL },   // MB85RS64T
    { 0x04, 0x2503, 32 * 1024UL },  // MB85RS256TY
    { 0x04, 0x4803, 256 * 1024UL }, // MB85RS2MTA
    { 0x04, 0x4903, 512 * 1024UL }, // MB85RS4MT

    // Cypress
    { 0x7F, 0x7F7f, 32 * 1024UL }, // FM25V02
                                   // (manu = 7F7F7F7F7F7FC2, device = 0x2200)

    // Lapis
    { 0xAE, 0x8305, 8 * 1024UL } // MR45V064B
};

/*!
 *  @brief  Check if the flash device is supported
 *  @param  manufID
 *          ManufactureID to be checked
 *  @param  prodID
 *          ProductID to be checked
 *  @return size of device, 0 if not supported
 */
static uint32_t check_supported_device(const uint8_t* manufID, const uint16_t* prodID)
{
    for (uint8_t i = 0; i < sizeof(_supported_devices) / sizeof(_supported_devices[0]); i++)
    {
        if (*manufID == _supported_devices[i].manufID && *prodID == _supported_devices[i].prodID)
            return _supported_devices[i].size;
    }
    return 0;
}

static bool fram_get_id(uint8_t* manufacturerID, uint32_t* productID)
{
    uint8_t cmd = OPCODE_RDID;
    uint8_t a[4u] = { 0u };

    SPI1_CS_LOW();
    HAL_SPI_Transmit(&SPI_FRAM_HANDLE, &cmd, 1u, -1);
    HAL_SPI_Receive(&SPI_FRAM_HANDLE, (uint8_t*)a, sizeof(a), -1);
    SPI1_CS_HIGH();

    if (a[1u] == 0x7fu)
    {
        // Device with continuation code (0x7F) in their second byte
        // Manuf ( 1 byte)  - 0x7F - Product (2 bytes)
        *manufacturerID = (a[0]);
        *productID = (a[2u] << 8u) + a[3u];
    }
    else
    {
        // Device without continuation code
        // Manuf ( 1 byte)  - Product (2 bytes)
        *manufacturerID = (a[0]);
        *productID = (a[1] << 8) + a[2u];
    }

    return true;
}

/*!
 *   @brief  Sets adress size to provided value
 *   @param  nAddressSize
 *           address size in bytes
 */
static void fram_set_addr_size(uint8_t nAddressSize)
{
    fram_data._nAddressSizeBytes = nAddressSize;
}

bool fram_init(void)
{
    bool ret = true;
    /* Check SPI state*/
    if (SPI_FRAM_HANDLE.State != HAL_SPI_STATE_READY)
    {
        return false;
    }

#ifdef MB85RS2MTA
    fram_set_addr_size(3u);
#else
    fram_set_addr_size(0u); // Se up guard against this
#endif

    uint8_t manuf_id;
    uint32_t prod_id;

    ret = fram_get_id(&manuf_id, &prod_id); // Check FRAM parameter
    const uint32_t fram_size = check_supported_device(&manuf_id, &prod_id);
    ret = fram_size != 0u;

    if (ret)
    {
        log_info("fram size: %lu bytes ", fram_size);
    }
    else
    {
        log_info("Unsupported / No connection to fram device ");
    }

    return ret;
}

/**
    @brief  Enables or disables writing to the SPI dram
    @param enable
            True enables writes, false disables writes
*/
static bool fram_write_enable(bool enabled)
{
    uint8_t cmd;
    if (enabled)
    {
        cmd = OPCODE_WREN;
    }
    else
    {
        cmd = OPCODE_WRDI;
    }
    SPI1_CS_LOW();
    HAL_SPI_Transmit(&SPI_FRAM_HANDLE, &cmd, sizeof(cmd), -1);
    SPI1_CS_HIGH();
}

/**
 * @brief Enables or disables sleep mode of the FRAM
 * @param enable
 *              True enables sleep, flash disables sleep
 */
static bool fram_sleep_enable(bool enable)
{
    uint8_t cmd;
    if (enable)
    {
        cmd = OPCODE_SLEEP;
        SPI1_CS_LOW();
        HAL_SPI_Transmit_IT(&SPI_FRAM_HANDLE, &cmd, sizeof(cmd));
        SPI1_CS_HIGH();
    }
    else
    {
        SPI1_CS_LOW();
        HAL_Delay(1);
        SPI1_CS_HIGH();
    }
}

/**
 *  @brief  Writes 8-bit at the specific FRAM address
 *  @param addr
 *         The 32-bit address to write to in FRAM memory
 *  @param value
 *         The 8-bit value to write at framAddr
 */
bool fram_write_8b(uint32_t addr, uint8_t value)
{
    bool ret = true;
    uint8_t buffer[10];
    uint8_t i = 0;

    buffer[i++] = OPCODE_WRITE;
    if (fram_data._nAddressSizeBytes > 3)
        buffer[i++] = (uint8_t)(addr >> 24);
    if (fram_data._nAddressSizeBytes > 2)
        buffer[i++] = (uint8_t)(addr >> 16);
    buffer[i++] = (uint8_t)(addr >> 8);
    buffer[i++] = (uint8_t)(addr & 0xFF);
    buffer[i++] = value;

    SPI1_CS_LOW();
    HAL_StatusTypeDef hal_stat = HAL_SPI_Transmit_IT(&SPI_FRAM_HANDLE, (uint8_t*)buffer, i);
    if (hal_stat != HAL_OK)
    {
        log_info("spi failed err %d", hal_stat);
        ret = false;
    }

    return ret;
}

/**
 *   @brief  Writes string starting at the specific FRAM address
 *   @param addr
 *           The 32-bit address to write to in FRAM memory
 *   @param values
 *           The pointer to an array of 8-bit values to write starting at addr
 */
bool fram_write(const uint32_t addr, const uint8_t* pData, uint8_t pLength)
{
    fram_write_enable(true);
    bool ret = true;
    uint8_t buffer[10];
    uint8_t index = 0;

    buffer[index++] = OPCODE_WRITE;
    if (fram_data._nAddressSizeBytes > 3)
    {
        buffer[index++] = (uint8_t)(addr >> 24);
    }
    if (fram_data._nAddressSizeBytes > 2)
    {
        buffer[index++] = (uint8_t)(addr >> 16);
    }

    buffer[index++] = (uint8_t)(addr >> 8);
    buffer[index++] = (uint8_t)(addr & 0xFF);

    SPI1_CS_LOW();

    if (HAL_SPI_Transmit(&SPI_FRAM_HANDLE, (uint8_t*)buffer, index, -1) != HAL_OK)
    {
        ret = false;
    }
    if (HAL_SPI_Transmit_DMA(&SPI_FRAM_HANDLE, pData, pLength) != HAL_OK)
    {
        ret = false;
    }
    else
    {
        fram_set_index(addr + pLength);
    }

    return ret;
}

/**
 *   @brief  Reads an 8-bit value from the specified FRAM address
 *   @param  addr
 *           The 32-bit address to read from in FRAM memory
 *   @return The 8-bit value retrieved at framAddr
 */
static uint8_t fram_read_8b(uint32_t addr)
{
    uint8_t buffer[10], val;
    uint8_t i = 0;

    buffer[i++] = OPCODE_READ;
    if (fram_data._nAddressSizeBytes > 3)
        buffer[i++] = (uint8_t)(addr >> 24);
    if (fram_data._nAddressSizeBytes > 2)
        buffer[i++] = (uint8_t)(addr >> 16);
    buffer[i++] = (uint8_t)(addr >> 8);
    buffer[i++] = (uint8_t)(addr & 0xFF);

    SPI1_CS_LOW();
    HAL_SPI_Transmit(&SPI_FRAM_HANDLE, (uint8_t*)buffer, i, -1);
    HAL_SPI_Receive(&SPI_FRAM_HANDLE, &val, sizeof(val), -1);
    SPI1_CS_HIGH();

    return val;
}

/**
 *   @brief  Read 'string' starting at the specific FRAM address
 *   @param  addr
 *           The 32-bit address to write to in FRAM memory
 *   @param  values
 *           The pointer to an array of 8-bit values to read starting at addr
 *   @param length
 *           The number of bytes to read
 */
bool fram_read(const uint32_t addr, uint8_t* pData, uint8_t pLength)
{
    bool ret = true;
    uint8_t buffer[10];
    uint8_t index = 0;

    buffer[index++] = OPCODE_READ;
    if (fram_data._nAddressSizeBytes > 3)
    {
        buffer[index++] = (uint8_t)(addr >> 24);
    }
    if (fram_data._nAddressSizeBytes > 2)
    {
        buffer[index++] = (uint8_t)(addr >> 16);
    }

    buffer[index++] = (uint8_t)(addr >> 8);
    buffer[index++] = (uint8_t)(addr & 0xFF);

    SPI1_CS_LOW();
    if (HAL_SPI_Transmit(&SPI_FRAM_HANDLE, (uint8_t*)buffer, index, -1) != HAL_OK)
    {
        ret = false;
    }
    if (HAL_SPI_Receive_DMA(&SPI_FRAM_HANDLE, pData, pLength) != HAL_OK)
    {
        ret = false;
    }

    return ret;
}

FRAM_STATE_T fram_get_state(void)
{
    return fram_data.state;
}

uint32_t fram_get_index(void)
{
    return fram_data.current_index;
}

void fram_set_state(FRAM_STATE_T state)
{
    fram_data.state = state;
}

void fram_set_index(uint32_t index)
{
    fram_data.current_index = index;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPI1_CS_HIGH();
    fram_set_state(FRAM_DONE_READ);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPI1_CS_HIGH();
    fram_write_enable(false); // Should this be here?
    fram_set_state(FRAM_IDLE);
}
