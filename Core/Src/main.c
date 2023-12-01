/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program of the SODAQ LOGGER
 * @author         : Phuoc K. Ly
 * @version        : 0.1
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *  This software is a free software and there is NO WARRANTY.
 *  No restriction on use. You can use, modify and redistribute it for
 *  personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 *  Redistributions of source code must retain the above copyright notice.
 *
 */

/*
    This copywrited (C) program was developed by Phuoc K. Ly (SODAQ) for a low power logging device using the STM32L412 MCU with external FRAM and SD card
    The main program is available at https://github.com/phuocly2304/SD_Logger
    This file provide all the main functions of the SODAQ Logger.

    SODAQ Logger is a simple but low power serial logger based on the STM32L412CBT6P running at 48MHz.
    The purpose of this logger was to create an easy to use logging device to assist all the Quality Assurance tasks

    SODAQ Logger works with an external 64KB FRAM and an 512MB XTX SD card. By default, the logger UART runs at 115200bps

    Two most important characteristics of the SODAQ Logger is the low power aspect and the ability to become a USB stick
    so the users can easily extract all the data directly on the device.

    22µA idle current consumption (When the device is not receiving any UART message)
    6mA actively buffering data to FRAM
    30mA actively writing from FRAM to SD Card.

    Input Voltage on can either be 3.3V directly to 3V3 pin or up to 6.5V. Input Voltage on RX-I pin must not exceed 6V.

    In order to extract data from the logger. Use a 10P Distance Test Stand Pcb Clip Clamp Fixture to clip on it.
    Note that a separated 3.3 - 5V must be apply on the Vbus pin to trigger the USB function.
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h> //for va_list var arg functions
#include <stdio.h>
#include <string.h>
#include "SPI_FRAM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define DEBUG
#ifdef DEBUG
    #warning "You are running a debug build!"
#endif

#define VERSION     "v1.0"
#define CONFIG_FILE "config.txt"

//#define LOGGER_RESET 1
/*************************************************FRAM ADDRESS CONFIG*****************************************************/
// External FRAM locations for user settings
#define DEVICE_ID_LOCATION 0x02 // Each logger will be assigned a unique ID

#define LOCATION_FILE_NUMBER_LSB 0x03 // 16-bit value LSB for file location number
#define LOCATION_FILE_NUMBER_MSB 0x04 // 16-bit value MSB for file location number

#define BAUD_LSB_LOCATION 0x05 // 16-bit value LSB for Logger baudrate
#define BAUD_MSB_LOCATION 0x06 // 16-bit value MSB for Logger baudrate

#define LOGGER_STAT_LOCATION 0x07 // Logger status location

/*---------------24 bit value of the FRAM Iterator address from the last logging ----------------*/
#define SYNC_BUFFER_MSB  0x08 // First 8 MSB of the Iterator value
#define SYNC_BUFFER_MSB2 0x09 // Second 8 bits of the Iterator
#define SYNC_BUFFER_LSB  0x0A // Last 8 LSB of the Iterator
/*-----------------------------------------------------------------------------------------------*/
// #define LOCATION_BAUD_SETTING_HIGH 0x0B
// #define LOCATION_BAUD_SETTING_MID 0x0C
// #define LOCATION_BAUD_SETTING_LOW 0x0D

// #define LOCATION_LOGGER_RESTORE 0x0E

#define LOCATION_BUFFER_START 20      // Address of start position when buffering
#define LOCATION_BUFFER_END   0x3ff70 // Address of end position when buffering (260000)
/*****************************************************************************************************************/

/********************************LOGGER STATUS & ERROR*****************************************************/
typedef enum { LOGGER_INIT, Buffer_Sync_, Buffer_Sync_DONE, BUFFER_OK, SD_DONE, USB_PLUGIN, USB_UNKNOWN, USER_CONFIG, CONFIG_OK } LOGGER_STAT;

typedef enum {
    ERROR_SD_INIT = 2, // blink LED 2 times a second if SD card is mounted fail.
    // LOGGER_RESET,
    CONFIG_ERROR
} LOGGER_ERROR;
/*************************************************************************************************/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS; // USB handler
UART_WakeUpTypeDef wakeup;              // Wake up handler
char Rx_data_1[2048];                   // Creat UART buffer [max 2048]
char Rx_data_2[2048];
char readBuf[2048];
/*************************************************SYSTEM FLAGS*****************************************************/
uint8_t USB_Flag = 0;          // USB Flag raise up when USB cable is connected
uint8_t RX_Flag_1 = 0;         // RX_flag when interrupt
uint8_t RX_Flag_2 = 0;         // RX_flag when interrupt
uint8_t RX_BACKUP_Flag = 0;    // Raise this flag mean data coming during SD write process
uint8_t SD_Flag = 0;           // Raise SD Flag when timeout & buffer is full
uint8_t RESET_FLAG = 1;        // If the device is reset - create new file in SD
uint8_t BUFFER_NEAR_FULL_ = 0; // When buffer is almost full and device is still buffering -- trigger this flag
uint8_t BOARD_POWER_UP = 1;    // First time power up the board -- create a new log file
uint8_t SD_MOUNT_FAIL = 0;     // Number of mount fail -- 3 times then error-handler
/*****************************************************************************************************************/

/*************************************************DEFAULT LOGGER CONFIGURATION*****************************************************/
uint32_t setting_user_baud = 0;          // User-config baud rate.
uint8_t setting_overwrite = 0;           // User-config overwrite mode. 1 = auto overwrite log file from the beginning
static char newFileName[13];             // Create new file name based on the available log spot
const char* CFG_FILENAME = "config.txt"; // This is the name of the file that contains the unit settings
long Current_System_Baud = 0;
/*************************************************************************************************************************/

/*************************************************USER DEFINE BUFFER SIZE, ITERATOR LOCAION AND TIMEOUT**********************************/
uint32_t LOCATION_BUFFER_ITERATOR = LOCATION_BUFFER_START; // Iterator to the current position of the buffer
#ifdef DEBUG
static const unsigned int BUFFER_MAX = 1000; // User define max buffer 100 characters for debugging
#else
static const unsigned int BUFFER_MAX = 30000; // User define max buffer from 20 to 262000
#endif
static const uint32_t MAX_IDLE_TIME_MSEC = 5000; // User define timeout before going low power
/****************************************************************************************************************************************/

// Some variables for FatFs
FATFS FatFs;                   // Fatfs handle
FIL newFile, workingFile, fil; // File handle
FRESULT fres;                  // Result after operations
char FATPATH[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void turn_off_spi_SD(void);
static void turn_off_spi_FRAM(void);

// void LoggerSetting(void);
void myprintf(const char* fmt, ...);
void Stop_Mode_Entry(void);
void System_error(LOGGER_ERROR Error_type);
void Append_Data();

uint8_t Tokenize_User_Setting(char* user_config_str);
uint8_t Baud_Config(uint32_t new_baud);
uint8_t Logger_Config(void);
uint8_t Logger_Reset(void);
uint8_t Append_File(char* fileName);
uint8_t Buffer_Sync(int* Buffer_Iterator, LOGGER_STAT stat);
uint8_t USB_init(LOGGER_STAT stat);

char* New_Log(LOGGER_STAT stat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Custom printf function using UART TX with interrupt*/
void myprintf(const char* fmt, ...)
{
    static char buffer[500];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}

/*!
 * @brief Check for USB cable appearance-- if yes then initialize USB peripheral
 * @param stat
 * @retval None
 */
uint8_t USB_init(LOGGER_STAT stat)
{
    /*If Cable plugin  USB device is configured by computer host*/
    if (stat == USB_UNKNOWN) {
        GPIO_InitTypeDef GPIO_InitStruct = { 0 };
        GPIO_InitStruct.Pin = Vbus_Sense_Pin | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else {
        /*Else turn off USB Peripheral & put the GPIOs to analog mode*/
        MX_USB_DEVICE_Init();
        USB_Flag = 1;
    }
    return LOGGER_INIT;
}

/*!
 * @brief Congigurate parameters for the logger based on the config.txt file inside the SD. If there is no config.txt file, default parameters will be apply.
 * @param None
 * @retval Logger status
 */
uint8_t Logger_Config(void)
{
    uint8_t attempt = 1;
    // Open the file system -- incase fail to mount the first time, re-try 5 times then blink LED for error handler
    fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now
    // FATFS_LinkDriver()
    while (fres != FR_OK) {
        if (attempt == 5) {
            SD_MOUNT_FAIL++;
            if (SD_MOUNT_FAIL == 4)
                SD_Flag = 0;
            myprintf("\r\n~ SD card re-initializing failed - device will reset ~\r\n\r\n");
            System_error(ERROR_SD_INIT);
            // return
        }
        myprintf("f_mount error (%i)\r\n Will try again for 5 times in 0.5 seconds\n", fres);
        myprintf("Try %d times\n", attempt);
        attempt++;
        if (FATFS_UnLinkDriver(&USERPath)) {
            myprintf("FATFS Unlinked fail!!");
            NVIC_SystemReset();
        }
        else
            myprintf("FATFS Unlink success\n");
        turn_off_spi_SD();
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
        HAL_Delay(500);

        /*Re-init SD*/
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin);
        MX_GPIO_Init();
        MX_SPI2_Init();
        MX_FATFS_Init();
        HAL_Delay(1000);
        myprintf("\r\n~ SD card re-initializing... ~\r\n\r\n");
        fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now
    }

#ifdef DEBUG
    myprintf("\r\n~ SD card mounted successfully ~\r\n\r\n");
    myprintf("\n-----Begin config:-----\n");
#endif
    fres = f_open(&fil, CONFIG_FILE, FA_READ | FA_OPEN_EXISTING);
    // /*This mean there is no config file -- a default config file will be added*/
    if (fres == FR_NO_FILE) {
        f_close(&fil);
        myprintf("No config found - creating default!\r\n");
        fres = f_open(&fil, CONFIG_FILE, FA_WRITE | FA_CREATE_NEW);
        myprintf("fres: (%i)\r\n", fres);
        if (fres == FR_OK) {
            BYTE baud_config[15];
            UINT bytesWrote;
            strncpy((char*)baud_config, "115200,0,0,0,", 7); // Default baudrate
            fres = f_write(&fil, baud_config, 7, &bytesWrote);
#ifdef DEBUG
            myprintf("Wrote total %i bytes!!\r\n", bytesWrote);
#endif
            myprintf("Default baud set to 115200\r\n");
            myprintf("Overwrite mode disabled\r\n");
            myprintf("CONFIG_OK!\r\n");
            f_close(&fil);
            return CONFIG_OK;
        }
        else {
            myprintf("f_write error (%i)\r\n", fres);
            // System_error(CONFIG_ERROR);
            return CONFIG_OK;
        }
    }
    else if (fres == FR_OK) {
#ifdef DEBUG
        myprintf("\r\nSuccessfully open configuration file!\r\n");
#endif
        char buffer[100];
        UINT byte_read;
        /*Read the file into the buffer*/
        f_read(&fil, (TCHAR*)buffer, sizeof(buffer), &byte_read);
        /*Tokenize the buffer by comma*/
        Tokenize_User_Setting(buffer);
        // close the file
        f_close(&fil);
        f_mount(NULL, "", 0);
        return CONFIG_OK;
    }
}

/*!
 * @brief Tokenize config variable from a string
 * @param user_config_str user_config_string
 * @retval Logger status
 */
uint8_t Tokenize_User_Setting(char* user_config_str)
{
    /*Tokenize the buffer by comma*/
    char* saveptr;
    char* token = strtok_r(user_config_str, ",", &saveptr);
    uint8_t count = 0;
    uint8_t tokens_list[4];
    /*Extract numbers from the tokens*/
    while (token != NULL && token < 4) {
        tokens_list[count] = atoi(token);
        myprintf("Token %d: %d\r\n", count, tokens_list[count]);
        token = strtok_r(NULL, ",", &saveptr);
        count++;
    }
    if (count < 4) {
        myprintf("Setting is invalid -- default setting will be applied\r\n");
    }
    setting_user_baud = tokens_list[0];
    Baud_Config(setting_user_baud);
    myprintf("New_baud: %d\r\n", setting_user_baud);
    setting_overwrite = tokens_list[1];
    myprintf("Overwrite mode: %d\r\n", setting_overwrite);
    return CONFIG_OK;
}

/*!
 * @brief Configurate baud rate from config.txt
 * @param new_baud new user-config baud rate
 * @retval Logger status
 */
uint8_t Baud_Config(uint32_t new_baud)
{
    // Disable the current USART
    HAL_UART_DeInit(&huart2);
    // Update the baudrate
    huart2.Init.BaudRate = new_baud;
    // Re-initialize the USART with the new baudrate
    HAL_UART_Init(&huart2);
    // Wait for the USART to be ready
    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {};
#ifdef DEBUG
    myprintf("Successfully config new baudrate of: %d\r\n", new_baud);
#endif
    return CONFIG_OK;
}

/*!
 * @brief Turn off FRAM peripheral by enable sleep mode and put the GPIOs to analog state
 * @param None
 * @retval None
 */
static void turn_off_spi_FRAM()
{
    HAL_SPI_DeInit(&hspi1);
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | FRAM_CS_Pin | LED_Pin | FRAM_HOLD_Pin, RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | FRAM_CS_Pin | LED_Pin | FRAM_HOLD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*!
 *  @brief  Turn off SD NAND chip by disable 3V3 power supply and put the GPIOs to analog mode
 *  @param  None
 *  @retval None
 */
static void turn_off_spi_SD()
{
    HAL_SPI_DeInit(&hspi2);
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin, RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
}

/*Receive data call back after idle time (1 frame reception) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    //HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)Rx_data_1, sizeof(Rx_data_1));
    if (RX_Flag_1==0) {
        if (RX_Flag_2) {
            memcpy(readBuf, Rx_data_2, sizeof(Rx_data_2));
            memset(Rx_data_2, 0, sizeof(Rx_data_2));
            RX_Flag_2 = 0;
        }
        else {
            memcpy(readBuf, Rx_data_1, sizeof(Rx_data_1));
            memset(Rx_data_1, 0, sizeof(Rx_data_1));
        }
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
        RX_Flag_1 = 1;
    }
    else {
        memcpy(Rx_data_2, Rx_data_1, sizeof(Rx_data_1));
        memset(Rx_data_1, 0, sizeof(Rx_data_1));
        RX_Flag_2 = 1;
    }
}

/*Enter stop mode 1 and set UART as wakeup source*/
void Stop_Mode_Entry()
{

    /*Kindly turn off FRAM and other peripheral :) */
    turn_off_spi_FRAM();

    /* Enable the USART2 Wake UP from STOP mode Interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_WUF);

    /* enable MCU wake-up by USART1 */
    HAL_UARTEx_EnableStopMode(&huart2);
// #ifdef DEBUG
// myprintf("I go to sleep now!");
// #endif
#ifdef DEBUG
    myprintf("******I am done waiting!!! Now I go sleep ******\n");
#else
    myprintf(">.<!\n");
#endif
    /*Deinit peripherals*/
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
    HAL_SuspendTick();
    /*Enter stop mode*/
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    /*Escape from stop mode*/
    SystemClock_Config();
    HAL_ResumeTick();
    MX_GPIO_Init();
    MX_SPI1_Init();
    turn_off_spi_SD();
    HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
    /* Wake up based on RXNE flag successful*/
    HAL_UARTEx_DisableStopMode(&huart2);
}

/*Create new | Check for logging file*/
char* New_Log(LOGGER_STAT stat)
{
    uint8_t msb, lsb;
    unsigned int newFileNumber = 0;
    uint8_t attempt = 1;

    /*Init SD PHY layer*/
    if (stat != Buffer_Sync_) {
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
        HAL_Delay(200);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin);
        MX_GPIO_Init();
        MX_SPI2_Init();
        MX_FATFS_Init();
        HAL_Delay(100);
    }
/*Init and Mount SD Card in FATFS*/
#ifdef DEBUG
    myprintf("\r\n~ SD card initializing... ~\r\n\r\n");
#endif
    // Open the file system -- incase fail to mount the first time, re-try 10 times before skipping this phase -- if the situation repeat for 3 times -> error
    // handler
    fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now
    // FATFS_LinkDriver()
    while (fres != FR_OK) {
        if (attempt == 5) {
            SD_MOUNT_FAIL++;
            if (SD_MOUNT_FAIL == 4)
                SD_Flag = 0;
            NVIC_SystemReset();
        }
        myprintf("f_mount error (%i)\r\n Will try again for 5 times in 0.5 seconds\n", fres);
        myprintf("Try %d times\n", attempt);
        attempt++;
        if (FATFS_UnLinkDriver(&USERPath)) {
            myprintf("FATFS Unlinked fail!!");
            NVIC_SystemReset();
        }
        else
            myprintf("FATFS Unlink success\n");
        turn_off_spi_SD();
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
        HAL_Delay(500);

        /*Re-init SD*/
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
        HAL_Delay(200);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin);
        MX_GPIO_Init();
        MX_SPI2_Init();
        MX_FATFS_Init();
        HAL_Delay(100);
        myprintf("\r\n~ SD card re-initializing... ~\r\n\r\n");
        fres = f_mount(&FatFs, (const TCHAR*)FATPATH, 1); // 1=mount now
    }

#ifdef DEBUG
    myprintf("\r\n~ SD card mounted successfully ~\r\n\r\n");
    myprintf("\n***FRAM generate/indicate file location!!\n");
#endif
    // Combine two 8-bit FRAM spots into one 16-bit number
    lsb = FRAM_read8(LOCATION_FILE_NUMBER_LSB);
    msb = FRAM_read8(LOCATION_FILE_NUMBER_MSB);

#ifdef DEBUG
    myprintf("Location lsb: %d\n", lsb);
    myprintf("Location msb: %d\n", msb);
#endif

    // If both FRAM spots are 255 (0xFF), that means they are un-initialized (first time Logger has been turned on)
    newFileNumber = msb;
    newFileNumber = newFileNumber << 8;
    newFileNumber |= lsb;

    if (stat == Buffer_Sync_) {
        sprintf(newFileName, "LOG%05d.TXT", newFileNumber);
        newFileNumber += 1;
        uint8_t temp_lsb = (uint8_t)(newFileNumber & 0x00ff);
        uint8_t temp_msb = (uint8_t)((newFileNumber & 0xff00) >> 8);
        FRAM_writeEnable(true);
        FRAM_write8(LOCATION_FILE_NUMBER_LSB, temp_lsb); // LSB

        if (FRAM_read8(LOCATION_FILE_NUMBER_MSB) != temp_msb)
            FRAM_write8(LOCATION_FILE_NUMBER_MSB, temp_msb); // MSB
        FRAM_writeEnable(false);

#ifdef DEBUG
        myprintf("Updated next log file number\n");
#endif
        return newFileName;
    }

    if (RESET_FLAG) {
        // Let's init them both to 0
        if ((lsb == 0xff) && (msb == 0xff)) {
            newFileNumber = 0;
            FRAM_writeEnable(true);
            FRAM_write8(LOCATION_FILE_NUMBER_LSB, 0x00);
            FRAM_write8(LOCATION_FILE_NUMBER_MSB, 0x00);
            FRAM_writeEnable(false);
#ifdef DEBUG
            myprintf("Location lsb updated: %d\n", lsb);
            myprintf("Location msb updated: %d\n", msb);
#endif
        }
        // Let's quit if we reach 65534 files
        if (newFileNumber == 65534) {
            return 0;
        }
        while (1) {
            sprintf(newFileName, "LOG%05d.TXT", newFileNumber);

            // If we are able to create this file, then it didn't exist, we're good, break
            fres = f_open(&newFile, newFileName, FA_WRITE | FA_CREATE_NEW);
            if (fres == FR_OK) {
                f_close(&newFile);
                break;
            }

            // If file exists, see if empty. If so, use it.
            if (fres == FR_EXIST) {
                fres = f_open(&newFile, newFileName, FA_READ | FA_OPEN_EXISTING);
                if (f_size(&newFile) == 0) {
                    f_close(&newFile);  // Close this existing file we just opened.
                    return newFileName; // Use existing empty file.
                }
                f_close(&newFile);
            }
            // try the next number
            newFileNumber++;
            if (newFileNumber > 65533)
                return 0;
        }
        lsb = (uint8_t)(newFileNumber & 0x00ff);
        msb = (uint8_t)((newFileNumber & 0xff00) >> 8);

        FRAM_writeEnable(true);
        FRAM_write8(LOCATION_FILE_NUMBER_LSB, lsb); // LSB

        if (FRAM_read8(LOCATION_FILE_NUMBER_MSB) != msb)
            FRAM_write8(LOCATION_FILE_NUMBER_MSB, msb); // MSB

        FRAM_writeEnable(false);
        FRAM_sleepEnable(true);

#ifdef DEBUG
        myprintf("Location lsb updated: %d\n", lsb);
        myprintf("Location msb updated: %d\n", msb);
#endif
        RESET_FLAG = 0;
    }
    else {
        sprintf(newFileName, "LOG%05d.TXT", newFileNumber);
#ifdef DEBUG
        myprintf("File number: %s\r\n", newFileName);
#endif // DEBUG
    }
    return newFileName;
}

/*!
 *  @brief  Main function to receive UART data and buffer to FRAM
 *          After interval waiting time, data from buffer will be written to SD NAND and buffer is reset
 *  @param  None
 *  @retval None
 */
void Append_Data()
{
    // unsigned long currentTime = HAL_GetTick();
    FRAM_sleepEnable(false);
    unsigned long lastSyncTime = HAL_GetTick(); // Keeps track of the last time the data was synced
    while (1) {
        /*Buffering incoming data*/
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
        if (RX_Flag_1) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            if (FRAM_write(&LOCATION_BUFFER_ITERATOR, readBuf) != FRAM_OK) {
                Error_Handler();
            }
            if (Buffer_Sync(&LOCATION_BUFFER_ITERATOR, Buffer_Sync_) == BUFFER_OK) {
                myprintf("Buffer_OK Size: %d\n", LOCATION_BUFFER_ITERATOR);
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
            /*If buffer exceed max capacity -- write to SD*/
            if (LOCATION_BUFFER_ITERATOR >= LOCATION_BUFFER_START + BUFFER_MAX) {
                SD_Flag = 1;
            }
            lastSyncTime = HAL_GetTick(); // Reset interval time
            RX_Flag_1 = 0;                // Reset RX_IT flag
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)Rx_data_1, sizeof(Rx_data_1));
        }

        /*Write data from buffer to SD after buffer is full*/
        else if (SD_Flag) {
            Append_File(New_Log(BUFFER_OK));
            SD_Flag = 0;
            lastSyncTime = HAL_GetTick(); // Reset interval time
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)Rx_data_1, sizeof(Rx_data_1));
        }

        /*After [USER_DEFINE] seconds -> Go to sleep*/
        else if ((unsigned long)(HAL_GetTick() - lastSyncTime) > MAX_IDLE_TIME_MSEC) {
            FRAM_sleepEnable(true);
            Stop_Mode_Entry();
#ifdef DEBUG
            myprintf("******WAKE UP FROM STOP MODE******\n");
#endif
            FRAM_sleepEnable(false); // make sure FRAM is woken up properly
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)Rx_data_1, sizeof(Rx_data_1));
            lastSyncTime = HAL_GetTick(); // Reset interval time
        }
    }
}

/*!
 *  @brief  Main function to receive UART data and buffer to FRAM
 *          After interval waiting time, data from buffer will be written to SD NAND and buffer is reset
 *  @param  fileName
 *          Name of the file generated from the function New_Log
 *  @retval None
 */
uint8_t Append_File(char* fileName)
{
    UINT bytesWrote = 0;
    UINT Total_Byte_Wrote = 0;
    const char writeBuf[8192];
    fres = f_open(&workingFile, fileName, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (fres != FR_OK) {
#ifdef DEBUG
        myprintf("f_open error (%i)\r\n", fres);
#endif
        Error_Handler();
    }
    if (f_size(&workingFile) == 0) {
        f_rewind(&workingFile);
        f_sync(&workingFile);
    }

    FRAM_sleepEnable(false);
    uint32_t index = LOCATION_BUFFER_START;
    while (index < LOCATION_BUFFER_ITERATOR) {
        strncpy((char*)writeBuf, FRAM_read(&index), sizeof(writeBuf)); // copy from buffer to SD (reset buffer iterator)
        fres = f_write(&workingFile, writeBuf, strlen(writeBuf), &bytesWrote);
        if (fres != FR_OK) {
            myprintf("f_write error (%i)\r\n");
        }
        else
            Total_Byte_Wrote += bytesWrote;
    }

#ifdef DEBUG
    myprintf("Wrote total %i bytes to %s!\r\n", Total_Byte_Wrote, fileName);
    f_close(&workingFile);
    /*Now let's try to open file "test.txt"*/
    fres = f_open(&workingFile, fileName, FA_READ | FA_OPEN_EXISTING);
    if (fres != FR_OK) {
        myprintf("f_open error (%i)\r\n", fres);
        while (1)
            ;
    }
    myprintf("I was able to open %s !!\r\n", fileName);
#endif
    f_sync(&workingFile);
    /*We're done with SD, let's un-mount the drive*/
    if (FATFS_UnLinkDriver(USERPath)) {
        myprintf("FATFS Unlinked fail!!");
        Error_Handler();
    }
    f_mount(NULL, FATPATH, 0);
    /*We're also done with FRAM, let's reset the iterator*/
    LOCATION_BUFFER_ITERATOR = LOCATION_BUFFER_START;
    Buffer_Sync(LOCATION_BUFFER_ITERATOR, SD_DONE);
    /*Don't forget to turn off the power after using :) */
    turn_off_spi_SD();
    HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
    myprintf("SD_OK!\n");
    return 0;
}

/*Blink leds when system receive error based on error type*/
void System_error(LOGGER_ERROR Error_type)
{
    while (1) {
        for (int index = 0; index < Error_type; index++) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
            HAL_Delay(200);
        }
        HAL_Delay(2000);
    }
}

/*!
 *  @brief  Synchronize the Buffer iterator after every buffer action
 *          On booting up, the logger will check this function status.
 *  @param  Buffer_Iterator To store the current buffer iterator position after
 *  @param  stat  Current status for the logger || LOGGER_INIT: Check for Logger current status.
 * If stat is Buffer_Sync than previous logging still have remain data in the buffer, the logger will sync all the data to the previous log file.
 * || Buffer_Sync: Save current iterator in to buffer
 * || SD_DONE: Done writing to the SD
 *  @retval None
 */
uint8_t Buffer_Sync(int* Buffer_Iterator, LOGGER_STAT stat)
{
    switch (stat) {
        case LOGGER_INIT:
            FRAM_sleepEnable(false);
            uint32_t _update_iterator = 0;
            uint8_t stat_check = FRAM_read8(LOGGER_STAT_LOCATION);
            if (stat_check == SD_DONE || stat_check == BUFFER_OK || stat_check == Buffer_Sync_DONE) {
#ifdef DEBUG
                myprintf("Done synchronizing.!!\n");
#endif
                return BUFFER_OK;
            }
            else if (stat_check == Buffer_Sync_) {
                _update_iterator = FRAM_read8(SYNC_BUFFER_MSB);
                _update_iterator = _update_iterator << 8;
                _update_iterator |= FRAM_read8(SYNC_BUFFER_MSB2);
                _update_iterator = _update_iterator << 8;
                _update_iterator |= FRAM_read8(SYNC_BUFFER_LSB);

                LOCATION_BUFFER_ITERATOR = _update_iterator;
#ifdef DEBUG
                myprintf("\n\nLast logging period stops at %d\n", LOCATION_BUFFER_ITERATOR);
                myprintf("Buffer is reset!!... Now start synchronizing to SD\n\n");
#endif
                Append_File(New_Log(Buffer_Sync_));
                FRAM_writeEnable(true);
                FRAM_write8((uint8_t)LOGGER_STAT_LOCATION, Buffer_Sync_DONE);
                FRAM_writeEnable(false);
                return Buffer_Sync_DONE;
            }
            break;

        case Buffer_Sync_:
            /*Write current buffer location to FRAM (From uint32_t to 3 uin8_t(s) )*/
            FRAM_writeEnable(true);
            uint8_t _location = 0; // Save current iterator location to buffer
            _location = (*Buffer_Iterator >> 16) & 0xff;
            FRAM_write8(SYNC_BUFFER_MSB, _location);
            _location = (*Buffer_Iterator >> 8) & 0xff;
            FRAM_write8(SYNC_BUFFER_MSB2, _location);
            _location = *Buffer_Iterator & 0xff;
            FRAM_write8(SYNC_BUFFER_LSB, _location);

            /*Update logger Status*/
            FRAM_write8((uint8_t)LOGGER_STAT_LOCATION, (uint8_t)Buffer_Sync_);
            FRAM_writeEnable(false);
            break;

        case SD_DONE:
            FRAM_writeEnable(true);
            FRAM_write8((uint8_t)LOGGER_STAT_LOCATION, SD_DONE);
            FRAM_writeEnable(true);
            return BUFFER_OK;
            break;
        default:
            break;
    }
    return BUFFER_OK;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
    if (!FRAM_begin(3)) {
        myprintf("SPI FRAM begin error.. check your connection !!\n");
        while (1)
            ;
    }
    else
        myprintf("Logger init %s\n", VERSION);
// if(Logger_Config() == CONFIG_OK){
//   NVIC_SystemReset();
// }
#ifdef DEBUG
    uint8_t manufID;
    uint16_t prodID;
    FRAM_getID(&manufID, &prodID); // Check FRAM parameter
    myprintf("Manufacture ID: 0x%X\n", manufID);
    myprintf("product ID: 0x%X\n", prodID);
    myprintf("Current iterator: %d\n", LOCATION_BUFFER_ITERATOR);
#endif

#ifdef LOGGER_RESET
    FRAM_writeEnable(true);
    myprintf("FRAM Start reseting!\n");
    FRAM_write8((uint8_t)LOGGER_STAT_LOCATION, BUFFER_OK);
    FRAM_write8(DEVICE_ID_LOCATION, 0);
    FRAM_write8(LOCATION_FILE_NUMBER_LSB, 0);
    FRAM_write8(LOCATION_FILE_NUMBER_MSB, 0);
    FRAM_writeEnable(false);
    myprintf("Logger has been reset sucessfully.. please reboot the device!\n");
    System_error(LOGGER_RESET);
#endif
    if (Buffer_Sync(NULL, LOGGER_INIT) == Buffer_Sync_DONE) {
#ifdef DEBUG
        myprintf("Done synchronizing from last logging time...reset now!!\n");
#endif
        NVIC_SystemReset();
    }

    if (HAL_GPIO_ReadPin(Vbus_Sense_GPIO_Port, Vbus_Sense_Pin)) {
        HAL_Delay(500);
        myprintf("Logger is in USB mode! Please connect to a USB port on the adapter\n");
        USB_init(USB_PLUGIN);
    }
    else
        USB_init(USB_UNKNOWN);

    if (USB_Flag == 0) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)Rx_data_1, sizeof(Rx_data_1));
        /*turn off SD NAND to save power*/
        turn_off_spi_SD();
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
        myprintf("<\n");
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (USB_Flag == 0) {
            Append_Data();
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FRAM_HOLD_Pin|FRAM_CS_Pin|SD_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FRAM_HOLD_Pin SD_EN_Pin LED_Pin */
  GPIO_InitStruct.Pin = FRAM_HOLD_Pin|SD_EN_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FRAM_CS_Pin */
  GPIO_InitStruct.Pin = FRAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB3 PB4 PB5 PB6
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Vbus_Sense_Pin */
  GPIO_InitStruct.Pin = Vbus_Sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Vbus_Sense_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
