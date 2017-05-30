/*
 * Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * Defines peripherals available for use on ISM43362_M3G_L44 board
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif


/*
ISM43362_M3G_L44 platform pin definitions ...
+--------------------------------------------------------------------------------------------------------+
| Enum ID       |Pin |   Pin Name on    |    Module     | STM32| Peripheral  |    Board     | Peripheral  |
|               | #  |      Module      |  GPIO Alias   | Port | Available   |  Connection  |   Alias     |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_1  | 16 | WKUP             | WICED_GPIO_1  | A  0 | GPIO        |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_2  | 23 | GPIO0            | WICED_GPIO_2  | B  5 | GPIO        | BUTTON SW1   | WICED_PWM_3 |
|               |    |                  |               |      | SPI1_MOSI   |              |             |
|               |    |                  |               |      | TIM3_CH2    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_3  | 26 | GPIO3            | WICED_GPIO_3  | B  8 | GPIO        | LED7(RED)    | WICED_PWM_4 |
|               |    |                  |               |      | I2C1_SCL    |              |             |
|               |    |                  |               |      | TIM4_CH3    |              |             |
|               |    |                  |               |      | TIM10_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_4  | 13 | ADC0/SPI_DATARDY | WICED_GPIO_4  | A  3 | ADC123_IN3  | THERMISTOR   | WICED_ADC_1 |
|               |    |                  |               |      | GPIO        | SPI_DATARDY  |             |
|               |    |                  |               |      | TIM2_CH4    |              |             |
|               |    |                  |               |      | TIM5_CH4    |              |             |
|               |    |                  |               |      | TIM9_CH2    |              |             |
|               |    |                  |               |      | UART2_RX    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_5  | 12 | ADC1/SPI_SSN     | WICED_GPIO_5  | A  4 | ADC12_IN4   |              | WICED_ADC_2 |
|               |    |                  |               |      | DAC1_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | I2S3_WS     |              |             |
|               |    |                  |               |      | SPI1_NSS    |              |             |
|               |    |                  |               |      | SPI3_NSS    |              |             |
|               |    |                  |               |      | USART2_CK   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_6  | 11 | ADC2/SPI_SCK     | WICED_GPIO_6  | A  5 | ADC12_IN5   |              | WICED_ADC_3 |
|               |    |                  |               |      | DAC2_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_SCK    |              |             |
|               |    |                  |               |      | TIM2_CH1_ETR|              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_7  | 10 | ADC3/SPI_MISO    | WICED_GPIO_7  | A  6 | ADC12_IN6   |              | WICED_ADC_4 |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MISO   |              |             |
|               |    |                  |               |      | TIM1_BKIN   |              |             |
|               |    |                  |               |      | TIM3_CH1    |              |             |
|               |    |                  |               |      | TIM8_BKIN   |              |             |
|               |    |                  |               |      | TIM13_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_8  | 09 | ADC4/SPI_MOSI    | WICED_GPIO_8  | A  7 | ADC12_IN7   |              | WICED_ADC_5 |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MOSI   |              |             |
|               |    |                  |               |      | TIM1_CH1N   |              |             |
|               |    |                  |               |      | TIM3_CH2    |              |             |
|               |    |                  |               |      | TIM8_CH1N   |              |             |
|               |    |                  |               |      | TIM14_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_9  | 22 | TX               | WICED_GPIO_9  | A  9 | GPIO        |              |             |
|               |    |                  |               |      | I2C3_SMBA   |              |             |
|               |    |                  |               |      | TIM1_CH2    |              |             |
|               |    |                  |               |      | USART1_TX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_10 | 21 | RX               | WICED_GPIO_10 | A 10 | GPIO        |              |             |
|               |    |                  |               |      | TIM1_CH3    |              |             |
|               |    |                  |               |      | USART1_RX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_11 | 24 | GPIO1            | WICED_GPIO_11 | B  6 | GPIO        | Button SW2   | WICED_PWM_1 |
|               |    |                  |               |      | TIM4_CH1    |              |             |
|               |    |                  |               |      | I2C1_SCL    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_12 | 25 | GPIO2            | WICED_GPIO_12 | B  7 | GPIO        |              | WICED_PWM_2 |
|               |    |                  |               |      | TIM4_CH2    |              |             |
|               |    |                  |               |      | I2C1_SDA    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_13 | 04 | TMS              | WICED_GPIO_13 | A 13 | GPIO        |              |             |
|               |    |                  |               |      | JTMS-SWDIO  |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_14 | 05 | TCK              | WICED_GPIO_14 | A 14 | GPIO        |              |             |
|               |    |                  |               |      | JTCK-SWCLK  |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_15 | 06 | TDI              | WICED_GPIO_15 | A 15 | GPIO        |              |             |
|               |    |                  |               |      | JTDI        |              |             |
|               |    |                  |               |      | I2S3_WS     |              |             |
|               |    |                  |               |      | SPI1_NSS    |              |             |
|               |    |                  |               |      | SPI3_NSS    |              |             |
|               |    |                  |               |      | TIM2_CH1_ETR|              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_16 | 07 | TDO              | WICED_GPIO_16 | B  3 | GPIO        |              |             |
|               |    |                  |               |      | JTDO        |              |             |
|               |    |                  |               |      | SPI1_SCK    |              |             |
|               |    |                  |               |      | SPI3_SCK    |              |             |
|               |    |                  |               |      | I2S3_SCK    |              |             |
|               |    |                  |               |      | TIM2_CH2    |              |             |
|               |    |                  |               |      | TRACESWO    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_17 | 08 | TRSTN            | WICED_GPIO_17 | B  4 | GPIO        |              |             |
|               |    |                  |               |      | NJTRST      |              |             |
|               |    |                  |               |      | SPI1_MISO   |              |             |
|               |    |                  |               |      | SPI3_MISO   |              |             |
|               |    |                  |               |      | TIM3_CH1    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_18 | 27 | GPIO4            | WICED_GPIO_18 | B  9 | GPIO        | LED6(GREEN)  | WICED_PWM_5 |
|               |    |                  |               |      | I2C1_SDA    |              |             |
|               |    |                  |               |      | TIM4_CH4    |              |             |
|               |    |                  |               |      | TIM11_CH1   |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_19 | 28 | CFG0             | WICED_GPIO_19 | B 10 | GPIO        |              |             |
|               |    |                  |               |      | USART3_TX   |              |             |
|               |    |                  |               |      | I2S_SCK     |              |             |
|               |    |                  |               |      | SPI2_SCK    |              |             |
|               |    |                  |               |      | I2C2_SCL    |              |             |
|               |    |                  |               |      | TIM2_CH3    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_20 | 29 | CFG1             | WICED_GPIO_20 | B 11 | GPIO        |              |             |
|               |    |                  |               |      | USART3_RX   |              |             |
|               |    |                  |               |      | I2C2_SDA    |              |             |
|               |    |                  |               |      | TIM2_CH4    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_21 | 32 | GPIO13           | WICED_GPIO_21 | B 13 | GPIO        |              | WICED_PWM_6 |
|               |    |                  |               |      | USART3_CTS  |              |             |
|               |    |                  |               |      | TIM1_CH1N   |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_22 | 31 | GPIO14           | WICED_GPIO_22 | B 14 | GPIO        |              | WICED_PWM_7 |
|               |    |                  |               |      | USART3_RTS  |              |             |
|               |    |                  |               |      | SPI2_MISO   |              |             |
|               |    |                  |               |      | TIM8_CH2N   |              |             |
|               |    |                  |               |      | TIM1_CH2N   |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_23 | 30 | GPIO15           | WICED_GPIO_23 | B 15 | GPIO        |              | WICED_PWM_8 |
|               |    |                  |               |      | SPI2_MISO   |              |             |
|               |    |                  |               |      | TIM1_CH3N   |              |             |
|               |    |                  |               |      | TIM8_CH3N   |              |             |
|               |    |                  |               |      | RTC_50Hz    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_24 | 18 | UART_CTS         | WICED_GPIO_24 | A 11 | USART1_CTS  |              |             |
|               |    |                  |               |      | CAN1_RX     |              |             |
|               |    |                  |               |      | TIM1_ETR    |              |             |
|               |    |                  |               |      | OTG_FS_DM   |              |             |
|               |    |                  |               |      | EVENTOUT    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
| WICED_GPIO_25 | 19 | UART_RTS         | WICED_GPIO_25 | A 12 | USART1_RTS  |              |             |
|               |    |                  |               |      | CAN1_TX     |              |             |
|               |    |                  |               |      | TIM1_ETR    |              |             |
|               |    |                  |               |      | OTG_FS_DP   |              |             |
|               |    |                  |               |      | EVENTOUT    |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
Notes
1. These mappings are defined in <WICED-SDK>/Platform/ISM43362_M3G_L44/platform.c
2. STM32F2xx Datasheet  -> http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/CD00237391.pdf
3. STM32F2xx Ref Manual -> http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/REFERENCE_MANUAL/CD00225773.pdf
*/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_GPIO_1,
    WICED_GPIO_2,
    WICED_GPIO_3,
    WICED_GPIO_4,
    WICED_GPIO_5,
    WICED_GPIO_6,
    WICED_GPIO_7,
    WICED_GPIO_8,
    WICED_GPIO_9,
    WICED_GPIO_10,
    WICED_GPIO_11,
    WICED_GPIO_12,
    WICED_GPIO_13,
    WICED_GPIO_14,
    WICED_GPIO_15,
    WICED_GPIO_16,
    WICED_GPIO_17,
    WICED_GPIO_18,
    WICED_GPIO_19,
    WICED_GPIO_20,
    WICED_GPIO_21,
    WICED_GPIO_22,
    WICED_GPIO_23,
    WICED_GPIO_24,
    WICED_GPIO_25,
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    WICED_GPIO_32BIT = 0x7FFFFFFF,
} wiced_gpio_t;

typedef enum
{
    WICED_SPI_1,
    WICED_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    WICED_SPI_32BIT = 0x7FFFFFFF,
} wiced_spi_t;

typedef enum
{
    WICED_I2C_NONE = 0xFF,
    WICED_I2C_MAX,
    WICED_I2C_32BIT = 0x7FFFFFFF,
} wiced_i2c_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_3,
    WICED_PWM_4,
    WICED_PWM_5,
    WICED_PWM_6,
    WICED_PWM_7,
    WICED_PWM_8,
    WICED_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    WICED_PWM_32BIT = 0x7FFFFFFF,
} wiced_pwm_t;

typedef enum
{
    WICED_ADC_1,
    WICED_ADC_2,
    WICED_ADC_3,
    WICED_ADC_4,
    WICED_ADC_5,
    WICED_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    WICED_ADC_32BIT = 0x7FFFFFFF,
} wiced_adc_t;

typedef enum
{
    WICED_UART_1,
    WICED_UART_2,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    WICED_UART_32BIT = 0x7FFFFFFF,
} wiced_uart_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_2,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_PLATFORM_BUTTON_COUNT  ( 2 )

/* UART port used for standard I/O */
#define STDIO_UART ( WICED_UART_1 )

/* SPI flash is present */
//#define WICED_PLATFORM_INCLUDES_SPI_FLASH
#define WICED_SPI_FLASH_CS ( WICED_GPIO_5 )

/* Components connected to external I/Os */
#define WICED_LED1         ( WICED_GPIO_3 )
#define WICED_LED2         ( WICED_GPIO_18 )
#define WICED_LED1_ON_STATE  ( WICED_ACTIVE_HIGH )
#define WICED_LED2_ON_STATE  ( WICED_ACTIVE_HIGH )
#define WICED_BUTTON1      ( WICED_GPIO_2 )
#define WICED_BUTTON2      ( WICED_GPIO_11 )
#define WICED_THERMISTOR   ( WICED_GPIO_4 )

/* I/O connection <-> Peripheral Connections */
#define WICED_LED1_JOINS_PWM        ( WICED_PWM_4 )
#define WICED_LED2_JOINS_PWM        ( WICED_PWM_6 )
#define WICED_THERMISTOR_JOINS_ADC  ( WICED_ADC_3 )

/* Bootloader OTA/OTA2 LED to flash while "Factory Reset" button held           */
 #define PLATFORM_FACTORY_RESET_LED_GPIO              ( WICED_LED1 )
 #define PLATFORM_FACTORY_RESET_LED_ON_STATE          ( WICED_LED1_ON_STATE )

/* Bootloader OTA/OTA2 "Factory Reset" button */
 #define PLATFORM_FACTORY_RESET_BUTTON_GPIO      ( WICED_BUTTON1 )
 #define PLATFORM_FACTORY_RESET_PRESSED_STATE    (   0  )
 #define PLATFORM_FACTORY_RESET_CHECK_PERIOD     (  100 )
#ifndef PLATFORM_FACTORY_RESET_TIMEOUT
 #define PLATFORM_FACTORY_RESET_TIMEOUT          ( 5000 )
#endif
#ifdef __cplusplus
} /*extern "C" */
#endif
