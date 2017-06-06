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
 * Defines board support package for ISM43362_M3G_L44 board
 */
#include "platform.h"
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "wwd_platform_common.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "platform_button.h"
#include "gpio_button.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define PLATFORM_FACTORY_RESET_CHECK_PERIOD     ( 100 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 5000 )

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_gpio_t platform_gpio_pins[] =
{									/* eS-WiFi Signal, EVB Function 		L44 Pin */
    [WICED_GPIO_1]  = { GPIOA,  0 },/* WKUP 								(16) */
    [WICED_GPIO_2]  = { GPIOB,  5 },/* GPIO0, Button1						(23) */
    [WICED_GPIO_11] = { GPIOB,  6 },/* GPIO1, Button2 						(24) */
    [WICED_GPIO_12] = { GPIOB,  7 },/* GPIO2 								(25) */
    [WICED_GPIO_3]  = { GPIOB,  8 },/* GPIO3, LED1 							(26) */
    [WICED_GPIO_18] = { GPIOB,  9 },/* GPIO4, LED2 							(27) */
    [WICED_GPIO_4]  = { GPIOA,  3 },/* ADC0/SPI_DATARDY 					(13) */
    [WICED_GPIO_5]  = { GPIOA,  4 },/* ADC1/SPI_SSN 						(12) */
    [WICED_GPIO_6]  = { GPIOA,  5 },/* ADC2/SPI_SCK 						(11) */
    [WICED_GPIO_7]  = { GPIOA,  6 },/* ADC3/SPI_MISO 						(10) */
    [WICED_GPIO_8]  = { GPIOA,  7 },/* ADC4/SPI_MOSI 						( 9) */
    [WICED_GPIO_9]  = { GPIOA,  9 },/* UART1_TX 							(22) */
    [WICED_GPIO_10] = { GPIOA, 10 },/* UART1_RX 							(21) */
    [WICED_GPIO_13] = { GPIOA, 13 },/* TMS 									( 4) */
    [WICED_GPIO_14] = { GPIOA, 14 },/* TCK 									( 5) */
    [WICED_GPIO_15] = { GPIOA, 15 },/* TDI 									( 6) */
    [WICED_GPIO_16] = { GPIOB,  3 },/* TDO 									( 7) */
    [WICED_GPIO_17] = { GPIOB,  4 },/* TRSTN 								( 8) */
    [WICED_GPIO_19] = { GPIOB, 10 },/* CFG0/USART3_TX 						(28) */
    [WICED_GPIO_20] = { GPIOB, 11 },/* CFG1/USART3_RX 						(29) */
    [WICED_GPIO_21] = { GPIOB, 13 },/* GPIO13/USART3_CTS 					(32) */
    [WICED_GPIO_22] = { GPIOB, 14 },/* GPIO14/USART3_RTS 					(31) */
    [WICED_GPIO_23] = { GPIOB, 15 },/* GPIO15 								(30) */
    [WICED_GPIO_24] = { GPIOA, 11 },/* UART1_CTS/USB DM						(19) */
    [WICED_GPIO_25] = { GPIOA, 12 },/* UART1_RTS/USB_DP						(18) */
};

/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] = {ADC1, ADC_Channel_3, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_4]}, /* PA3, ADC0 */
    [WICED_ADC_2] = {ADC1, ADC_Channel_4, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_5]}, /* PA4, ADC1 */
    [WICED_ADC_3] = {ADC1, ADC_Channel_5, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_6]}, /* PA5, ADC2 */
    [WICED_ADC_4] = {ADC1, ADC_Channel_6, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_7]}, /* PA6, ADC3 */
    [WICED_ADC_5] = {ADC1, ADC_Channel_7, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_8]}, /* PA7, ADC4 */
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {TIM4, 1, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_11]}, /* PB6,  GPIO1,  TIM4_CH1  */
    [WICED_PWM_2]  = {TIM4, 2, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_12]}, /* PB7,  GPIO2,  TIM4_CH2  */
    [WICED_PWM_3]  = {TIM3, 2, RCC_APB1Periph_TIM3,  GPIO_AF_TIM3,  &platform_gpio_pins[WICED_GPIO_2 ]}, /* PB5,  GPIO0,  TIM3_CH2  */
    [WICED_PWM_4]  = {TIM4, 3, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_3 ]}, /* PB8,  GPIO3,  TIM4_CH3  */
    [WICED_PWM_5]  = {TIM4, 4, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_18]}, /* PB9,  GPIO4 , TIM4_CH4  */
    [WICED_PWM_6]  = {TIM1, 1, RCC_APB2Periph_TIM1,  GPIO_AF_TIM1,  &platform_gpio_pins[WICED_GPIO_21]}, /* PB13, GPIO13, TIM1_CH1N */
    [WICED_PWM_7]  = {TIM12,1, RCC_APB1Periph_TIM12, GPIO_AF_TIM12, &platform_gpio_pins[WICED_GPIO_22]}, /* PB14, GPIO14, TIM12_CH1 */
    [WICED_PWM_8]  = {TIM12,2, RCC_APB1Periph_TIM12, GPIO_AF_TIM12, &platform_gpio_pins[WICED_GPIO_23]}, /* PB15, GPIO15, TIM12_CH2 */
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1]  =
    {
        .port                  = SPI1,
        .gpio_af               = GPIO_AF_SPI1,
        .peripheral_clock_reg  = RCC_APB2Periph_SPI1,
        .peripheral_clock_func = RCC_APB2PeriphClockCmd,
        .pin_mosi              = &platform_gpio_pins[WICED_GPIO_8], /* PA7, SPI_MOSI */
        .pin_miso              = &platform_gpio_pins[WICED_GPIO_7], /* PA6, SPI_MISI */
        .pin_clock             = &platform_gpio_pins[WICED_GPIO_6], /* PA5, SPI_SCK  */
        .tx_dma =
        {
            .controller        = DMA2,
            .stream            = DMA2_Stream5,
            .channel           = DMA_Channel_3,
            .irq_vector        = DMA2_Stream5_IRQn,
            .complete_flags    = DMA_HISR_TCIF5,
            .error_flags       = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
        },
        .rx_dma =
        {
            .controller        = DMA2,
            .stream            = DMA2_Stream0,
            .channel           = DMA_Channel_3,
            .irq_vector        = DMA2_Stream0_IRQn,
            .complete_flags    = DMA_LISR_TCIF0,
            .error_flags       = ( DMA_LISR_TEIF0 | DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 ),
        },
    }
};

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port               = USART1,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_9 ], /* PA9,  TX */
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_10], /* PA10, RX */
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_24], /* PA11, CTS */
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_25], /* PA12, RTS */
        .tx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream7,
            .channel        = DMA_Channel_4,
            .irq_vector     = DMA2_Stream7_IRQn,
            .complete_flags = DMA_HISR_TCIF7,
            .error_flags    = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 | DMA_HISR_DMEIF7 ),
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream2,
            .channel        = DMA_Channel_4,
            .irq_vector     = DMA2_Stream2_IRQn,
            .complete_flags = DMA_LISR_TCIF2,
            .error_flags    = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
        },
    },
    [WICED_UART_2] =
    {
        .port               = USART3,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_19], /* PB10, USART3_TX  */
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_20], /* PB11, USART3_RX  */
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_21], /* PB13, USART3_CTS */
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_22], /* PB14, USART3_RTS */
        .tx_dma_config =
        {
            .controller     = DMA1,
            .stream         = DMA1_Stream4,
            .channel        = DMA_Channel_7,
            .irq_vector     = DMA1_Stream4_IRQn,
            .complete_flags = DMA_HISR_TCIF4,
            .error_flags    = ( DMA_HISR_TEIF4 | DMA_HISR_FEIF4 | DMA_HISR_DMEIF4 ),
        },
        .rx_dma_config =
        {
            .controller     = DMA1,
            .stream         = DMA1_Stream1,
            .channel        = DMA_Channel_4,
            .irq_vector     = DMA1_Stream1_IRQn,
            .complete_flags = DMA_LISR_TCIF1,
            .error_flags    = ( DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 ),
        },
    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* SPI flash. Exposed to the applications through include/wiced_platform.h */
#if defined ( WICED_PLATFORM_INCLUDES_SPI_FLASH )
const wiced_spi_device_t wiced_spi_flash =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_SPI_FLASH_CS,
    .speed       = 5000000,
    .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_NO_DMA | SPI_MSB_FIRST),
    .bits        = 8
};
#endif

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
#ifndef ENSTDIOFC
    .flow_control = FLOW_CONTROL_DISABLED,
#else
    .flow_control = FLOW_CONTROL_CTS_RTS,
#endif
};
#endif

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c
 * SDIO: WWD_PIN_BOOTSTRAP[1:0] = b'00
 * gSPI: WWD_PIN_BOOTSTRAP[1:0] = b'01
 */
const platform_gpio_t wifi_control_pins[] =
{
#ifdef WICED_USE_WIFI_POWER_PIN
    //Power pin unavailable on ISM43362_M3G_L44 module
	[WWD_PIN_POWER      ] = { GPIOB,  2 },
#endif
	[WWD_PIN_RESET      ] = { GPIOB,  2 },
#if defined ( WICED_USE_WIFI_32K_CLOCK_MCO )
    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
#else
    [WWD_PIN_32K_CLK    ] = { GPIOA, 11 },
#endif
    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  0 },
    [WWD_PIN_BOOTSTRAP_1] = { GPIOB,  1 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOB,  0 },
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },
    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },
    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },
    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },
};

/* Wi-Fi gSPI bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SPI.c */
const platform_gpio_t wifi_spi_pins[] =
{
    [WWD_PIN_SPI_IRQ ] = { GPIOC,  9 },
    [WWD_PIN_SPI_CS  ] = { GPIOC, 11 },
    [WWD_PIN_SPI_CLK ] = { GPIOB, 13 },
    [WWD_PIN_SPI_MOSI] = { GPIOB, 15 },
    [WWD_PIN_SPI_MISO] = { GPIOB, 14 },
};

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON2,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
    /* Interrupt priority setup. Called by WICED/platform/MCU/STM32F2xx/platform_init.c */
    NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
    NVIC_SetPriority( SDIO_IRQn        ,  2 ); /* WLAN SDIO           */
    NVIC_SetPriority( DMA2_Stream3_IRQn,  3 ); /* WLAN SDIO DMA       */
    NVIC_SetPriority( DMA1_Stream3_IRQn,  3 ); /* WLAN SPI DMA        */
    NVIC_SetPriority( USART1_IRQn      ,  6 ); /* WICED_UART_1        */
    NVIC_SetPriority( USART2_IRQn      ,  6 ); /* WICED_UART_2        */
    NVIC_SetPriority( DMA2_Stream5_IRQn,  6 ); /* WICED_SPI_2_TX      */
    NVIC_SetPriority( DMA2_Stream0_IRQn,  6 ); /* WICED_SPI_2_RX      */
    NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* WICED_UART_1 TX DMA */
    NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* WICED_UART_1 RX DMA */
    NVIC_SetPriority( DMA1_Stream6_IRQn,  7 ); /* WICED_UART_2 TX DMA */
    NVIC_SetPriority( DMA1_Stream5_IRQn,  7 ); /* WICED_UART_2 RX DMA */
    NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
}

void platform_init_external_devices( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
    platform_gpio_output_low( &platform_gpio_pins[WICED_LED1] );
    platform_gpio_output_low( &platform_gpio_pins[WICED_LED2] );

    /* Initialise buttons to input by default */
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_PULL_UP );
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON2], INPUT_PULL_UP );

#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    uint32_t button_press_timer = 0;
    int led_state = 0;

    /* Initialise input */
     platform_gpio_init( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_BUTTON_GPIO ], INPUT_PULL_UP );

     while ( (PLATFORM_FACTORY_RESET_PRESSED_STATE == platform_gpio_input_get(&platform_gpio_pins[ PLATFORM_FACTORY_RESET_BUTTON_GPIO ])) )
    {
         /* How long is the "Factory Reset" button being pressed. */
         host_rtos_delay_milliseconds( PLATFORM_FACTORY_RESET_CHECK_PERIOD );

         /* Toggle LED every PLATFORM_FACTORY_RESET_CHECK_PERIOD  */
        if ( led_state == 0 )
        {
            platform_gpio_output_high( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
            led_state = 1;
        }
        else
        {
            platform_gpio_output_low( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
            led_state = 0;
        }

        button_press_timer += PLATFORM_FACTORY_RESET_CHECK_PERIOD;
        if ((max_time > 0) && (button_press_timer >= max_time))
        {
            break;
        }
    }

     /* turn off the LED */
     if (PLATFORM_FACTORY_RESET_LED_ON_STATE == WICED_ACTIVE_HIGH)
     {
         platform_gpio_output_low( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
     }
     else
     {
         platform_gpio_output_high( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
     }

    return button_press_timer;
}

/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR( usart1_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart3_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_2] );
}

WWD_RTOS_DEFINE_ISR( usart1_tx_dma_irq )
{
    platform_uart_tx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart3_tx_dma_irq )
{
    platform_uart_tx_dma_irq( &platform_uart_drivers[WICED_UART_2] );
}

WWD_RTOS_DEFINE_ISR( usart1_rx_dma_irq )
{
    platform_uart_rx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart3_rx_dma_irq )
{
    platform_uart_rx_dma_irq( &platform_uart_drivers[WICED_UART_2] );
}

/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

/* These DMA assignments can be found STM32F2xx datasheet DMA section */
WWD_RTOS_MAP_ISR( usart1_irq       , USART1_irq       )
WWD_RTOS_MAP_ISR( usart1_tx_dma_irq, DMA2_Stream7_irq )
WWD_RTOS_MAP_ISR( usart1_rx_dma_irq, DMA2_Stream2_irq )
WWD_RTOS_MAP_ISR( usart3_irq       , USART3_irq       )
WWD_RTOS_MAP_ISR( usart3_tx_dma_irq, DMA1_Stream4_irq )
WWD_RTOS_MAP_ISR( usart3_rx_dma_irq, DMA1_Stream1_irq )

