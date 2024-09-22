#pragma once
/*----------------------------------------------------------------------------------*/
/* Edubard2 Defines                                                                 */
/* Martin Burger                                                                    */
/* Juventus Technikerschule                                                         */
/*----------------------------------------------------------------------------------*/

#ifdef EDUBOARD_CPU_BOARD_ESP32_S3

    #define GPIO_LED_0          4
    #define GPIO_LED_1          5
    #define GPIO_LED_2          6
    #define GPIO_LED_3          7
    #define GPIO_LED_4          38
    #define GPIO_LED_5          35
    #define GPIO_LED_6          37
    #define GPIO_LED_7          36

    #define GPIO_PWMLED_R       39
    #define GPIO_PWMLED_G       40
    #define GPIO_PWMLED_B       41

    #define GPIO_RotEnc_A       41
    #define GPIO_RotEnc_B       40
    #define GPIO_RotEnc_SW      39

    #define GPIO_WS2812         15

    #define GPIO_BUZZER         46

    #define GPIO_SW_0           21
    #define GPIO_SW_1           47
    #define GPIO_SW_2           48
    #define GPIO_SW_3           45

    #define GPIO_AN_0           9
    #define AN0_CHANNEL         ADC_CHANNEL_8

    #define GPIO_MISO           12
    #define GPIO_MOSI           13
    #define GPIO_SCK            14

    #define GPIO_UART0_TX       43
    #define GPIO_UART0_RX       44

    #define GPIO_UART1_TX       17
    #define GPIO_UART1_RX       18

    #define GPIO_RS485_DE       42

    #define GPIO_I2C_SDA        2
    #define GPIO_I2C_SCL        1

    #define GPIO_LCD_CS         11
    #define GPIO_LCD_DC         16

    #define GPIO_FLASH_DAC_CS   3
    #define GPIO_SD_LDAC_CS     10
    
    #define GPIO_GPIOX0         6
    #define GPIO_GPIOX1         7
    #define GPIO_GPIOX2         5
    #define GPIO_GPIOX3         4
    #define GPIO_GPIOX4         0

    #define GPIO_GENERAL_RESET  8

#endif

#ifdef CONFIG_ENABLE_LED0 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED1 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED2 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED3 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED4 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED5 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED6 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_LED7 
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_PWMLED
    #define CONFIG_ENABLE_LED
#endif
#ifdef CONFIG_ENABLE_WS2812
    #define CONFIG_ENABLE_LED
#endif

#ifdef CONFIG_ENABLE_SW0
    #define CONFIG_ENABLE_BUTTONS
#endif
#ifdef CONFIG_ENABLE_SW1
    #define CONFIG_ENABLE_BUTTONS
#endif
#ifdef CONFIG_ENABLE_SW2
    #define CONFIG_ENABLE_BUTTONS
#endif
#ifdef CONFIG_ENABLE_SW3
    #define CONFIG_ENABLE_BUTTONS
#endif

#ifdef CONFIG_ENABLE_ROTARYENCODER
    #define CONFIG_ENABLE_BUTTONS
#endif

#ifdef CONFIG_ENABLE_AN0
    #define CONFIG_ENABLE_ANALOG
#endif

#ifdef CONFIG_ENABLE_SENSOR_TMP112
    #define CONFIG_ENABLE_I2C
#endif
#ifdef CONFIG_ENABLE_SENSOR_STK8321
    #define CONFIG_ENABLE_I2C
#endif

#ifdef CONFIG_ENABLE_LCD
    
    #ifdef CONFIG_LCD_ILI9488
        #define SCREEN_ROTATION 90
        #define SCREEN_MAX_X 480
        #define SCREEN_MAX_Y 320
    #endif
    #ifdef CONFIG_LCD_ST7789
        #define SCREEN_ROTATION 0
        #ifdef CONFIG_LCD_RESOLUTION_240x240
            #define SCREEN_MAX_X 240
            #define SCREEN_MAX_Y 240
        #endif
        #ifdef CONFIG_LCD_RESOLUTION_240x320
            #define SCREEN_MAX_X 240
            #define SCREEN_MAX_Y 320
        #endif            
    #endif
    #ifndef SCREEN_ROTATION
        #define SCREEN_ROTATION 0
    #endif
    #ifdef CONFIG_USE_DIFFUPDATE
        #define CONFIG_USE_VSCREEN
    #endif
    #define CONFIG_ENABLE_SPIFFS
    #define CONFIG_ENABLE_SPI
#endif

#ifdef CONFIG_ENABLE_TOUCH_FT6236
    #define CONFIG_ENABLE_I2C
    #ifndef SCREEN_ROTATION
        #define SCREEN_ROTATION 0
    #endif
    #ifndef SCREEN_MAX_X
        #define SCREEN_MAX_X 480
        #define SCREEN_MAX_Y 320
    #endif
#endif

#ifndef SCREEN_ROTATION
    #define SCREEN_ROTATION 0
#endif
#ifndef SCREEN_MAX_X
    #define SCREEN_MAX_X 10
    #define SCREEN_MAX_Y 10
#endif

#ifdef CONFIG_ENABLE_FLASH
    #define CONFIG_ENABLE_LITTLEFS
    #define CONFIG_ENABLE_SPI
#endif
#ifdef CONFIG_ENABLE_DAC
    #ifdef CONFIG_ENABLE_FLASH
        #error "You cant enable Flash and DAC at the same time. CS are shared"
    #endif
    #ifdef CONFIG_ENABLE_SDCARD
        #error "You cant enable SDCARD and DAC at the same time. LDAC and SDCS are shared"
    #endif
    #ifdef CONFIG_DAC_STREAMING
        #ifdef CONFIG_ENABLE_SPI
            #error "Something else is already using Spi. DAC Double Buffering is only supported as the only SPI device on the Bus"
        #endif
    #endif
    #define CONFIG_ENABLE_SPI    
    #ifndef DAC_STREAM_SAMPLERATE
        #define DAC_STREAM_SAMPLERATE 100
    #else
        #if DAC_STREAM_SAMPLERATE < 50
            #error "DAC_STREAM_SAMPLERATE must be 50us at minimum"
        #endif
    #endif
#endif

#define EDUBOARD2_HWVERSION 2.2
#define EDUBOARD2_SWVERSION 0.1