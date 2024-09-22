#include "../../eduboard2.h"
#include "../eduboard2_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#define TAG "Eduboard2_Init"

#ifdef CONFIG_ENABLE_LED
    #include "eduboardLED/eduboard2_led.h"
#endif

#ifdef CONFIG_ENABLE_BUZZER
    #include "eduboardBuzzer/eduboard2_buzzer.h"
#endif

#ifdef CONFIG_ENABLE_BUTTONS
    #include "eduboardButton/eduboard2_button.h"
#endif

#ifdef CONFIG_ENABLE_ROTARYENCODER
    #include "eduboardRotaryEncoder/eduboard2_rotary_encoder.h"
#endif

#ifdef CONFIG_ENABLE_ANALOG
    #include "eduboardADC/eduboard2_adc.h"
#endif

#ifdef CONFIG_ENABLE_SPIFFS
    #include "eduboardSpiffs/eduboard2_spiffs.h"
#endif

#ifdef CONFIG_ENABLE_LCD
    #include "eduboardLCD/eduboard2_lcd.h"
#endif
#ifdef CONFIG_ENABLE_FLASH
    #include "eduboardFlash/eduboard2_flash.h"
#endif
#ifdef CONFIG_ENABLE_DAC
    #include "eduboardDAC/eduboard2_dac.h"
#endif

#ifdef CONFIG_ENABLE_SENSOR_TMP112
    #include "eduboardSensor/eduboard2_sensor.h"
#endif
#ifdef CONFIG_ENABLE_SENSOR_STK8321
    #include "eduboardSensor/eduboard2_sensor.h"
#endif
#ifdef CONFIG_ENABLE_TOUCH_FT6236
    #include "eduboardTouch/eduboard2_touch.h"
#endif
#ifdef CONFIG_ENABLE_RTC
    #include "eduboardRTC/eduboard2_rtc.h"
#endif

#ifdef CONFIG_ENABLE_I2C
    #include "gpi2c.h"
#endif
#ifdef CONFIG_ENABLE_SPI
    #include "gpspi.h"
#endif

SemaphoreHandle_t sem_initdone;

void eduboard2_initTask(void* param) {
    #ifdef CONFIG_ENABLE_LED
    eduboard_init_leds();
    #endif

    #ifdef CONFIG_ENABLE_BUZZER
    eduboard_init_buzzer();
    buzzer_set_volume(3);
    #endif

    #ifdef CONFIG_ENABLE_BUTTONS
    eduboard_init_buttons();
    #endif

    #ifdef CONFIG_ENABLE_ROTARYENCODER
    eduboard_init_rotary_encoder();
    #endif

    #ifdef CONFIG_ENABLE_ANALOG
    eduboard_init_adc();
    #endif

    #ifdef CONFIG_ENABLE_SPIFFS
    eduboard_init_spiffs();
    #endif

    #ifdef CONFIG_ENABLE_FLASH
    eduboard_init_flash();
    #endif
    #ifdef CONFIG_ENABLE_DAC
    eduboard_init_dac();
    #endif
    
    #ifdef CONFIG_ENABLE_LCD
    eduboard_init_lcd();
    #endif

    #ifdef CONFIG_ENABLE_SENSOR_TMP112
    eduboard_init_tmp112();
    #endif
    #ifdef CONFIG_ENABLE_SENSOR_STK8321
    eduboard_init_stk8321();
    #endif
    #ifdef CONFIG_ENABLE_TOUCH_FT6236
    eduboard_init_ft6236();
    #endif
    #ifdef CONFIG_ENABLE_RTC
    eduboard_init_rtc();
    #endif
    xSemaphoreGive(sem_initdone);
    vTaskDelete(NULL);
}

void eduboard2_init() {
    sem_initdone = xSemaphoreCreateBinary();
    ESP_LOGI(TAG, "Init Eduboard2...");
    xTaskCreate(eduboard2_initTask, "init_task", 4*2048, NULL, 10, NULL);
    xSemaphoreTake(sem_initdone, portMAX_DELAY);
    ESP_LOGI(TAG, "Init Eduboard2 done");
    vSemaphoreDelete(sem_initdone);
}