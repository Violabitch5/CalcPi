#include "../../eduboard2.h"
#include "../eduboard2_button.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "Button_Driver"
#define BUTTON_UPDATE_TIME_MS       10

typedef struct {
    uint32_t count;
    button_state state;
    uint32_t timeout;
} button_data;

button_data buttons[4];
SemaphoreHandle_t buttondataLock;

uint8_t button_pins[] = {GPIO_SW_0, GPIO_SW_1, GPIO_SW_2, GPIO_SW_3};

uint32_t button_state_timeout_time = 5000;


bool checkButtonEnabled(uint8_t button_num) {
    switch(button_num) {
        case 0:
#ifndef CONFIG_ENABLE_SW0
            return false;
#endif
        break;
        case 1:
#ifndef CONFIG_ENABLE_SW1
            return false;
#endif
        break;
        case 2:
#ifndef CONFIG_ENABLE_SW2
            return false;
#endif
        break;
        case 3:
#ifndef CONFIG_ENABLE_SW3
            return false;
#endif
        break;
    }
    return true;
}

bool checkButton(uint8_t button_num) {
    if(checkButtonEnabled(button_num) == true) {
        return gpio_get_level(button_pins[button_num]);
    } else {
        return false;
    }
}

void buttonTask(void* arg) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;    
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
#ifdef CONFIG_ENABLE_SW0
    io_conf.pin_bit_mask |= (1ULL<<GPIO_SW_0);
#endif
#ifdef CONFIG_ENABLE_SW1
    io_conf.pin_bit_mask |= (1ULL<<GPIO_SW_1);
#endif
#ifdef CONFIG_ENABLE_SW2
    io_conf.pin_bit_mask |= (1ULL<<GPIO_SW_2);
#endif
#ifdef CONFIG_ENABLE_SW3
    io_conf.pin_bit_mask |= (1ULL<<GPIO_SW_3);
#endif
    gpio_config(&io_conf);
    for(;;) {
        xSemaphoreTake(buttondataLock, portMAX_DELAY);
        for(int i = 0; i < 4; i++) {
            if(checkButton(i) == true) {
                if(buttons[i].count <= BUTTONPRESS_LONG_MS / BUTTON_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
                    buttons[i].count++;
                }
            } else {
                if(buttons[i].count < BUTTONPRESS_SHORT_MS / BUTTON_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
                    buttons[i].count = 0;
                } else if(buttons[i].count < BUTTONPRESS_LONG_MS / BUTTON_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
                    buttons[i].state = SHORT_PRESSED;
                    buttons[i].count = 0;
                    buttons[i].timeout = button_state_timeout_time / BUTTON_UPDATE_TIME_MS / portTICK_PERIOD_MS;
                } else {
                    buttons[i].state = LONG_PRESSED;
                    buttons[i].count = 0;
                    buttons[i].timeout = button_state_timeout_time / BUTTON_UPDATE_TIME_MS / portTICK_PERIOD_MS;
                }
            }
            if(buttons[i].timeout > 0) {
                if(--buttons[i].timeout == 0) {
                    buttons[i].state = NOT_PRESSED;
                }
            }
        }
        xSemaphoreGive(buttondataLock);
        vTaskDelay(BUTTON_UPDATE_TIME_MS/portTICK_PERIOD_MS);
    }
}

button_state button_get_state(uint8_t button_num, bool reset) {
    button_state returnValue = NOT_PRESSED;
    if(buttondataLock == NULL) {
        return NOT_PRESSED;
    }
    xSemaphoreTake(buttondataLock, portMAX_DELAY);
    returnValue = buttons[button_num].state;
    if(reset == true) {
        buttons[button_num].state = NOT_PRESSED;
    }
    xSemaphoreGive(buttondataLock);
    return returnValue;
}

void eduboard_init_buttons() {    
    buttondataLock = xSemaphoreCreateMutex();
    xTaskCreate(buttonTask, "buttonTask", 2*2048, NULL, 10, NULL);
}

