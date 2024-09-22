#include "../../eduboard2.h"
#include "../eduboard2_rotary_encoder.h"
#include "driver/gpio.h"

#define TAG "RotaryEncoder_Driver"
#ifdef ROTARYENCODER_USE_INTERRUPTS
#define ROTARY_ENCODER_UPDATE_TIME_MS       10
#else
#define ROTARY_ENCODER_UPDATE_TIME_MS       1
#endif

#define ESP_INTR_FLAG_DEFAULT 0

typedef struct {
    uint32_t count;
    button_state state;
    uint32_t timeout;
} button_data;

button_data rotarybuttonstate;
SemaphoreHandle_t rotencdataLock;
uint32_t rotenc_state_timeout_time = 5000;

#ifdef ROTARYENCODER_USE_INTERRUPTS
static QueueHandle_t rotenc_evt_queue;
#endif

int32_t rotenc_position = 0;

typedef enum {
    ROTENC_NOEDGE,
    ROTENCA_POSEDGE,
    ROTENCB_POSEDGE,
    ROTENCA_NEGEDGE,
    ROTENCB_NEGEDGE,
} rotenc_event_t;

#ifdef ROTARYENCODER_USE_INTERRUPTS
static void IRAM_ATTR isr_rotenc_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    rotenc_event_t rotenc_evt = ROTENC_NOEDGE;
    switch(gpio_num) {
        case GPIO_RotEnc_A:
            if(gpio_get_level(GPIO_RotEnc_A) == true) {
                rotenc_evt = ROTENCA_POSEDGE;
            } else {
                rotenc_evt = ROTENCA_NEGEDGE;
            }
        break;
        case GPIO_RotEnc_B:
            if(gpio_get_level(GPIO_RotEnc_B) == true) {
                rotenc_evt = ROTENCB_POSEDGE;
            } else {
                rotenc_evt = ROTENCB_NEGEDGE;
            }
        break;
    }
    xQueueSendFromISR(rotenc_evt_queue, &rotenc_evt, NULL);
}
#endif

void initRotaryEncoderGPIOs() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;    
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL<<GPIO_RotEnc_SW);
    gpio_config(&io_conf);

    #ifdef ROTARYENCODER_USE_INTERRUPTS
    rotenc_evt_queue = xQueueCreate(100, sizeof(rotenc_event_t));

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    #else
    io_conf.intr_type = GPIO_INTR_DISABLE;
    #endif
    io_conf.pull_down_en = 0;    
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL<<GPIO_RotEnc_A);
    io_conf.pin_bit_mask |= (1ULL<<GPIO_RotEnc_B);
    gpio_config(&io_conf);

    #ifdef ROTARYENCODER_USE_INTERRUPTS
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_RotEnc_A, isr_rotenc_handler, (void*)GPIO_RotEnc_A);
    gpio_isr_handler_add(GPIO_RotEnc_B, isr_rotenc_handler, (void*)GPIO_RotEnc_B);
    #endif
}

void updateRotaryEncoderButton() {
    xSemaphoreTake(rotencdataLock, portMAX_DELAY);
    if(gpio_get_level(GPIO_RotEnc_SW) == true) {
        if(rotarybuttonstate.count <= BUTTONPRESS_LONG_MS / ROTARY_ENCODER_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
            rotarybuttonstate.count++;
        }
    } else {
        if(rotarybuttonstate.count < BUTTONPRESS_SHORT_MS / ROTARY_ENCODER_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
            rotarybuttonstate.count = 0;
        } else if(rotarybuttonstate.count < BUTTONPRESS_LONG_MS / ROTARY_ENCODER_UPDATE_TIME_MS / portTICK_PERIOD_MS) {
            rotarybuttonstate.state = SHORT_PRESSED;
            rotarybuttonstate.count = 0;
            rotarybuttonstate.timeout = rotenc_state_timeout_time / ROTARY_ENCODER_UPDATE_TIME_MS / portTICK_PERIOD_MS;
        } else {
            rotarybuttonstate.state = LONG_PRESSED;
            rotarybuttonstate.count = 0;
            rotarybuttonstate.timeout = rotenc_state_timeout_time / ROTARY_ENCODER_UPDATE_TIME_MS / portTICK_PERIOD_MS;
        }
    }
    if(rotarybuttonstate.timeout > 0) {
        if(--rotarybuttonstate.timeout == 0) {
            rotarybuttonstate.state = NOT_PRESSED;
        }
    }
    xSemaphoreGive(rotencdataLock);
}
void updateRotaryEncoder() {
    #ifdef ROTARYENCODER_USE_INTERRUPTS
    static rotenc_event_t lastEvent = ROTENC_NOEDGE;    
    rotenc_event_t rotenc_evt = ROTENC_NOEDGE;
    while(xQueueReceive(rotenc_evt_queue, &rotenc_evt, 0) == pdTRUE) {   

        // switch(rotenc_evt) {
        //     case ROTENCA_POSEDGE:                
        //     case ROTENCA_NEGEDGE:
        //         if((lastEvent == ROTENCA_POSEDGE) || (lastEvent == ROTENCA_NEGEDGE)) {                    
        //             lastEvent = rotenc_evt;
        //             continue;
        //         }
        //     break;
        //     case ROTENCB_POSEDGE:
        //     case ROTENCB_NEGEDGE:
        //         if((lastEvent == ROTENCB_POSEDGE) || (lastEvent == ROTENCB_NEGEDGE)) {
        //             lastEvent = rotenc_evt;
        //             continue;
        //         }
        //     break;
        //     default:
        //         ESP_LOGI(TAG, "WTF");
        //     break;
        // }
        // lastEvent = rotenc_evt;

        xSemaphoreTake(rotencdataLock, portMAX_DELAY);                
        switch(rotenc_evt) {
            case ROTENCA_POSEDGE:
                if(lastB == false) {
                    rotenc_position++;
                } else {
                    rotenc_position--;
                }
                lastA = true;
            break;
            case ROTENCA_NEGEDGE:
                if(lastB == true) {
                    rotenc_position++;
                } else {
                    rotenc_position--;
                }
                lastA = false;
            break;
            case ROTENCB_POSEDGE:
                if(lastA == true) {
                    rotenc_position++;
                } else {
                    rotenc_position--;
                }
                lastB = true;
            break;
            case ROTENCB_NEGEDGE:
                if(lastA == false) {
                    rotenc_position++;
                } else {
                    rotenc_position--;
                }
                lastB = false;
            break;
            default:

            break;
        }
        xSemaphoreGive(rotencdataLock);
    }
    #else
    bool rotencA = gpio_get_level(GPIO_RotEnc_A);
    bool rotencB = gpio_get_level(GPIO_RotEnc_B);
    xSemaphoreTake(rotencdataLock, portMAX_DELAY);
    #ifdef ROTARYENCODER_USE_EDGEA
    static bool lastA = false;
    if(rotencA != lastA) {
        if(rotencA == true) {
            if(rotencB == false) {
                rotenc_position++;
            } else {
                rotenc_position--;
            }
        } else {
            if(rotencB == true) {
                rotenc_position++;
            } else {
                rotenc_position--;
            }
        }
    }
    #endif
    #ifdef ROTARYENCODER_USE_EDGEB
    static bool lastB = false;
    if(rotencB != lastB) {
        if(rotencB == true) {
            if(rotencA == true) {
                rotenc_position++;
            } else {
                rotenc_position--;
            }
        } else {
            if(rotencA == false) {
                rotenc_position++;
            } else {
                rotenc_position--;
            }
        }
    }
    #endif
    xSemaphoreGive(rotencdataLock);
    #ifdef ROTARYENCODER_USE_EDGEA
    lastA = rotencA;
    #endif
    #ifdef ROTARYENCODER_USE_EDGEB
    lastB = rotencB;
    #endif
    #endif
}

void rotaryEncoderTask(void* param) {
    ESP_LOGI(TAG, "Init Rotary Encoder");
    initRotaryEncoderGPIOs();
    for(;;) {
        updateRotaryEncoderButton();
        updateRotaryEncoder();
        vTaskDelay(ROTARY_ENCODER_UPDATE_TIME_MS/portTICK_PERIOD_MS);
    }
}

button_state rotary_encoder_button_get_state(bool reset) {
    button_state returnValue = NOT_PRESSED;
    if(rotencdataLock == NULL) {
        return NOT_PRESSED;
    }
    xSemaphoreTake(rotencdataLock, portMAX_DELAY);
    returnValue = rotarybuttonstate.state;
    if(reset == true) {
        rotarybuttonstate.state = NOT_PRESSED;
    }
    xSemaphoreGive(rotencdataLock);
    return returnValue;
}
int32_t rotary_encoder_get_rotation(bool reset) {
    int32_t returnvalue = 0;
    if(rotencdataLock == NULL) {
        return NOT_PRESSED;
    }
    xSemaphoreTake(rotencdataLock, portMAX_DELAY);
    returnvalue = rotenc_position;
    if(reset == true) {
        rotenc_position = 0;
    }
    xSemaphoreGive(rotencdataLock);
    return returnvalue;
}
void eduboard_init_rotary_encoder() {
    rotencdataLock = xSemaphoreCreateMutex();
    xTaskCreate(rotaryEncoderTask, "rotenc_task", 2*2048, NULL, 10, NULL);
}