#include "../../eduboard2.h"
#include "../eduboard2_buzzer.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define TAG "Buzzer_Driver"

#define BUZZER_TIMER              LEDC_TIMER_0
#define BUZZER_MODE               LEDC_LOW_SPEED_MODE
#define BUZZER_CHANNEL            LEDC_CHANNEL_4
#define BUZZER_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bit

struct {
    uint16_t frequency;
    uint32_t length_tick;
    uint32_t starttime_tick;
}  buzzer_stat;
uint32_t buzzer_volume = 4096; //Max Volume, 50% duty cycle

void buzzer_start(uint16_t freq_hz, uint32_t length_ms) {
    if(buzzer_stat.frequency > 0) {
        buzzer_stop();
    } 
    buzzer_stat.frequency = freq_hz;
    buzzer_stat.length_tick = length_ms/portTICK_PERIOD_MS;
    buzzer_stat.starttime_tick = xTaskGetTickCount();
    ledc_set_freq(BUZZER_MODE, BUZZER_TIMER, buzzer_stat.frequency);
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, buzzer_volume);
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}
void buzzer_stop() {
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
    buzzer_stat.frequency = 0;
    buzzer_stat.length_tick = 0;
    buzzer_stat.starttime_tick = 0;
}

void buzzerControlTask(void* arg) {
    uint32_t now;
    for(;;) {
        now = xTaskGetTickCount();
        if(now - buzzer_stat.starttime_tick > buzzer_stat.length_tick) {
            buzzer_stop();
        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void buzzer_set_volume(uint8_t volume) {
    volume = (volume > 100 ? 100:volume);
    buzzer_volume = (uint32_t)(4095.0 / 100.0 * volume);
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, buzzer_volume);
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}

void eduboard_init_buzzer() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_MODE,
        .timer_num        = BUZZER_TIMER,
        .duty_resolution  = BUZZER_DUTY_RES,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = BUZZER_MODE,
        .channel        = BUZZER_CHANNEL,
        .timer_sel      = BUZZER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_BUZZER,
        .duty           = 0, //Disable Buzzer
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    xTaskCreate(buzzerControlTask, "buzzerTask", 4*2048, NULL, 4, NULL);
}