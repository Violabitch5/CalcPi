#include "../../eduboard2.h"
#include "../eduboard2_led.h"
#include "driver/gpio.h"
#ifdef CONFIG_ENABLE_PWMLED
#include "driver/ledc.h"
#endif
#ifdef CONFIG_ENABLE_WS2812
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#endif

#define TAG "LED_Driver"



uint8_t led_pins[] = {GPIO_LED_0, GPIO_LED_1, GPIO_LED_2, GPIO_LED_3, GPIO_LED_4, GPIO_LED_5, GPIO_LED_6, GPIO_LED_7};

uint8_t checkLEDEnabled(uint8_t led_num) {
    switch(led_num) {
        case 0:
#ifndef CONFIG_ENABLE_LED0
            return 0;
#endif
        break;
        case 1:
#ifndef CONFIG_ENABLE_LED1
            return 0;
#endif
        break;
        case 2:
#ifndef CONFIG_ENABLE_LED2
            return 0;
#endif
        break;
        case 3:
#ifndef CONFIG_ENABLE_LED3
            return 0;
#endif
        break;
        case 4:
#ifndef CONFIG_ENABLE_LED4
            return 0;
#endif
        break;
        case 5:
#ifndef CONFIG_ENABLE_LED5
            return 0;
#endif
        break;
        case 6:
#ifndef CONFIG_ENABLE_LED6
            return 0;
#endif
        break;
        case 7:
#ifndef CONFIG_ENABLE_LED7
            return 0;
#endif
        break;
    }
    return 1;
}

SemaphoreHandle_t ledLock;
uint8_t ledValue = 0;

void updateLedValues() {
    for(int i = 0; i < 8; i++) {
        uint8_t level = (ledValue >> i) & 0x01;
        uint8_t ledPin = led_pins[i];
        gpio_set_level(ledPin, level);
    }
}

void led_set(uint8_t led_num, uint8_t level) {
    if(checkLEDEnabled(led_num) == 0) {
        ESP_LOGE(TAG, "LED%i not Enabled", led_num);
        return;
    }
    if(xSemaphoreTake(ledLock, 10)) {
        if(level > 0) {
            ledValue |= (0x01 << led_num);
        } else {
            ledValue &= ~(0x01 << led_num);
        }
        updateLedValues();
        xSemaphoreGive(ledLock);
    }
}
void led_toggle(uint8_t led_num) {
    if(checkLEDEnabled(led_num) == 0) {
        ESP_LOGE(TAG, "LED%i not Enabled", led_num);
        return;
    }
    if(xSemaphoreTake(ledLock, 10)) {
        ledValue ^= (0x01 << led_num);
        updateLedValues();
        xSemaphoreGive(ledLock);
    }
}
void led_setAll(uint8_t newLedValue) {
    if(xSemaphoreTake(ledLock, 10)) {
        ledValue = newLedValue;
        updateLedValues();
        xSemaphoreGive(ledLock);
    }
}
#ifdef CONFIG_ENABLE_PWMLED
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_R        GPIO_PWMLED_R // Define the output GPIO
#define LEDC_OUTPUT_IO_G        GPIO_PWMLED_G // Define the output GPIO
#define LEDC_OUTPUT_IO_B        GPIO_PWMLED_B // Define the output GPIO
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
void pwmled_set(uint8_t red, uint8_t green, uint8_t blue) {
    red = (red > 100 ? 100 : red);
    green = (green > 100 ? 100 : green);
    blue = (blue > 100 ? 100 : blue);
    uint32_t duty = (0x1FFF / 100.0) * red;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, duty));    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));
    duty = (0x1FFF / 100.0) * green;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, duty));    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G));
    duty = (0x1FFF / 100.0) * blue;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty));    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B));
}
void init_pwm_led() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_r = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_R,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_R,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_g = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_G,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_G,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_B,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_r));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_g));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));
}
#endif

#ifdef CONFIG_ENABLE_WS2812
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
static uint8_t led_strip_pixels[1 * 3];
void ws2812_set(uint8_t red, uint8_t green, uint8_t blue) {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    led_strip_pixels[0] = green;
    led_strip_pixels[1] = red;
    led_strip_pixels[2] = blue;
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
}
void init_ws2812() {
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = GPIO_WS2812,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}
#endif
void eduboard_init_leds() {
    ledLock = xSemaphoreCreateMutex();
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
#ifdef CONFIG_ENABLE_LED0
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_0);
#endif
#ifdef CONFIG_ENABLE_LED1
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_1);
#endif
#ifdef CONFIG_ENABLE_LED2
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_2);
#endif
#ifdef CONFIG_ENABLE_LED3
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_3);
#endif
#ifdef CONFIG_ENABLE_LED4
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_4);
#endif
#ifdef CONFIG_ENABLE_LED5
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_5);
#endif
#ifdef CONFIG_ENABLE_LED6
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_6);
#endif
#ifdef CONFIG_ENABLE_LED7
    io_conf.pin_bit_mask |= (1ULL<<GPIO_LED_7);
#endif
    if(io_conf.pin_bit_mask > 0) {
        gpio_config(&io_conf);
    }
    #ifdef CONFIG_ENABLE_PWMLED
    #ifdef CONFIG_ENABLE_ROTARYENCODER
    ESP_LOGE(TAG, "PWM-LED and RotaryEncoder are enabled at the same time. Pins are shared.");
    #endif
    init_pwm_led();
    #endif
    #ifdef CONFIG_ENABLE_WS2812
    init_ws2812();
    #endif
}

