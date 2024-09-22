#include "../../eduboard2.h"
#include "../eduboard2_dac.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/*************************************************************************
*               Driver for SPI DAC MCP4802-E/SN
*
*
*************************************************************************/

#define DAC_FREQ_MHZ      SPI_MASTER_FREQ_20M

#define DAC_A_BASE              0x0000
#define DAC_B_BASE              0x8000
#define DAC_CONFIG_GAIN_1       0x2000
#define DAC_CONFIG_GAIN_2       0x0000
#define DAC_CONFIG_OUTPUT_ON    0x1000
#define DAC_CONFIG_OUTPUT_OFF   0x0000

#ifdef CONFIG_DAC_STREAMING
typedef struct {
    uint8_t dacA[CONFIG_DAC_STREAMING_BUFFERSIZE];
    uint8_t dacB[CONFIG_DAC_STREAMING_BUFFERSIZE];
} dac_stream_data_t;
void (*streamCallbackFunction)() = NULL;
uint32_t dacPos = 0;
uint8_t selectedBuffer = 0;
dac_stream_data_t dacBufferA, dacBufferB;
EventGroupHandle_t dacStreamControl;
#define EG_STREAMCONTROL_BUFFERA_EMPTY  1 << 0
#define EG_STREAMCONTROL_BUFFERB_EMPTY  1 << 1
QueueHandle_t dacstreamqueue;
SemaphoreHandle_t dacBufferSwitchSignal;
#endif


typedef struct {
    bool enabled;
    dac_gain_t gain;
    uint8_t value;
} dac_data_t;

SemaphoreHandle_t dacLock;

dac_data_t dacData[2];

#define TAG "DAC_driver"

spi_device_handle_t dev_dac_spi;

void dac_set_config(dac_num_t dacNum, dac_gain_t gain, bool enabled) {
    xSemaphoreTake(dacLock, portMAX_DELAY);
    dacData[dacNum].gain = gain;
    dacData[dacNum].enabled = enabled;
    xSemaphoreGive(dacLock);
}

void dac_set_value(dac_num_t dacNum, uint8_t value) {
#ifndef CONFIG_DAC_STREAMING
    xSemaphoreTake(dacLock, portMAX_DELAY);
    dacData[dacNum].value = value;
    xSemaphoreGive(dacLock);
#endif
}

void dac_update() {
#ifndef CONFIG_DAC_STREAMING
    uint16_t dataA = DAC_A_BASE;
    uint16_t dataB = DAC_B_BASE;
    xSemaphoreTake(dacLock, portMAX_DELAY);
    if(dacData[DAC_A].enabled) dataA |= DAC_CONFIG_OUTPUT_ON;
    if(dacData[DAC_B].enabled) dataB |= DAC_CONFIG_OUTPUT_ON;
    if(dacData[DAC_A].gain) dataA |= DAC_CONFIG_GAIN_1;
    if(dacData[DAC_B].gain) dataB |= DAC_CONFIG_GAIN_1;
    dataA |= ((dacData[DAC_A].value & 0x00FF)<<4);
    dataB |= ((dacData[DAC_B].value & 0x00FF)<<4);
    xSemaphoreGive(dacLock);
    uint8_t data[2];
    data[0] = dataA >> 8;
    data[1] = dataA & 0xFF;
    gpspi_write_data(&dev_dac_spi, data, 2);
    data[0] = dataB >> 8;
    data[1] = dataB & 0xFF;
    gpspi_write_data(&dev_dac_spi, data, 2);
    gpio_set_level(GPIO_SD_LDAC_CS, 0);
    vTaskDelay(1);
    gpio_set_level(GPIO_SD_LDAC_CS, 1);
#endif
}
void dac_load_stream_data(uint8_t* dataA, uint8_t* dataB) {
#ifdef CONFIG_DAC_STREAMING
    dac_stream_data_t data;
    for(int i = 0; i < CONFIG_DAC_STREAMING_BUFFERSIZE; i++) {
        data.dacA[i] = dataA[i];
        data.dacB[i] = dataB[i];
    }
    xQueueSend(dacstreamqueue, &data, portMAX_DELAY);
#else
    printf("DAC Streaming must be enabled for this Function to work\n");
#endif
}
#ifdef CONFIG_DAC_STREAMING
static void periodic_dac_callback(void* arg) {
    gpio_set_level(GPIO_SD_LDAC_CS, 0);
    gpio_set_level(GPIO_SD_LDAC_CS, 1); 
    uint8_t valueA, valueB;
    if(selectedBuffer == 0) {
        valueA = dacBufferA.dacA[dacPos];
        valueB = dacBufferA.dacB[dacPos];
    } else {
        valueA = dacBufferB.dacA[dacPos];
        valueB = dacBufferB.dacB[dacPos];
    }
    if(++dacPos >= CONFIG_DAC_STREAMING_BUFFERSIZE) {
        if(selectedBuffer == 0) {
            xEventGroupSetBitsFromISR(dacStreamControl, EG_STREAMCONTROL_BUFFERA_EMPTY, NULL);
            selectedBuffer = 1;
        } else {
            xEventGroupSetBitsFromISR(dacStreamControl, EG_STREAMCONTROL_BUFFERB_EMPTY, NULL);
            selectedBuffer = 0;
        }
        xSemaphoreGiveFromISR(dacBufferSwitchSignal, NULL);
        dacPos = 0;
    }
    uint8_t dataA[2] = {(0x00 | (dacData[DAC_A].enabled ? (DAC_CONFIG_OUTPUT_ON>>8) : 0) | (dacData[DAC_A].gain ? (DAC_CONFIG_GAIN_1>>8) : 0) | (valueA >> 4)), valueA << 4};
    uint8_t dataB[2] = {(0x80 | (dacData[DAC_A].enabled ? (DAC_CONFIG_OUTPUT_ON>>8) : 0) | (dacData[DAC_A].gain ? (DAC_CONFIG_GAIN_1>>8) : 0) | (valueB >> 4)), valueB << 4};
    gpspi_write_data_nonblocking(&dev_dac_spi, dataA, 2);
    gpspi_write_data_nonblocking(&dev_dac_spi, dataB, 2);
    
}

void dac_bufferManager(void* param) {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_dac_callback,
        .name = "periodic_dac"
    };
    esp_timer_handle_t periodic_dac_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_dac_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_dac_timer, DAC_STREAM_SAMPLERATE));
    for(;;) {
        xSemaphoreTake(dacBufferSwitchSignal, portMAX_DELAY);
        dac_stream_data_t data;
        if(xQueueReceive(dacstreamqueue, &data, 0) == pdTRUE) {
            if(xEventGroupGetBits(dacStreamControl) & EG_STREAMCONTROL_BUFFERA_EMPTY){
                for(int i = 0; i < CONFIG_DAC_STREAMING_BUFFERSIZE; i++) {
                    dacBufferA.dacA[i] = data.dacA[i];
                    dacBufferA.dacB[i] = data.dacB[i];
                }
                xEventGroupClearBits(dacStreamControl, EG_STREAMCONTROL_BUFFERA_EMPTY);
            }
            if(xEventGroupGetBits(dacStreamControl) & EG_STREAMCONTROL_BUFFERB_EMPTY){
                for(int i = 0; i < CONFIG_DAC_STREAMING_BUFFERSIZE; i++) {
                    dacBufferB.dacA[i] = data.dacA[i];
                    dacBufferB.dacB[i] = data.dacB[i];
                }
                xEventGroupClearBits(dacStreamControl, EG_STREAMCONTROL_BUFFERB_EMPTY);
            }
            if(streamCallbackFunction != NULL) {
                (*streamCallbackFunction)();
            }
        }
    }
}
#endif

void dac_set_stream_callback(void* stream_callback_function) {
#ifdef CONFIG_DAC_STREAMING
    streamCallbackFunction = stream_callback_function;
#endif
}

void eduboard_init_dac() {
    ESP_LOGI(TAG, "Init DAC...");
    dacLock = xSemaphoreCreateMutex();
    for(int i = 0; i < 2; i++) {
        dacData[i].enabled = false;
        dacData[i].gain = false;
        dacData[i].value = 0;
    }
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL<<GPIO_SD_LDAC_CS);
    gpio_config(&io_conf);
    gpio_set_level(GPIO_SD_LDAC_CS, 1);
#ifdef CONFIG_DAC_STREAMING
    dacstreamqueue = xQueueCreate(2, sizeof(dac_stream_data_t));
    dacBufferSwitchSignal = xSemaphoreCreateBinary();
    dacStreamControl = xEventGroupCreate();
    gpspi_init_nonblocking(&dev_dac_spi, GPIO_MOSI, GPIO_MISO, GPIO_SCK, GPIO_FLASH_DAC_CS, DAC_FREQ_MHZ, true);
    xTaskCreate(dac_bufferManager, "dac_bufferManagerTask", 2048, NULL, 10, NULL);
#else
    gpspi_init(&dev_dac_spi, GPIO_MOSI, GPIO_MISO, GPIO_SCK, GPIO_FLASH_DAC_CS, DAC_FREQ_MHZ, false);
#endif
}