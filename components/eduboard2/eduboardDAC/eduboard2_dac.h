#pragma once

typedef enum {
    DAC_GAIN_2 = 0,
    DAC_GAIN_1 = 1,
} dac_gain_t;
typedef enum {
    DAC_A = 0,
    DAC_B = 1,
} dac_num_t;

void dac_set_config(dac_num_t dacNum, dac_gain_t gain, bool enabled);
void dac_set_value(dac_num_t dacNum, uint8_t value);
void dac_update();
void dac_load_stream_data(uint8_t* dataA, uint8_t* dataB);
void dac_set_stream_callback(void* stream_callback_function);
void eduboard_init_dac();