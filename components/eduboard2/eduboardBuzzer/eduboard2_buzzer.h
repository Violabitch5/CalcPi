#pragma once
#include <stdlib.h>

#ifdef CONFIG_ENABLE_BUZZER
void buzzer_start(uint16_t freq_hz, uint32_t length_ms);
void buzzer_stop();
void buzzer_set_volume(uint8_t volume);
void eduboard_init_buzzer();
#endif