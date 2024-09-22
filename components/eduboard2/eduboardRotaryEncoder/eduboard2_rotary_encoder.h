#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "../eduboardButton/eduboard2_button.h"

// #define ROTARYENCODER_USE_INTERRUPTS
// #define ROTARYENCODER_USE_EDGEA
#define ROTARYENCODER_USE_EDGEB

button_state rotary_encoder_button_get_state(bool reset);
int32_t rotary_encoder_get_rotation(bool reset);
void eduboard_init_rotary_encoder();