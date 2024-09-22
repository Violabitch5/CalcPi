#pragma once

#include <stdlib.h>
#include <stdbool.h>

#ifdef CONFIG_ENABLE_SW0
    #define SW0 0
#endif
#ifdef CONFIG_ENABLE_SW1
    #define SW1 1
#endif
#ifdef CONFIG_ENABLE_SW2
    #define SW2 2
#endif
#ifdef CONFIG_ENABLE_SW3
    #define SW3 3
#endif

#define BUTTONPRESS_LONG_MS     500
#define BUTTONPRESS_SHORT_MS    50

typedef enum {
    NOT_PRESSED, 
    SHORT_PRESSED, 
    LONG_PRESSED
} button_state;

button_state button_get_state(uint8_t button_num, bool reset);
void eduboard_init_buttons();