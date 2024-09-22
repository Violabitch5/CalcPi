#pragma once
#include "../eduboard2_config.h"
#include <stdlib.h>

#ifdef CONFIG_LCD_RESOLUTION_240x240
    #define CONFIG_WIDTH    240
    #define CONFIG_HEIGHT   240
    #define CONFIG_OFFSET_WIDTH 0
    #define CONFIG_OFFSET_HEIGHT 0
#endif
#ifdef CONFIG_LCD_RESOLUTION_240x320
    #define CONFIG_WIDTH    240
    #define CONFIG_HEIGHT   320
    #define CONFIG_OFFSET_WIDTH 0
    #define CONFIG_OFFSET_HEIGHT 0
#endif
#ifdef CONFIG_LCD_RESOLUTION_320x480
    #define CONFIG_WIDTH    320
    #define CONFIG_HEIGHT   480
    #define CONFIG_OFFSET_WIDTH 0
    #define CONFIG_OFFSET_HEIGHT 0
#endif


void eduboard_init_lcd();