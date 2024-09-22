#pragma once

#include <stdio.h>

typedef struct 
{
    uint16_t x;
    uint16_t y;
}touchpos_t;


typedef struct {
    uint8_t touches;
    touchpos_t points[2];
} touchevent_t;

void eduboard_init_ft6236(void);

touchevent_t ft6236_get_touch_event(bool reset);
bool ft6236_is_touched();