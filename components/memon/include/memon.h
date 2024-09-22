#ifndef MEMON_H
#define MEMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MEMON_BASE_UPDATERATE_S      3

void memon_enable();
void memon_disable();
void memon_setUpdateTime(uint8_t updateTime_s);
void initMemon(void);

#endif