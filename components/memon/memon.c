#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
//#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_freertos_hooks.h"

#include "lwip/err.h"
#include "esp_attr.h"

#include "esp_log.h"
#include "esp_err.h"

#include "memon.h"

#define TAG "MEMON"
#define MEMON_VERSION   "1.0.0"
#define MEMON_BUFFERSIZE    2048

EventGroupHandle_t evMemon;
#define EV_MEMON_ENABLED    1<<0

uint8_t memonUpdateTime_s = MEMON_BASE_UPDATERATE_S;

void memonTask(void* param) {
    ESP_LOGI(TAG, "MEMON startup...");
    ESP_LOGI(TAG, "MEMON Version: %s", MEMON_VERSION);
    char* memonoutput = malloc(MEMON_BUFFERSIZE);
    
    TaskStatus_t* systemtasklist;
    uint32_t systemtasklistsize = 0;
    uint32_t totalRuntime, StatsAsPercentage;

    for(;;) {        
        xEventGroupWaitBits(evMemon, EV_MEMON_ENABLED, false, true, portMAX_DELAY);
        uint32_t systemtasklistsize = uxTaskGetNumberOfTasks();
        systemtasklist = malloc(systemtasklistsize * sizeof(TaskStatus_t));
        systemtasklistsize = uxTaskGetSystemState(systemtasklist, systemtasklistsize, &totalRuntime);
        memset(memonoutput, 0, MEMON_BUFFERSIZE);
        sprintf(&memonoutput[0], "\n-----------------------------------------\nMEMON-Report:");
        sprintf(&memonoutput[strlen(memonoutput)], "\n\nActive Tasks: %i", (int)systemtasklistsize);
        sprintf(&memonoutput[strlen(memonoutput)], "\n----Name---------------- TaskNr -- Prio -- CoreID -- Stack[bytes]---Runtime[cyc/s]");
        TaskStatus_t* systemtaskstate;
        for(int i = 0; i < systemtasklistsize;i++) {
            systemtaskstate = &systemtasklist[i];
            char taskname[20];
            uint32_t tasknumber = systemtaskstate->xTaskNumber;
            int32_t taskprio = systemtaskstate->uxBasePriority;            
            uint32_t runtime = systemtaskstate->ulRunTimeCounter/memonUpdateTime_s;
            uint32_t taskstack = systemtaskstate->usStackHighWaterMark;
            uint32_t taskcoreid = (uint32_t)systemtaskstate->xCoreID;
            //If Error here: Enable Components->FreeRTOS->"Enable FreeRTOS trace facility"->"Enable FreeRTOS stats formatting functions"->"Enable display of xCoreID in vTaskList"
            taskcoreid = (taskcoreid > 1 ? -1 :taskcoreid);
            strcpy(&taskname[0], systemtaskstate->pcTaskName);
            sprintf(&memonoutput[strlen(memonoutput)], "\n    %-20s %-6i    %-6i  %-6d    %-6i         %-9i", &taskname[0], (int)tasknumber, (int)taskprio, (int)taskcoreid, (int)taskstack, (int)runtime);
        }sprintf(&memonoutput[strlen(memonoutput)], "\n\nGlobal Heap: %i bytes\n-----------------------------------------\n",(int)xPortGetFreeHeapSize());

        ESP_LOGW(TAG, "%s", memonoutput);
        free(systemtasklist);
        vTaskDelay(memonUpdateTime_s*1000/portTICK_PERIOD_MS);
    }
}

void memon_enable() {
    xEventGroupSetBits(evMemon, EV_MEMON_ENABLED);
}
void memon_disable() {
    xEventGroupClearBits(evMemon, EV_MEMON_ENABLED);
}

void memon_setUpdateTime(uint8_t updateTime_s) {
    memonUpdateTime_s = updateTime_s;
}

TaskHandle_t hMemonTask;
void initMemon(void) {
    evMemon = xEventGroupCreate();
    xTaskCreate(&memonTask, "memonTask", 3000, NULL, 20, &hMemonTask);
}

