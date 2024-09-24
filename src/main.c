/********************************************************************************************* */
//    Eduboard2 ESP32-S3 PI Calculation Firmware
//    Author: Nathanael Gubler
//    Juventus Technikerschule
//    Version: 1.0.0
//    
//    This program compares two calculation methods for PI approximations which can be run seperately.
//    Hardware is included under components/eduboard2.
//    Hardware support can be activated/deactivated in components/eduboard2/eduboard2_config.h
/********************************************************************************************* */
#include "eduboard2.h"
#include "memon.h"

#include "math.h"
#include "string.h"

#define TAG "CALCULATIONofPI"

#define UPDATETIME_MS 100

#define NUM_BTNS 4

#define STATE_MASK 0xFF
#define ACTION_MASK 0xFF00

#define DEBUG_LOGS (false)
#define HIGHWATERMARK_LOGS (false)
#define BTN_LOGS (true)
#define DISPLAY_DEBUG (true)
#define CALC_DEBUG (true)

typedef enum {
    S1_SHORT = 1 << SW0,
    S2_SHORT = 1 << SW1,
    S3_SHORT = 1 << SW2,
    S4_SHORT = 1 << SW3,
    S1_LONG = 1 << (SW0 + NUM_BTNS),
    S2_LONG = 1 << (SW1 + NUM_BTNS),
    S3_LONG = 1 << (SW2 + NUM_BTNS),
    S4_LONG = 1 << (SW3 + NUM_BTNS),
};

struct timestamp{
    double_t curr_v;
    u_int32_t ticks;
};

static TaskHandle_t
    DisplayTask_hndl = NULL,
    ButtonTask_hndl = NULL,
    LogicTask_hndl = NULL,
    CalcTaskA_hndl = NULL,
    CalcTaskB_hndl = NULL;

double_t g_running_val_A;
double_t g_running_val_B;

u_int64_t g_running_ticks_A;
u_int64_t g_running_ticks_B;

char g_running_pistring_A[60];
char g_running_pistring_B[60];

typedef enum {
    A,
    B,
    NUM_OF_METHODS
} Calculation_Method;

struct timestamp GetCurrTimestamp(TaskHandle_t CalcTask_hndl) {
    struct timestamp current_time = {0, 0, NULL};

    vTaskSuspend(CalcTask_hndl);

    switch (( int ) xTaskGetApplicationTaskTag(CalcTask_hndl)){
    case A:
        current_time.curr_v = g_running_val_A;
        current_time.ticks = g_running_ticks_A * portTICK_PERIOD_MS;
        break;
    case B:
        current_time.curr_v = g_running_val_B;
        current_time.ticks = g_running_ticks_B * portTICK_PERIOD_MS;
        break;
    }
    
    vTaskResume(CalcTask_hndl);

    return current_time;
}

void CalcTaskA(){
    // Calculation via Madhava–Leibniz
    u_int32_t iters = 0, divisor = 3, dividend = 4;
    int64_t sign = -1;

    g_running_val_A = 4.0;

    for(;;){
        g_running_val_A += sign * (dividend/divisor);
        sign *= -1;
        divisor += 2;
        g_running_ticks_A = xTaskGetTickCount();
        vTaskDelay(0);
    }
}

void templateTask(void* param) {
    //Init stuff here

    for(;;) {
        // task main loop
        if(button_get_state(SW0, true) == SHORT_PRESSED) {
            led_toggle(LED0);
        }
        led_toggle(LED7);
        // delay
        vTaskDelay(UPDATETIME_MS/portTICK_PERIOD_MS);
    }
}

void app_main()
{
    //Initialize Eduboard2 BSP
    eduboard2_init();
    
    //Create templateTask
    xTaskCreate(templateTask,   //Subroutine
                "testTask",     //Name
                2*2048,         //Stacksize
                NULL,           //Parameters
                10,             //Priority
                NULL);          //Taskhandle
    for(;;) {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Hello Eduboard");
        }
}
