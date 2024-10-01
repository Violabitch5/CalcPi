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
#define CLEAR_ALL 0xFFFFFF

#define DEBUG_LOGS (false)
#define HIGHWATERMARK_LOGS (false)
#define BTN_LOGS (true)
#define DISPLAY_DEBUG (false)
#define CALC_DEBUG (true)

typedef enum {
    SW0_SHORT = 1 << SW0,
    SW1_SHORT = 1 << SW1,
    SW2_SHORT = 1 << SW2,
    SW3_SHORT = 1 << SW3,
    SW0_LONG = 1 << (SW0 + NUM_BTNS),
    SW1_LONG = 1 << (SW1 + NUM_BTNS),
    SW2_LONG = 1 << (SW2 + NUM_BTNS),
    SW3_LONG = 1 << (SW3 + NUM_BTNS),
    ALL_BTN_EVENTS = 255
}btn_events;

typedef enum {
    STOPPED     = 1 << 0,
    STARTING    = 1 << 1,
    RUNNING     = 1 << 2,
    RESETTING   = 1 << 3,
    ANY_STATE   = STOPPED | STARTING | RUNNING | RESETTING
} calc_state;

struct pi_bounds {
    double_t upper;
    double_t lower;
};

static struct pi_bounds PI_1DIGIT =     {3.1999999999999999,3.1};
static struct pi_bounds PI_2DIGIT =     {3.1499999999999999,3.14};
static struct pi_bounds PI_3DIGIT =     {3.1419999999999999,3.141};
static struct pi_bounds PI_4DIGIT =     {3.1415999999999999,3.1415};
static struct pi_bounds PI_5DIGIT =     {3.1415999999999999,3.14159};
static struct pi_bounds PI_6DIGIT =     {3.1415929999999999,3.141592};
static struct pi_bounds PI_7DIGIT =     {3.1415926999999999,3.1415926};
static struct pi_bounds PI_8DIGIT =     {3.1415926599999999,3.14159265};
static struct pi_bounds PI_9DIGIT =     {3.1415926539999999,3.141592653};
static struct pi_bounds PI_10DIGIT =    {3.1415926535999999,3.1415926535};
static struct pi_bounds PI_11DIGIT =    {3.1415926535899999,3.14159265358};
static struct pi_bounds PI_12DIGIT =    {3.1415926535899999,3.141592653589};
static struct pi_bounds PI_13DIGIT =    {3.1415926535897999,3.1415926535897};
static struct pi_bounds PI_14DIGIT =    {3.1415926535897999,3.14159265358979};
static struct pi_bounds PI_15DIGIT =    {3.1415926535897939,3.141592653589793};

struct timestamp{
    double_t curr_val;
    u_int32_t start_tick_count;
    u_int32_t end_tick_count;
    u_int32_t ms;
    u_int32_t iters;
    bool reached_prec;
} g_running_ts_A, g_running_ts_B, g_calc_result_A, g_calc_result_B;

static TaskHandle_t
    DisplayTask_hndl = NULL,
    ButtonTask_hndl = NULL,
    LogicTask_hndl = NULL,
    CalcTaskA_hndl = NULL,
    CalcTaskB_hndl = NULL;

static EventGroupHandle_t
    Calc_Eventgroup_A_hndl = NULL,
    Calc_Eventgroup_B_hndl = NULL,
    Btn_Eventgroup_hndl = NULL,
    MethodInfo_Eventgroup_hndl = NULL;

typedef enum {
    A = 1 << 0,
    B = 1 << 1,
} Calculation_Method;

struct timestamp GetCurrTimestamp(TaskHandle_t CalcTask_hndl) {
    // stops the indicated task and fetches the data
    struct timestamp current_timestamp = {0, 0, 0, 0, 0, false};
    eTaskState curr_state = 0;

    curr_state = eTaskGetState(CalcTask_hndl);

    if ((curr_state == eRunning) || (curr_state == eReady)) { 
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Suspending Task to fetch data."); }
        vTaskSuspend(CalcTask_hndl); 
        }

    switch (( int ) xTaskGetApplicationTaskTag(CalcTask_hndl)){
    case A:
        current_timestamp.curr_val = g_running_ts_A.curr_val;
        current_timestamp.ms = (g_running_ts_A.end_tick_count - g_running_ts_A.start_tick_count) * portTICK_PERIOD_MS;
        current_timestamp.iters = g_running_ts_A.iters;
        current_timestamp.reached_prec = g_running_ts_A.reached_prec;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Taking Data from A."); }
        break;
    case B:
        current_timestamp.curr_val = g_running_ts_B.curr_val;
        current_timestamp.ms = (g_running_ts_B.end_tick_count - g_running_ts_B.start_tick_count) * portTICK_PERIOD_MS;
        current_timestamp.iters = g_running_ts_B.iters;
        current_timestamp.reached_prec = g_running_ts_B.reached_prec;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Taking Data from B."); }
        break;
    }
    
    if ((curr_state == eRunning) || (curr_state == eReady)) { 
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Resuming Task after fetching data."); }
        vTaskResume(CalcTask_hndl); 
        }

    return current_timestamp;
}

void copy_data_into_result() {

    switch (( int ) xTaskGetApplicationTaskTag(NULL)){

    vTaskSuspend(DisplayTask_hndl);
    case A:
        g_calc_result_A = g_running_ts_A;
        g_calc_result_A.ms = (g_calc_result_A.end_tick_count - g_calc_result_A.start_tick_count) * portTICK_PERIOD_MS;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Copied data into result A."); }
        break;
    case B:
        g_calc_result_B = g_running_ts_B;
        g_calc_result_B.ms = (g_calc_result_B.end_tick_count - g_calc_result_B.start_tick_count) * portTICK_PERIOD_MS;
        
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Copied data into result B."); }
        break;
    }
    vTaskResume(DisplayTask_hndl);
}

int check_for_precision(double_t value, struct pi_bounds bounds){
    if (CALC_DEBUG) {ESP_LOGI(TAG,"Current lower bound: %.15lf, upper: %.15lf", bounds.lower, bounds.upper);}
    if ((value < bounds.upper) && (value > bounds.lower)){
        return 1;
    }else{
        return 0;
    }
}

void CalcTaskA(struct pi_bounds * boundaries){
    // iterative calculation via Madhava–Leibniz
    // Notifies Logic Task when it has reached the requested precision
    double_t divisor = 3, dividend = 4, sign = -1, running_sum = 0;
    
    Calculation_Method method = A;
    EventBits_t state = STOPPED;

    xEventGroupSetBits(Calc_Eventgroup_A_hndl, state);

    g_running_ts_A.curr_val = 4.0;
    g_running_ts_A.iters = 1;
    g_running_ts_A.start_tick_count = 0;
    g_running_ts_A.end_tick_count = 0;
    g_running_ts_A.reached_prec = false;

    copy_data_into_result();

    vTaskSetApplicationTaskTag(NULL, (void *) method);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task A initialized.");}

    for(;;){
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Calculation Task A Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        state = xEventGroupGetBits(Calc_Eventgroup_A_hndl);

        if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task A state: %li", state);}

        switch (state)
        {
        case STOPPED:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is stopped.");}
            vTaskSuspend(NULL);
            continue;
        
        case RESETTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is resetting.");}
            g_running_ts_A.start_tick_count = 0;
            g_running_ts_A.end_tick_count = 0;
            g_running_ts_A.curr_val = 4.0;
            g_running_ts_A.iters = 1;
            g_running_ts_A.reached_prec = false;
            divisor = 3;
            sign = -1;
            copy_data_into_result();
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPED);
            vTaskDelay(UPDATETIME_MS/portTICK_PERIOD_MS);
            ulTaskNotifyValueClear(DisplayTask_hndl,CLEAR_ALL);
            continue;
        case STARTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is starting.");}
            g_running_ts_A.start_tick_count = xTaskGetTickCount();
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, RUNNING);
            break;
        case RUNNING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is running. Current value: %.19lf", g_running_ts_A.curr_val);}
            //if (CALC_DEBUG) {ESP_LOGI(TAG, "running sum: %1.15lf", running_sum);}

            running_sum = sign * (dividend/divisor);
            g_running_ts_A.curr_val += running_sum;
            sign *= -1;
            divisor += 2;

            g_running_ts_A.end_tick_count = xTaskGetTickCount();
            g_running_ts_A.iters++;

            if (!g_running_ts_A.reached_prec){
                if (check_for_precision(g_running_ts_A.curr_val, *boundaries)) {
                    g_running_ts_A.reached_prec = true;
                    copy_data_into_result();
                    xTaskNotifyGive(DisplayTask_hndl);
                    }
            }

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
    }
}

/// Helper Function for Chudnovsky calculation method
double_t P (double_t j) {
    double_t prod;

    prod = -(6.0*j - 5.0) * (2.0*j - 1.0) * (6.0*j - 1.0);

    return prod;
}

/// Helper Function for Chudnovsky calculation method
double_t Q (double_t j) {
    double_t prod;

    prod = 10939058860032000 * pow(j,3.0);
    
    return prod;
}

// Recursive calculation led to quick stack overflow
    // struct PQR bin_split(double_t a, double_t b){
    //     // Helper Function for Chudnovsky calculation method
    //     struct PQR pqr_vals = {0.0,0.0,0.0}, 
    //     pqr_am_vals = {0.0,0.0,0.0},
    //     pqr_mb_vals = {0.0,0.0,0.0};

    //     double_t m = 0.0;

    //     if (b == (a + 1)){
    //         pqr_vals.Pab = -(6*a - 5) * (2*a - 1) * (6*a - 1);
    //         pqr_vals.Qab = 10939058860032000 * pow(a,3.0);
    //         pqr_vals.Rab = pqr_vals.Pab * (545140134*a + 13591409);
    //     } else {
    //         m = (a + b) / 2.0;
    //         pqr_am_vals = bin_split(a, m);
    //         pqr_mb_vals = bin_split(m, b);

    //         pqr_vals.Pab = pqr_am_vals.Pab * pqr_mb_vals.Pab;
    //         pqr_vals.Qab = pqr_am_vals.Qab * pqr_mb_vals.Qab;
    //         pqr_vals.Rab = pqr_mb_vals.Qab * pqr_am_vals.Rab + pqr_mb_vals.Rab * pqr_am_vals.Pab;
    //     }
    //     if (CALC_DEBUG) {ESP_LOGI(TAG, "bin_split values: a:%.4lf    b:%.4lf  m:%.4lf", a, b, m);}
    //     return pqr_vals;
    // }


void CalcTaskB(struct pi_bounds * boundaries){
    //iterative calculation via Chudnovsky method
    //Notifies Logic Task when it has reached the requested precision
    g_running_ts_B.curr_val = 0.0;
    g_running_ts_B.iters = 1;
    g_running_ts_B.start_tick_count = 0;
    g_running_ts_B.end_tick_count = 0;
    g_running_ts_B.reached_prec = false;

    copy_data_into_result();
    
    double_t running_prod = 1.0, running_sum = 0.0;
    double_t dividend = 426880 * sqrt(10005);

    EventBits_t state = STOPPED;
    xEventGroupSetBits(Calc_Eventgroup_B_hndl, state);

    Calculation_Method method = B;

    vTaskSetApplicationTaskTag(NULL, (void *) method);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task B initialized.");}

    for(;;){
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Calculation Task B Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        state = xEventGroupGetBits(Calc_Eventgroup_B_hndl);

        if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task B state: %li", state);}

        switch (state)
        {
        case STOPPED:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is stopped.");}
            vTaskSuspend(NULL);
            continue;
        
        case RESETTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is resetting.");}
            g_running_ts_B.start_tick_count = 0;
            g_running_ts_B.end_tick_count = 0;
            g_running_ts_B.curr_val = 0.0;
            g_running_ts_B.iters = 1;
            running_prod = 1.0;
            running_sum = 0.0;
            copy_data_into_result();
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPED);
            vTaskDelay(UPDATETIME_MS/portTICK_PERIOD_MS);
            ulTaskNotifyValueClear(DisplayTask_hndl,CLEAR_ALL);
            continue;

        case STARTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is starting.");}
            g_running_ts_A.start_tick_count = xTaskGetTickCount();
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, RUNNING);
            break;

        case RUNNING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is running. Current value:%.19lf", g_running_ts_B.curr_val);}
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Running SUM: %.50lf    Running PROD: %.50lf", running_sum, running_prod);}

            running_prod *= (double_t) P(g_running_ts_B.iters) / Q(g_running_ts_B.iters);
            running_sum += (double_t) running_prod * (545140134 * g_running_ts_B.iters + 13591409);

            g_running_ts_B.curr_val = (double_t) dividend / (13591409 + running_sum);
            
            g_running_ts_B.end_tick_count = xTaskGetTickCount();
            g_running_ts_B.iters++;

            if (!g_running_ts_B.reached_prec){
                if (check_for_precision(g_running_ts_B.curr_val, *boundaries)) {
                    g_running_ts_B.reached_prec = true;
                    copy_data_into_result();
                    xTaskNotifyGive(DisplayTask_hndl);
                    }
            }

            if (running_prod < 0.00000000000000000000000000001) {
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Stopping Calc Task B due to running product reaching limit.");}
                xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPED);
            }
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }   
    } 
}

void BtnTask(void* param){
    //Checks if any buttons has been pressed and give notification to LogicTask if so.

    uint16_t btn_states;

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Button Task gestartet");}

    for(;;){
        //Delay (more elegant to do it here)
        vTaskDelay(50/portTICK_PERIOD_MS);

        //Check if any of the buttons is long or short pressed. (different bits are set for long or short presses)
        for (int i = 0; i < NUM_BTNS; i++){
            if (button_get_state(i, false) == NOT_PRESSED) {continue;}
            if (button_get_state(i, true) == SHORT_PRESSED) { btn_states += 1 << i;}
            else { btn_states += 1 << (i + NUM_BTNS);}   
        }

        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Button Task Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        //loop if none is pressed
        if (btn_states == 0) { continue; }

        if (BTN_LOGS) {ESP_LOGI(TAG, "Button was pressed: %i", btn_states);}

        //Notify Logic task with changed states
        xEventGroupClearBits(Btn_Eventgroup_hndl, CLEAR_ALL);
        xEventGroupSetBits(Btn_Eventgroup_hndl, btn_states);

        //reset btn states
        btn_states = 0;
    }
}

void start_calc_method(Calculation_Method method){
    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, STARTING);
        vTaskResume(CalcTaskA_hndl);
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, STARTING);
        vTaskResume(CalcTaskB_hndl);
    default:
        break;
    }
}

void stop_calc_method(Calculation_Method method){
    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPED);
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPED);
    default:
        break;
    }
}

void reset_calc_method(Calculation_Method method){
    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, RESETTING);
        if (eTaskGetState(CalcTaskA_hndl) == eSuspended) { vTaskResume(CalcTaskA_hndl); }
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, RESETTING);
        if (eTaskGetState(CalcTaskB_hndl) == eSuspended) { vTaskResume(CalcTaskB_hndl); }
    default:
        break;
    }
}

void LogicTask(void* param){
    //Waits for and handles all btn state changes
    EventBits_t btns = 0, curr_method = 0;

    xEventGroupSetBits(MethodInfo_Eventgroup_hndl,A);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Logic Task initialized.");}

    for(;;){
        curr_method = xEventGroupGetBits(MethodInfo_Eventgroup_hndl);
        btns = xEventGroupWaitBits(Btn_Eventgroup_hndl, ALL_BTN_EVENTS, true, false, portMAX_DELAY);
        switch (btns)
        {
        //Switch calculation method
        case SW3_SHORT:
            if (curr_method == A) {
                xEventGroupClearBits(MethodInfo_Eventgroup_hndl, CLEAR_ALL);
                xEventGroupSetBits(MethodInfo_Eventgroup_hndl, B);
                if (DEBUG_LOGS) {ESP_LOGI(TAG,"Current calculation method: B");}
            } else {
                xEventGroupClearBits(MethodInfo_Eventgroup_hndl, CLEAR_ALL);
                xEventGroupSetBits(MethodInfo_Eventgroup_hndl, A);
                if (DEBUG_LOGS) {ESP_LOGI(TAG,"Current calculation method: A");}
            }
            break;
        //Starts calculation method
        case SW0_SHORT:
            start_calc_method(curr_method);
            break;
        //Halts calculation method
        case SW1_SHORT:
            stop_calc_method(curr_method);
            break;
        //Resets calculation method
        case SW2_SHORT:
            reset_calc_method(curr_method);
            break;
        default:
            if (DEBUG_LOGS) {ESP_LOGI(TAG,"Undefined button state received: %li",(uint32_t)btns);}
            break;
        }
    }
}

void DisplayTask(void* param) {
    //Draws Diisplay content
    EventBits_t calcA_state = STOPPED, calcB_state = STOPPED;
    struct timestamp curr_pi_calcA_data = {0,0,0,0,0, false}, curr_pi_calcB_data = {0,0,0,0,0, false};

    char methodA_status_string[60],
    methodB_status_string[60],
    methodA_resetted_string[60],
    methodB_resetted_string[60],  
    curr_valueA_string[60], 
    curr_valueB_string[60], 
    curr_timeA_string[60], 
    curr_timeB_string[60],
    precA_reached_string[60],
    precB_reached_string[60];

    uint32_t prec_reached = 0;

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Display Task initialized.");}

    for(;;) {
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Display Task Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        lcdFillScreen(BLACK);
        lcdDrawString(fx32M, 10, 30, "ESP32 Pi Calcualtion", GREEN);
        lcdDrawString(fx24M, 10, 80, "by Nathanael", GREEN);

        ulTaskNotifyTake(false, 500/portTICK_PERIOD_MS);
        //vTaskDelay(500/portTICK_PERIOD_MS);

        calcA_state = xEventGroupGetBits(Calc_Eventgroup_A_hndl);
        calcB_state = xEventGroupGetBits(Calc_Eventgroup_B_hndl);
        curr_pi_calcA_data = GetCurrTimestamp(CalcTaskA_hndl);
        curr_pi_calcB_data = GetCurrTimestamp(CalcTaskB_hndl);

        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Current Value A for Pi: %lf",curr_pi_calcA_data.curr_val);}
        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Current Value B for Pi: %lf",curr_pi_calcB_data.curr_val);}

        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"CalcA_bits: %li",calcA_state);}
        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"CalcB_bits: %li",calcB_state);}

        switch (calcA_state)
        {
        case STOPPED:
            sprintf((char *)methodA_status_string, "Methode A deaktiviert");
            lcdDrawString(fx16M, 30, 110, &methodA_status_string[0], GRAY);
            break;
        case RUNNING:
            sprintf((char *)methodA_status_string, "Methode A aktiv");
            sprintf((char *)methodA_resetted_string, " ");
            lcdDrawString(fx16M, 30, 110, &methodA_status_string[0], CYAN);
            if (g_calc_result_A.reached_prec) {
                sprintf((char *)precA_reached_string, "Die Genauigkeit wurde nach %li ms erreicht!", g_calc_result_A.ms);
                lcdDrawString(fx16M, 30, 130, &precA_reached_string[0], GREEN);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method A reached precision!");}
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Value: %.15lf, Time: %8li ms, iterations: %12li", g_calc_result_A.curr_val, g_calc_result_A.ms, g_calc_result_A.iters);}
            } else {
                sprintf((char *)precA_reached_string, "Der Wert ist noch zu ungenau.");
                lcdDrawString(fx16M, 30, 130, &precA_reached_string[0], RED);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method A has not yet reached precision...");}
            };
            break;
        case RESETTING:
            sprintf((char *)methodA_resetted_string, "Methode A zurueckgesetzt");
            break;
        }
        lcdDrawString(fx16M, 30, 90, &methodA_resetted_string[0], CYAN);

        switch (calcB_state)
        {
        case STOPPED:
            sprintf((char *)methodB_status_string, "Methode B deaktiviert");
            lcdDrawString(fx16M, 80, 110, &methodB_status_string[0], RED);
            break;
        case RUNNING:
            sprintf((char *)methodB_status_string, "Methode B aktiv");
            sprintf((char *)methodB_resetted_string, " ");
            lcdDrawString(fx16M, 80, 110, &methodB_status_string[0], GREEN);
            if (g_calc_result_B.reached_prec) {
                sprintf((char *)precB_reached_string, "Die Genauigkeit wurde nach %li ms erreicht!", g_calc_result_B.ms);
                lcdDrawString(fx16M, 80, 130, &precB_reached_string[0], GREEN);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method B reached precision!");}
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Value: %.15lf, Time: %8li ms, iterations: %12li", g_calc_result_B.curr_val, g_calc_result_B.ms, g_calc_result_B.iters);}
            } else {
                sprintf((char *)precB_reached_string, "Der Wert ist noch zu ungenau.");
                lcdDrawString(fx16M, 80, 130, &precB_reached_string[0], RED);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method B has not yet reached precision...");}
            };
            break;
        case RESETTING:
            sprintf((char *)methodB_resetted_string, "Methode B zurueckgesetzt");
            break;
        }
        lcdDrawString(fx16M, 30, 90, &methodB_resetted_string[0], CYAN);

        sprintf((char *)curr_valueA_string, "Aktueller A Wert:/t%lf", curr_pi_calcA_data.curr_val);
        sprintf((char *)curr_timeA_string, "Aktuelle Berechnungszeit A:/t%li ms", curr_pi_calcA_data.ms);
        sprintf((char *)curr_valueB_string, "Aktueller B Wert:/t%lf", curr_pi_calcB_data.curr_val);
        sprintf((char *)curr_timeB_string, "Aktuelle Berechnungszeit B:/t%li ms", curr_pi_calcB_data.ms);

        lcdDrawString(fx16M, 80, 160, &curr_valueB_string[0], WHITE);
        lcdDrawString(fx16M, 80, 190, &curr_timeB_string[0], WHITE);

        lcdDrawString(fx16M, 30, 160, &curr_valueA_string[0], WHITE);
        lcdDrawString(fx16M, 30, 190, &curr_timeA_string[0], WHITE);
    }
}

void app_main()
{
    struct pi_bounds prec = PI_4DIGIT;

    //Initialize Eduboard2 BSP
    eduboard2_init();
    
    //create EventGroups
    Calc_Eventgroup_A_hndl = xEventGroupCreate();
    Calc_Eventgroup_B_hndl = xEventGroupCreate();
    Btn_Eventgroup_hndl = xEventGroupCreate();
    MethodInfo_Eventgroup_hndl = xEventGroupCreate();

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Event Groups initialized.");}

    //Create Tasks
    xTaskCreate(BtnTask,"Button Task", 2*2048,NULL,10,&ButtonTask_hndl);
    xTaskCreate(LogicTask,"Logic Task",2*2048,NULL,5,&LogicTask_hndl);
    xTaskCreate(DisplayTask,"Display Taks", 2*2048,NULL,4,&DisplayTask_hndl);
    xTaskCreate(CalcTaskA,"Calculation Task A",2*2048,&prec,2,&CalcTaskA_hndl);
    xTaskCreate(CalcTaskB,"Calculation Task B",8*2048,&prec,2,&CalcTaskB_hndl);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Tasks initialized");}

    for(;;) {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        }
}
