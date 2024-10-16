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

#define UPDATETIME_MS 100       //general update time
#define CALCITER_TIME_MS 0      //iteration speed for calculation tasks

#define NUM_BTNS 4

#define STATE_MASK 0xFF         
#define ACTION_MASK 0xFF00

#define CLEAR_ALL 0xFFFFFF

#define DEBUG_LOGS (false)
#define HIGHWATERMARK_LOGS (false)
#define BTN_LOGS (false)
#define DISPLAY_DEBUG (false)
#define CALC_DEBUG (false)

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
    STOPPED         = 1 << 0,
    STARTING        = 1 << 1,
    RUNNING         = 1 << 2,
    RESETTING       = 1 << 3,
    STOPPING        = 1 << 4,
    WRITING_RESULT  = 1 << 5,
    ANY_STATE   = STOPPED | STARTING | RUNNING | RESETTING | STOPPING
} calc_state;       //describes different states of calculation task

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
} g_running_ts_A, g_running_ts_B, g_calc_result_A, g_calc_result_B;     //Main struct for holding various calculation data

static TaskHandle_t
    DisplayTask_hndl = NULL,
    ButtonTask_hndl = NULL,
    LogicTask_hndl = NULL,
    CalcTaskA_hndl = NULL,
    CalcTaskB_hndl = NULL;

EventGroupHandle_t
    Calc_Eventgroup_A_hndl = NULL,          // Contains state of Task A, there can only be ONE state at a time
    Calc_Eventgroup_B_hndl = NULL,          // same for B
    Btn_Eventgroup_hndl = NULL,             // used to trigger Logic task to process button inputs
    MethodInfo_Eventgroup_hndl = NULL;      // used to show which Method is currently active


typedef enum {
    A = 1 << 0,
    B = 1 << 1,
} Calculation_Method;

struct timestamp GetCurrTimestamp(TaskHandle_t CalcTask_hndl) {
    // signals the indicated task to stop, waits for that to happen, fetches the data and then sets the task to its previous state
    struct timestamp current_timestamp = {0, 0, 0, 0, 0, false};
    EventBits_t calc_state = STOPPING, wait_state = 0;

    switch (( int ) xTaskGetApplicationTaskTag(CalcTask_hndl)){
    case A:
        calc_state = xEventGroupGetBits(Calc_Eventgroup_A_hndl);
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Task A state before stopping it: %li", calc_state); }

        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPING);
        wait_state = xEventGroupWaitBits(Calc_Eventgroup_A_hndl, STOPPED, pdFALSE, pdFALSE, 500/portTICK_PERIOD_MS);

        if (wait_state != STOPPED) {
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "Failed to get correct calculation data form A due to timeout."); }
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "wait state: %li", wait_state); }
            return current_timestamp;
        }
        
        current_timestamp.curr_val = g_running_ts_A.curr_val;
        current_timestamp.ms = (g_running_ts_A.end_tick_count - g_running_ts_A.start_tick_count) * portTICK_PERIOD_MS;
        current_timestamp.iters = g_running_ts_A.iters;
        current_timestamp.reached_prec = g_running_ts_A.reached_prec;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Taking Data from A."); }

        if (calc_state != STOPPED) {
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "Setting Task A to previous state: %li", calc_state); }
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, calc_state);
        }
        break;

    case B:
        calc_state = xEventGroupGetBits(Calc_Eventgroup_B_hndl);
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Task B state before stopping it: %li", calc_state); }
        
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPING);
        wait_state = xEventGroupWaitBits(Calc_Eventgroup_B_hndl, STOPPED, pdFALSE, pdFALSE, 500/portTICK_PERIOD_MS);

        if (wait_state != STOPPED) {
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "Failed to get correct calculation data form B due to timeout."); }
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "Wait state: %li", wait_state); }
            return current_timestamp;
        }
        
        current_timestamp.curr_val = g_running_ts_B.curr_val;
        current_timestamp.ms = (g_running_ts_B.end_tick_count - g_running_ts_B.start_tick_count) * portTICK_PERIOD_MS;
        current_timestamp.iters = g_running_ts_B.iters;
        current_timestamp.reached_prec = g_running_ts_B.reached_prec;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Taking Data from B."); }

        if (calc_state != STOPPED) {
            if (DEBUG_LOGS) { ESP_LOGI(TAG, "Setting Task B to previous state: %li", calc_state); }
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, calc_state);
        }

        break;

    default:
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Could not copy current calc data due to unknown Task Tag"); }
    }   

    return current_timestamp;
}

void copy_data_into_result() {
    //copies running data into global result struct. This should only be called after the calling task is set to WRITING_RESULT
    switch (( int ) xTaskGetApplicationTaskTag(NULL)){

    case A:
        g_calc_result_A = g_running_ts_A;
        g_calc_result_A.ms = (g_calc_result_A.end_tick_count - g_calc_result_A.start_tick_count) * portTICK_PERIOD_MS;
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Copied data into result A."); }
        break;
    case B:
        g_calc_result_B = g_running_ts_B;
        g_calc_result_B.ms = (g_calc_result_B.end_tick_count - g_calc_result_B.start_tick_count) * portTICK_PERIOD_MS;
        
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Copied data into result B."); }
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "start ticks: %li   end ticks: %li", g_calc_result_B.start_tick_count, g_calc_result_B.end_tick_count ); }
        break;
    default:
        if (DEBUG_LOGS) { ESP_LOGI(TAG, "Could not copy results due to unknown Task Tag"); }
    }
    
    return;
}

int check_for_precision(double_t value, struct pi_bounds bounds){
    //checks a value against the provided precision bounds

    if (CALC_DEBUG) {ESP_LOGI(TAG,"Current lower bound: %.15lf, upper: %.15lf", bounds.lower, bounds.upper);}
    if ((value < bounds.upper) && (value > bounds.lower)){
        return 1;
    }else{
        return 0;
    }
}

void CalcTaskA(struct pi_bounds * boundaries){
    // iterative calculation via Madhava–Leibniz
    // Writes data into result once it has reached requested precision

    double_t divisor = 3, dividend = 4, sign = -1, running_sum = 0;
    
    Calculation_Method method = A;
    EventBits_t init_state = STOPPING, state = STOPPING;

    g_running_ts_A.curr_val = 4.0;
    g_running_ts_A.iters = 1;
    g_running_ts_A.start_tick_count = 0;
    g_running_ts_A.end_tick_count = 0;
    g_running_ts_A.reached_prec = false;

    vTaskSetApplicationTaskTag(NULL, (void *) method);

    xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
    xEventGroupSetBits(Calc_Eventgroup_A_hndl, WRITING_RESULT);
    copy_data_into_result();
    xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
    xEventGroupSetBits(Calc_Eventgroup_A_hndl, init_state);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task A initialized.");}

    for(;;){
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Calculation Task A Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        state = xEventGroupGetBits(Calc_Eventgroup_A_hndl);

        if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task A state: %li", state);}

        switch (state)
        {
        case STOPPING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is stopping.");}
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPED);
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is stopped.");}
            xEventGroupWaitBits(Calc_Eventgroup_A_hndl, RUNNING | STARTING | RESETTING | STOPPING, pdFALSE, pdFALSE, portMAX_DELAY);
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
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, WRITING_RESULT);
            copy_data_into_result();
            xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPING);
            vTaskDelay(UPDATETIME_MS/portTICK_PERIOD_MS);
            continue;

        case STARTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation A is starting.");}
            if (g_running_ts_A.iters == 1) { g_running_ts_A.start_tick_count = xTaskGetTickCount(); }
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

            if ((!g_running_ts_A.reached_prec) && (check_for_precision(g_running_ts_A.curr_val, *boundaries))){
                g_running_ts_A.reached_prec = true;
                xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_A_hndl, WRITING_RESULT);
                copy_data_into_result();
                xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_A_hndl, RUNNING);
            }

            vTaskDelay(CALCITER_TIME_MS/portTICK_PERIOD_MS);
        }
    }
}


double_t P (double_t j) {
    /// Helper Function for Chudnovsky calculation method

    double_t prod;

    prod = -(6.0*j - 5.0) * (2.0*j - 1.0) * (6.0*j - 1.0);

    return prod;
}


double_t Q (double_t j) {
    /// Helper Function for Chudnovsky calculation method

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
    // Writes data into result once it has reached requested precision

    double_t running_prod = 1.0, running_sum = 0.0;
    double_t dividend = 426880 * sqrt(10005);

    EventBits_t init_state = STOPPING, state = STOPPING;
    
    Calculation_Method method = B;

    g_running_ts_B.curr_val = 0.0;
    g_running_ts_B.iters = 1;
    g_running_ts_B.start_tick_count = 0;
    g_running_ts_B.end_tick_count = 0;
    g_running_ts_B.reached_prec = false;

    vTaskSetApplicationTaskTag(NULL, (void *) method);

    xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
    xEventGroupSetBits(Calc_Eventgroup_B_hndl, WRITING_RESULT);
    copy_data_into_result();
    xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
    xEventGroupSetBits(Calc_Eventgroup_B_hndl, init_state);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task B initialized.");}

    for(;;){
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Calculation Task B Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        state = xEventGroupGetBits(Calc_Eventgroup_B_hndl);

        if (DEBUG_LOGS) {ESP_LOGI(TAG, "Calculation Task B state: %li", state);}

        switch (state)
        {
        case STOPPING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is stopping.");}
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPED);
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is stopped.");}
            xEventGroupWaitBits(Calc_Eventgroup_B_hndl, RUNNING | STARTING | RESETTING | STOPPING, pdFALSE, pdFALSE, portMAX_DELAY);
            continue;
        
        case RESETTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is resetting.");}
            g_running_ts_B.start_tick_count = 0;
            g_running_ts_B.end_tick_count = 0;
            g_running_ts_B.curr_val = 0.0;
            g_running_ts_B.iters = 1;
            g_running_ts_B.reached_prec = false;
            running_prod = 1.0;
            running_sum = 0.0;
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, WRITING_RESULT);
            copy_data_into_result();
            xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
            xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPING);
            vTaskDelay(UPDATETIME_MS/portTICK_PERIOD_MS);
            continue;

        case STARTING:
            if (CALC_DEBUG) {ESP_LOGI(TAG, "Calculation B is starting.");}
            if (g_running_ts_B.iters == 1) { g_running_ts_B.start_tick_count = xTaskGetTickCount(); }
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

            if ((!g_running_ts_B.reached_prec) && (check_for_precision(g_running_ts_B.curr_val, *boundaries))){
                g_running_ts_B.reached_prec = true;
                xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_B_hndl, WRITING_RESULT);
                copy_data_into_result();
                xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_B_hndl, RUNNING);
            }

            if (running_prod < 0.00000000000000000000000000001) {
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Stopping Calc Task B due to running product reaching limit.");}
                xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
                xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPING);
            }
            
            vTaskDelay(CALCITER_TIME_MS/portTICK_PERIOD_MS);
            break;
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
    // Helper function to start up a calculation Task
    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, STARTING);
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, STARTING);
    default:
        break;
    }
}

void stop_calc_method(Calculation_Method method){
    // Helper function to stop a calculation Task

    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, STOPPING);
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, STOPPING);
    default:
        break;
    }
}

void reset_calc_method(Calculation_Method method){
    // Helper function to reset a calculation Task

    switch (method)
    {
    case A:
        xEventGroupClearBits(Calc_Eventgroup_A_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_A_hndl, RESETTING);
        break;
    case B:
        xEventGroupClearBits(Calc_Eventgroup_B_hndl, CLEAR_ALL);
        xEventGroupSetBits(Calc_Eventgroup_B_hndl, RESETTING);
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
    //Draws Diisplay content depending on task states
    
    EventBits_t calcA_state = STOPPING, calcB_state = STOPPING, curr_method = A, display_state = RUNNING;
    struct timestamp curr_pi_calcA_data = {0,0,0,0,0, false}, curr_pi_calcB_data = {0,0,0,0,0, false};

    char methodA_status_string[60],
    methodB_status_string[60], 
    curr_valueA_string[60], 
    curr_valueB_string[60], 
    curr_timeA_string[60], 
    curr_timeB_string[60],
    precA_reached_string[60],
    precB_reached_string[60];

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Display Task initialized.");}

    for(;;) {
        if (HIGHWATERMARK_LOGS) {ESP_LOGI(TAG,"Display Task Highwatermark: %i",uxTaskGetStackHighWaterMark(NULL));}

        lcdFillScreen(BLACK);
        lcdDrawString(fx32M, 10, 30, "ESP32 Pi Calcualtion", GREEN);
        lcdDrawString(fx24M, 10, 80, "by Nathanael", GREEN);

        curr_pi_calcA_data = GetCurrTimestamp(CalcTaskA_hndl);
        curr_pi_calcB_data = GetCurrTimestamp(CalcTaskB_hndl);
        calcA_state = xEventGroupGetBits(Calc_Eventgroup_A_hndl);
        calcB_state = xEventGroupGetBits(Calc_Eventgroup_B_hndl);
        curr_method = xEventGroupGetBits(MethodInfo_Eventgroup_hndl);
        
        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Current Value A for Pi: %lf",curr_pi_calcA_data.curr_val);}
        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Current Value B for Pi: %lf",curr_pi_calcB_data.curr_val);}

        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"CalcA_bits: %li",calcA_state);}
        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"CalcB_bits: %li",calcB_state);}

        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Display state: %li",display_state);}

        if (DISPLAY_DEBUG) {ESP_LOGI(TAG,"Display Task running");}
        vTaskDelay(500/portTICK_PERIOD_MS);
        
        if (curr_method == A) {
            lcdDrawString(fx24M, 10, 110, "Methode A (Madhava/Leibniz)", BLUE);
            lcdDrawString(fx24M, 10, 200, "Methode B (Chudnovsky)", GRAY);
        } else {
            lcdDrawString(fx24M, 10, 110, "Methode A (Madhava/Leibniz)", GRAY);
            lcdDrawString(fx24M, 10, 200, "Methode B (Chudnovsky)", BLUE);
        }

        switch (calcA_state)
        {
        case STOPPING:
            sprintf((char *)methodA_status_string, "Methode A inaktiv");
            lcdDrawString(fx16M, 10, 125, &methodA_status_string[0], GRAY);
            break;
        case STOPPED:
            sprintf((char *)methodA_status_string, "Methode A inaktiv");
            lcdDrawString(fx16M, 10, 125, &methodA_status_string[0], GRAY);
            break;
        case RUNNING:
            sprintf((char *)methodA_status_string, "Methode A berechnet...");
            lcdDrawString(fx16M, 10, 125, &methodA_status_string[0], CYAN);
            break;
        case WRITING_RESULT:
            sprintf((char *)methodA_status_string, "update Resultat A");
            lcdDrawString(fx16M, 10, 125, &methodA_status_string[0], CYAN);
        }

        if (curr_pi_calcA_data.iters > 1){
            if ((g_calc_result_A.reached_prec) && (calcA_state != WRITING_RESULT)) {
                sprintf((char *)precA_reached_string, "Die Genauigkeit wurde nach %li ms erreicht!", g_calc_result_A.ms);
                lcdDrawString(fx16M, 10, 140, &precA_reached_string[0], GREEN);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method A reached precision!");}
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Value: %.15lf, Time: %8li ms, iterations: %12li", g_calc_result_A.curr_val, g_calc_result_A.ms, g_calc_result_A.iters);}
            } else {
                sprintf((char *)precA_reached_string, "Der Wert ist noch zu ungenau.");
                lcdDrawString(fx16M, 10, 140, &precA_reached_string[0], RED);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method A has not yet reached precision...");}
            }
        }

        sprintf((char *)curr_valueA_string, "Aktueller Wert:  %.20lf", curr_pi_calcA_data.curr_val);
        sprintf((char *)curr_timeA_string, "Aktuelle Berechnungszeit A: %li ms", curr_pi_calcA_data.ms);

        lcdDrawString(fx16M, 10, 155, &curr_valueA_string[0], WHITE);
        lcdDrawString(fx16M, 10, 170, &curr_timeA_string[0], WHITE);

        switch (calcB_state)
        {
        case STOPPING:
            sprintf((char *)methodB_status_string, "Methode B inaktiv");
            lcdDrawString(fx16M, 10, 215, &methodB_status_string[0], GRAY);
            break;
        case STOPPED:
            sprintf((char *)methodB_status_string, "Methode B inaktiv");
            lcdDrawString(fx16M, 10, 215, &methodB_status_string[0], GRAY);
            break;
        case RUNNING:
            sprintf((char *)methodB_status_string, "Methode B berechnet...");
            lcdDrawString(fx16M, 10, 215, &methodB_status_string[0], GREEN);
            break;
        case WRITING_RESULT:
            sprintf((char *)methodB_status_string, "update Resultat B");
            lcdDrawString(fx16M, 10, 215, &methodB_status_string[0], CYAN);
        }

        if (curr_pi_calcB_data.iters > 1){
            if ((g_calc_result_B.reached_prec) && (calcB_state != WRITING_RESULT)) {
                sprintf((char *)precB_reached_string, "Die Genauigkeit wurde nach %li ms erreicht!", g_calc_result_B.ms);
                lcdDrawString(fx16M, 10, 230, &precB_reached_string[0], GREEN);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method B reached precision!");}
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Value: %.15lf, Time: %8li ms, iterations: %12li", g_calc_result_B.curr_val, g_calc_result_B.ms, g_calc_result_B.iters);}
            } else {
                sprintf((char *)precB_reached_string, "Der Wert ist noch zu ungenau.");
                lcdDrawString(fx16M, 10, 230, &precB_reached_string[0], RED);
                if (CALC_DEBUG) {ESP_LOGI(TAG, "Method B has not yet reached precision...");}
            }
        }

        sprintf((char *)curr_valueB_string, "Aktueller Wert:  %.20lf", curr_pi_calcB_data.curr_val);
        sprintf((char *)curr_timeB_string, "Aktuelle Berechnungszeit B: %li ms", curr_pi_calcB_data.ms);

        lcdDrawString(fx16M, 10, 245, &curr_valueB_string[0], WHITE);
        lcdDrawString(fx16M, 10, 260, &curr_timeB_string[0], WHITE);

        lcdUpdateVScreen();
    }
}

void app_main()
{
    struct pi_bounds prec = PI_5DIGIT;

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
    xTaskCreate(CalcTaskA,"Calculation Task A",8*2048,&prec,2,&CalcTaskA_hndl);
    xTaskCreate(CalcTaskB,"Calculation Task B",8*2048,&prec,2,&CalcTaskB_hndl);
    xTaskCreate(DisplayTask,"Display Taks", 2*2048,NULL,4,&DisplayTask_hndl);

    if (DEBUG_LOGS) {ESP_LOGI(TAG, "Tasks initialized");}

    for(;;) {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        }
}
