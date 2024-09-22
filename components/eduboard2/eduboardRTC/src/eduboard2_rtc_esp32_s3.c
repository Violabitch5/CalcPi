#include "../../eduboard2.h"
#include "../eduboard2_rtc.h"
#include "gpi2c.h"
#include "time.h"

#define TAG "RTC_Driver"
#define RTC_UPDATE_TIME_MS       10

#define RTC_ADDR 0x51

#define RTCC_SEC        1
#define RTCC_MIN        2
#define RTCC_HR         3
#define RTCC_DAY        4
#define RTCC_WEEKDAY    5
#define RTCC_MONTH      6
#define RTCC_YEAR       7
#define RTCC_CENTURY    8

/* register addresses in the rtc */
#define RTCC_STAT1_ADDR     0x0
#define RTCC_STAT2_ADDR     0x01
#define RTCC_SEC_ADDR       0x02
#define RTCC_MIN_ADDR       0x03
#define RTCC_HR_ADDR        0x04
#define RTCC_DAY_ADDR       0x05
#define RTCC_WEEKDAY_ADDR   0x06
#define RTCC_MONTH_ADDR     0x07
#define RTCC_YEAR_ADDR      0x08
#define RTCC_ALRM_MIN_ADDR  0x09
#define RTCC_SQW_ADDR       0x0D
#define RTCC_TIMER1_ADDR    0x0E
#define RTCC_TIMER2_ADDR    0x0F

/* setting the alarm flag to 0 enables the alarm.
 * set it to 1 to disable the alarm for that value.
 */
#define RTCC_ALARM          0x80
#define RTCC_ALARM_AIE      0x02
#define RTCC_ALARM_AF       0x08
/* optional val for no alarm setting */
#define RTCC_NO_ALARM       99

#define RTCC_TIMER_TIE      0x01  // Timer Interrupt Enable

#define RTCC_TIMER_TF       0x04  // Timer Flag, read/write active state
                                  // When clearing, be sure to set RTCC_TIMER_AF
                                  // to 1 (see note above).
#define RTCC_TIMER_TI_TP    0x10  // 0: INT is active when TF is active
                                  //    (subject to the status of TIE)
                                  // 1: INT pulses active
                                  //    (subject to the status of TIE);
                                  // Note: TF stays active until cleared
                                  // no matter what RTCC_TIMER_TI_TP is.
#define RTCC_TIMER_TD10     0x03  // Timer source clock, TMR_1MIN saves power
#define RTCC_TIMER_TE       0x80  // Timer 1:enable/0:disable

#define RTCC_CENTURY_MASK   0x80
#define RTCC_VLSEC_MASK     0x80

typedef struct {
    /* time variables */
    uint8_t hour;
    uint8_t minute;
    bool volt_low;
    uint8_t sec;
    uint8_t day;
    uint8_t weekday;
    uint8_t month;
    uint8_t year;
    /* alarm */
    uint8_t alarm_hour;
    uint8_t alarm_minute;
    uint8_t alarm_weekday;
    uint8_t alarm_day;
    bool alarm_hour_active;
    bool alarm_minute_active;
    bool alarm_weekday_active;
    bool alarm_day_active;
    /* CLKOUT */
    uint8_t squareWave;
    /* timer */
    uint8_t timer_control;
    uint8_t timer_value;
    /* support */
    uint8_t status1;
    uint8_t status2;
    bool century;
} timedata_t;

timedata_t timedata;

SemaphoreHandle_t rtclock = NULL;

rtc_alarm_mode_t alarmmode;
bool alarm_occured = false;
SemaphoreHandle_t rtcAlarmSemaphore = NULL;
rtc_timer_mode_t timermode;
bool timer_elapsed = false;
SemaphoreHandle_t rtcTimerSemaphore = NULL;

void writePCF8563Register(uint8_t reg, uint8_t data) {
    gpi2c_writeRegister(RTC_ADDR, reg, (uint8_t*)&data, 1);
}
void writePCF8563Registers(uint8_t reg, uint8_t* data, uint8_t len) {
    gpi2c_writeRegister(RTC_ADDR, reg, data, len);
}
void readPCF8563Registers(uint8_t reg, uint8_t* data, uint8_t len) {
    gpi2c_readRegister(RTC_ADDR, reg, data, len);
}

uint8_t decToBcd(uint8_t val)
{
    return ( (val/10*16) + (val%10) );
}
uint8_t bcdToDec(uint8_t val)
{
    return ( (val/16*10) + (val%16) );
}

int yisleap(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}
int get_yday(int mon, int day, int year)
{
    static const int days[2][13] = {
        {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
        {0, 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
    };
    int leap = yisleap(year);

    return days[leap][mon] + day;
}

void getRTCData() {
    uint8_t data[16];
    readPCF8563Registers(RTCC_STAT1_ADDR, &data[0], sizeof(data));
    timedata.status1 = data[0];
    timedata.status2 = data[1];
    timedata.volt_low = data[2] & RTCC_VLSEC_MASK;
    timedata.sec = bcdToDec(data[2] & ~RTCC_VLSEC_MASK);
    timedata.minute = bcdToDec(data[3] & 0x7F);
    timedata.hour = bcdToDec(data[4] & 0x3F);
    timedata.day = bcdToDec(data[5] & 0x3F);
    timedata.weekday = bcdToDec(data[6] & 0x07);
    timedata.month = bcdToDec(data[7] & 0x1F);    
    timedata.century = (data[7] & RTCC_CENTURY_MASK ? true : false);
    timedata.year = bcdToDec(data[8]);
    timedata.alarm_minute = bcdToDec(data[9] & 0x7F);
    timedata.alarm_hour = bcdToDec(data[10] & 0x7F);
    timedata.alarm_day = bcdToDec(data[11] & 0x7F);
    timedata.alarm_weekday = bcdToDec(data[12] & 0x7F);
    timedata.squareWave = data[13] & 0x03;
    timedata.timer_control = data[14] & 0x03;
    timedata.timer_value = data[15];
}

void setDateTime() {
    uint8_t data[] = {  decToBcd(timedata.sec & ~RTCC_VLSEC_MASK),
                        decToBcd(timedata.minute),
                        decToBcd(timedata.hour),
                        decToBcd(timedata.day),
                        decToBcd(timedata.weekday),
                        timedata.month | (timedata.century ? 0x80 : 0x00),
                        decToBcd(timedata.year),
                        };
    writePCF8563Registers(RTCC_SEC_ADDR, &data[0], sizeof(data));
    getRTCData();
}
void clearAlarm() {
    getRTCData();
    timedata.status2 &= ~RTCC_ALARM_AF; //Reset Alarm Flag
    writePCF8563Register(RTCC_STAT2_ADDR, timedata.status2);
}
void enableAlarm() {
    uint8_t data[] = {  decToBcd(timedata.alarm_minute) | (timedata.alarm_minute_active ? 0x00 : 0x80),
                        decToBcd(timedata.alarm_hour) | (timedata.alarm_hour_active ? 0x00 : 0x80),
                        decToBcd(timedata.alarm_day) | (timedata.alarm_day_active ? 0x00 : 0x80),
                        decToBcd(timedata.alarm_weekday) | (timedata.alarm_weekday_active ? 0x00 : 0x80)
                        };
    writePCF8563Registers(RTCC_ALRM_MIN_ADDR, &data[0], sizeof(data));
    clearAlarm();
}
void disableAlarm() {
    uint8_t data[] = {  decToBcd(timedata.alarm_minute) | (0x80),
                        decToBcd(timedata.alarm_hour) | (0x80),
                        decToBcd(timedata.alarm_day) | (0x80),
                        decToBcd(timedata.alarm_weekday) | (0x80)
                        };
    writePCF8563Registers(RTCC_ALRM_MIN_ADDR, &data[0], sizeof(data));
    clearAlarm();
}
void writeTimerData() {
    writePCF8563Register(RTCC_TIMER1_ADDR, timedata.timer_control);
    writePCF8563Register(RTCC_TIMER2_ADDR, timedata.timer_value);
    getRTCData();
    timedata.status2 &= ~RTCC_TIMER_TF;
    writePCF8563Register(RTCC_STAT2_ADDR, timedata.status2);
}

void initClock() {
    uint8_t data[] = {  0x00, //status1
                        0x00, //status2
                        0x81, //set seconds & VL
                        0x01, //set minutes
                        0x01, //set hour
                        0x01, //set day
                        0x01, //set weekday
                        0x01, //set month, century to 1
                        0x01, //set year to 99
                        0x80, //minute alarm value reset to 00
                        0x80, //hour alarm value reset to 00
                        0x80, //day alarm value reset to 00
                        0x80, //weekday alarm value reset to 00
                        0x00, //set SQW
                        0x00};//timer off
    writePCF8563Registers(RTCC_STAT1_ADDR, &data[0], sizeof(data));
}
void debugRTCData() {
    uint8_t data[16];
    readPCF8563Registers(RTCC_STAT1_ADDR, &data[0], sizeof(data));
    ESP_LOGI(TAG, "RTC-Data Readback:");
    for(int i = 0; i < sizeof(data); i++) {
        ESP_LOGI(TAG, "Reg:0x%02X - 0x%02X", i, data[i]);
    }
    ESP_LOGI(TAG, "RTC-Data Readback done.");
}

void rtc_task(void* param) {
    ESP_LOGI(TAG, "Init RTC PCF8563...");
    rtclock = xSemaphoreCreateMutex();
    gpi2c_init(GPIO_I2C_SDA, GPIO_I2C_SCL, 400000);
#ifdef CONFIG_RTC_RESET_DATA_ON_STARTUP
    initClock();
    timedata.hour = 18;
    timedata.minute = 15;
    timedata.sec = 0;
    setDateTime();
#endif

    ESP_LOGI(TAG, "Init RTC PCF8563 done");
    for(;;) {
        xSemaphoreTake(rtclock, portMAX_DELAY);
        getRTCData();
        if(timedata.status2 & RTCC_ALARM_AF) {
            //Alarm occured
            if(alarmmode == RTCALARM_ENABLED) {
                if(rtcAlarmSemaphore != NULL) {
                    xSemaphoreGive(rtcAlarmSemaphore);
                }
                alarm_occured = true;
            }
        }
        if(timedata.status2 & RTCC_TIMER_TF) {
            //Timer elapsed
            if(timermode == RTCTIMER_ENABLED) {
                if(rtcTimerSemaphore != NULL) {
                    xSemaphoreGive(rtcTimerSemaphore);
                }
                timer_elapsed = true;
            }
        }
#ifdef CONFIG_RTC_SHOW_TIME        
        ESP_LOGI(TAG, "Time: %02i:%02i:%02i", timedata.hour, timedata.minute, timedata.sec);
#endif
        xSemaphoreGive(rtclock);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void eduboard_init_rtc() {
    xTaskCreate(rtc_task, "rtc_task", 2*2048, NULL, 10, NULL);
}

void rtc_set_alarm_time(int8_t hour, int8_t minute, int8_t day, int8_t weekday) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    if(hour >= 0 && hour < 24) {
        timedata.alarm_hour = hour;
        timedata.alarm_hour_active = true;
    } else {
        timedata.alarm_hour_active = false;
    }
    if(minute >= 0 && minute < 60) {
        timedata.alarm_minute = minute;
        timedata.alarm_minute_active = true;
    } else {
        timedata.alarm_minute_active = false;
    }
    if(day >= 0 && day < 32) {
        timedata.alarm_day = day;
        timedata.alarm_day_active = true;
    } else {
        timedata.alarm_day_active = false;
    }
    if(weekday >= 0 && weekday < 7) {
        timedata.alarm_weekday = weekday;
        timedata.alarm_weekday_active = true;
    } else {
        timedata.alarm_weekday_active = false;
    }
    xSemaphoreGive(rtclock);
}
void rtc_config_alarm(rtc_alarm_mode_t mode, SemaphoreHandle_t alarm_semaphore) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    rtcAlarmSemaphore = alarm_semaphore;    
    if(rtcAlarmSemaphore != NULL) {
        xSemaphoreGive(rtcAlarmSemaphore);
        xSemaphoreTake(rtcAlarmSemaphore, portMAX_DELAY);
    }
    alarmmode = mode;
    if(alarmmode == RTCALARM_ENABLED) {
        enableAlarm();
    } else {
        disableAlarm();
    }
    alarm_occured = false;
    xSemaphoreGive(rtclock);
}

void rtc_set_timer_time(uint8_t value) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    timedata.timer_value = value;
    xSemaphoreGive(rtclock);
}
void rtc_config_timer(rtc_timer_mode_t mode, SemaphoreHandle_t timer_semaphore, rtc_timer_frequency_t frequency_mode) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    rtcTimerSemaphore = timer_semaphore;
    if(rtcTimerSemaphore != NULL) {
        xSemaphoreGive(rtcTimerSemaphore);
        xSemaphoreTake(rtcTimerSemaphore, portMAX_DELAY);
    }
    timermode = mode;
    timedata.timer_control = (frequency_mode & 0x03) | (timermode == RTCTIMER_ENABLED ? 0x80 : 0x00);
    writeTimerData();
    timer_elapsed = false;
    xSemaphoreGive(rtclock);
}

bool rtc_alarm_occured() {
    bool returnValue = false;
    if(rtclock == NULL) return false;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    returnValue = alarm_occured;
    alarm_occured = false;
    xSemaphoreGive(rtclock);
    return returnValue;
}
bool rtc_timer_elapsed() {
    bool returnValue = false;
    if(rtclock == NULL) return false;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    returnValue = timer_elapsed;
    timer_elapsed = false;
    xSemaphoreGive(rtclock);
    return returnValue;
}

void rtc_set_time(uint8_t hour, uint8_t minute, uint8_t sec) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    getRTCData();
    timedata.hour = hour;
    timedata.minute = minute;
    timedata.sec = sec;
    setDateTime();
    xSemaphoreGive(rtclock);
}
void rtc_set_date(uint8_t day, uint8_t weekday, uint8_t month, uint16_t year) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    getRTCData();
    if(year >= 1900 && year < 2100) {
        if(year < 2000) {
            timedata.year = year - 1900;
            timedata.century = false;
        } else {
            timedata.year = year - 2000;
            timedata.century = true;
        }
    } else if(year < 100) {
        timedata.year = year;
        timedata.century = true;
    }
    timedata.month = month;
    timedata.day = day;
    timedata.weekday = weekday;
    setDateTime();
    xSemaphoreGive(rtclock);
}
void rtc_get_time(uint8_t* hour, uint8_t* minute, uint8_t* sec) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    getRTCData();
    *hour = timedata.hour;
    *minute = timedata.minute;
    *sec = timedata.sec;
    xSemaphoreGive(rtclock);
}
void rtc_get_date(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday) {
    if(rtclock == NULL) return;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    getRTCData();
    if(timedata.century) {
        *year = 2000 + timedata.year;
    } else {
        *year = 1900 + timedata.year;
    }
    *month = timedata.month;
    *day = timedata.day;
    *weekday = timedata.weekday;
    xSemaphoreGive(rtclock);
}

uint32_t rtc_get_unix_timestamp() {
    if(rtclock == NULL) return 0;
    xSemaphoreTake(rtclock, portMAX_DELAY);
    getRTCData();

    uint16_t year = (timedata.century ? 2000 + timedata.year : 1900 + timedata.year);
    uint16_t yday = get_yday(timedata.month, timedata.day, timedata.year)-1;
    if(year < 1970) {
        xSemaphoreGive(rtclock);
        return 0;
    }
    year -= 1900;
    
    uint32_t timestamp = 0;
    timestamp = timedata.sec + timedata.minute*60 + timedata.hour*3600 + yday*86400 +
    (year-70)*31536000 + ((year-69)/4)*86400 -
    ((year-1)/100)*86400 + ((year+299)/400)*86400;
    xSemaphoreGive(rtclock);
    return timestamp;
}