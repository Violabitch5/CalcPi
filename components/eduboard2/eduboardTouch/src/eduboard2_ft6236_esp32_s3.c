#include "../../eduboard2.h"
#include "../eduboard2_touch.h"

#define TAG "FT6236_driver"

//#define FT6236_ADDR             0x36
#define FT6236_ADDR             0x38 //I2C Address

#define FT6236_G_FT5201ID       0xA8 // FocalTech's panel ID
#define FT6236_REG_NUMTOUCHES   0x02 // Number of touch points

#define FT6236_NUM_X            0x33 // Touch X position
#define FT6236_NUM_Y            0x34 // Touch Y position

#define FT6236_REG_MODE         0x00 // Device mode, either WORKING or FACTORY
#define FT6236_REG_CALIBRATE    0x02 // Calibrate mode
#define FT6236_REG_WORKMODE     0x00 // Work mode
#define FT6236_REG_FACTORYMODE  0x40 // Factory mode
#define FT6236_REG_THRESHHOLD   0x80 // Threshold for touch detection
#define FT6236_REG_CTRL         0x86 //Control Register
#define FT6236_REG_POINTRATE    0x88 // Point rate
#define FT6236_REG_FIRMVERS     0xA6 // Firmware version
#define FT6236_REG_CHIPID       0xA3 // Chip selecting
#define FT6236_REG_G_MODE       0xA4 // Chip selecting
#define FT6236_REG_VENDID       0xA8 // FocalTech's panel ID

#define FT6236_VENDID           0x11 // FocalTech's panel ID
#define FT6206_CHIPID           0x06 // FT6206 ID
#define FT6236_CHIPID           0x36 // FT6236 ID
#define FT6236U_CHIPID          0x64 // FT6236U ID

#define FT6236_DEFAULT_THRESHOLD    128 // Default threshold for touch detection
#define FT6236_REFRESH_RATE_HZ      50

SemaphoreHandle_t touchlock;
touchevent_t touchevent;

void writeFT6236TouchRegister(uint8_t reg, uint8_t val) {
    gpi2c_writeRegister(FT6236_ADDR, reg, (uint8_t*)&val, 1);
}
uint8_t readFT6236TouchRegister(uint8_t reg) {
    uint8_t returnvalue = 0;
    gpi2c_readRegister(FT6236_ADDR, reg, &returnvalue, 1);
    return returnvalue;
}
uint8_t readFT6236TouchLocation(touchevent_t *te) {
    uint8_t touchdata[16];
    gpi2c_readRegister(FT6236_ADDR, 0, &touchdata[0], 16);
    
    uint8_t touches = touchdata[2];
    if(touches>0 && touches <= 2) {
        te->touches = touches;
        te->points[0].x = ((touchdata[3]&0x0F)<<8) + touchdata[4];
        te->points[0].y = ((touchdata[5]&0x0F)<<8) + touchdata[6];
        if(touches>1) {
            te->points[1].x = ((touchdata[0x09]&0x0F)<<8) + touchdata[0x0A];
            te->points[1].y = ((touchdata[0x0B]&0x0F)<<8) + touchdata[0x0C];
        }
        return touches;
    }
    return 0;
}
void currentTouchEvent() {

}

void touchupdate_task(void* param) {
    touchevent_t te;
    for(;;) {
        if(readFT6236TouchLocation(&te) > 0) {
            xSemaphoreTake(touchlock, portMAX_DELAY);
            touchevent.touches = te.touches;
            touchevent.points[0].x = te.points[0].x;
            touchevent.points[0].y = te.points[0].y;
            touchevent.points[1].x = te.points[1].x;
            touchevent.points[1].y = te.points[1].y;
            xSemaphoreGive(touchlock);
        }
        vTaskDelay((1000/FT6236_REFRESH_RATE_HZ)/portTICK_PERIOD_MS);
    }
}

void eduboard_init_ft6236(void) {
    ESP_LOGI(TAG, "Init FT6236...");
    touchlock = xSemaphoreCreateMutex();
    gpi2c_init(GPIO_I2C_SDA, GPIO_I2C_SCL, 400000);
    writeFT6236TouchRegister(FT6236_REG_MODE, 0x00); // device mode = Normal
    writeFT6236TouchRegister(FT6236_REG_G_MODE, 0x01); // Interrupt trigger mode
    writeFT6236TouchRegister(FT6236_REG_POINTRATE, FT6236_REFRESH_RATE_HZ); // Set Refresh Rate to 50Hz
    writeFT6236TouchRegister(FT6236_REG_CTRL, 0); // Active Mode.
    xTaskCreate(touchupdate_task, "touch_task", 2*2048, NULL, 3, NULL);
    ESP_LOGI(TAG, "Init FT6236 done.");
}

void rotate_touchpoint(touchevent_t* te) {
    int x1 = te->points[0].x;
    int y1 = te->points[0].y;
    int x2 = te->points[1].x;
    int y2 = te->points[1].y;
    switch(SCREEN_ROTATION) {
        case 0:
            te->points[0].x = x1;
            te->points[0].y = y1;
            te->points[1].x = x2;
            te->points[1].y = y2;
        break;
        case 90:
            te->points[0].x = y1;
            te->points[0].y = SCREEN_MAX_Y - x1;
            te->points[1].x = y2;
            te->points[1].y = SCREEN_MAX_Y - x2;
        break;
        case 180:
            te->points[0].x = x1;
            te->points[0].y = y1;
            te->points[1].x = x2;
            te->points[1].y = y2;
        break;
        case 270:
            te->points[0].x = x1;
            te->points[0].y = y1;
            te->points[1].x = x2;
            te->points[1].y = y2;
        break;
        default:
    }
}

touchevent_t ft6236_get_touch_event(bool reset) {
    touchevent_t returnvalue;
    xSemaphoreTake(touchlock, portMAX_DELAY);
    memcpy(&returnvalue, &touchevent, sizeof(touchevent_t));
    if(reset == true) {
        memset(&touchevent, 0, sizeof(touchevent_t));
    }
    xSemaphoreGive(touchlock);

    rotate_touchpoint(&returnvalue);

    return returnvalue;
}

bool ft6236_is_touched() {
    xSemaphoreTake(touchlock, portMAX_DELAY);
    uint8_t touches = touchevent.touches;
    xSemaphoreGive(touchlock);
    return (touches > 0 ? true : false);
}