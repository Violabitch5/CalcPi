#include "../../eduboard2.h"
#include "../eduboard2_sensor.h"

#define TAG "Sensor_STK8321_Driver"

#define STK8xxx_SLAVE_ADDRESS	0x0F

#define STK_REG_CHIPID		    0x00
/* STK8321 or STK8323 CHIP ID = 0x23 */
#define STK8xxx_CHIPID_VAL      0x23
#define STK8327_CHIPID_VAL      0x26
/* STK8BAxx */
 /* S or R resolution = 10 bit */
#define STK8BA50_X_CHIPID_VAL   0x86
#define STK8BA5X_CHIPID_VAL     0x87

#define	STK8xxx_REG_POWMODE     0x11
#define STK8xxx_VAL_SLEEP_05    0b0000
#define STK8xxx_VAL_SLEEP_1     0b0110
#define STK8xxx_VAL_SLEEP_2     0b0111
#define STK8xxx_VAL_SLEEP_4     0b1000
#define STK8xxx_VAL_SLEEP_6     0b1001
#define STK8xxx_VAL_SLEEP_10    0b1010
#define STK8xxx_VAL_SLEEP_25    0b1011
#define STK8xxx_VAL_SLEEP_50    0b1100
#define STK8xxx_VAL_SLEEP_100   0b1101
#define STK8xxx_VAL_SLEEP_500   0b1110
#define STK8xxx_VAL_SLEEP_1000  0b1111
#define	STK8xxx_VAL_LOWPOWER    1 << 6
#define	STK8xxx_VAL_SUSPEND     1 << 7

#define STK8xxx_REG_RANGESEL	0x0F
#define STK8xxx_RANGE_2G		0x03
#define STK8xxx_RANGE_4G		0x05
#define STK8xxx_RANGE_8G		0x08

#define STK8xxx_REG_XOUT1       0x02
#define STK8xxx_REG_XOUT2       0x03
#define STK8xxx_REG_YOUT1       0x04
#define STK8xxx_REG_YOUT2       0x05
#define STK8xxx_REG_ZOUT1       0x06
#define STK8xxx_REG_ZOUT2       0x07

#define STK8xxx_REG_BWSEL       0x10
#define STK8xxx_VAL_BW_7_81     0b01000
#define STK8xxx_VAL_BW_15_63    0b01001
#define STK8xxx_VAL_BW_31_25    0b01010
#define STK8xxx_VAL_BW_62_5     0b01011
#define STK8xxx_VAL_BW_125      0b01100
#define STK8xxx_VAL_BW_250      0b01101
#define STK8xxx_VAL_BW_500      0b01110
#define STK8xxx_VAL_BW_1000     0b01111

#define STK8xxx_REG_SWRST       0x14
#define STK8xxx_VAL_RST_DFLTS   0xB6

#define STK8xxx_REG_INTEN1      0x16
#define STK8xxx_VAL_SLP_EN_X    1 << 0
#define STK8xxx_VAL_SLP_EN_Y    1 << 1
#define STK8xxx_VAL_SLP_EN_Z    1 << 2

#define STK8xxx_REG_INTCFG1     0x20
#define STK8xxx_VAL_INT_LV      1 << 0
#define STK8xxx_VAL_INT_OD      1 << 1

#define STK8xxx_REG_SLOPETHD    0x28
#define STK8xxx_VAL_SLP_DFLT    150 // 0x14

#define STK8xxx_REG_SIGMOT2     0x2A
#define STK8xxx_VAL_SKIP_TIME   1 << 0
#define STK8xxx_VAL_SIG_MOT_EN  1 << 1
#define STK8xxx_VAL_ANY_MOT_EN  1 << 2

#define STK8xxx_REG_INTMAP1     0x19
#define STK8xxx_VAL_SIGMOT2INT1 1 << 0
#define STK8xxx_VAL_ANYMOT2INT1 1 << 2

#define STK8xxx_REG_INTFCFG     0x34
#define STK8xxx_VAL_I2C_WDT_SEL 1 << 1
#define STK8xxx_VAL_I2C_WDT_EN  1 << 2

// static esp_err_t read_register16(uint8_t reg, uint16_t *r)
// {
// 	return gpi2c_readRegister(TMP112_I2C_ADDR_FLAGS, reg, (uint8_t*)r, 2);
// }
// inline static esp_err_t write_register16(uint8_t reg, uint16_t data)
// {
// 	return gpi2c_writeRegister(TMP112_I2C_ADDR_FLAGS, reg, (uint8_t*)&data, 2);
// }

#define PID_SIZE	16
uint8_t chipid_temp = 0x00;
uint8_t stk8xxx_pid_list[PID_SIZE] = {STK8xxx_CHIPID_VAL, STK8BA50_X_CHIPID_VAL, STK8BA5X_CHIPID_VAL, STK8327_CHIPID_VAL};

void ReadAccRegister(uint8_t reg, uint8_t *data) {
    gpi2c_readRegister(STK8xxx_SLAVE_ADDRESS, reg, data, 1);
}
void WriteAccRegister(uint8_t reg, uint8_t data) {
    gpi2c_writeRegister(STK8xxx_SLAVE_ADDRESS, reg, &data, 1);
}
void STK8xxx_Suspend_mode() {
    uint8_t RegAddr, RegWriteValue;

    /* suspend mode enable */
	RegAddr       = STK8xxx_REG_POWMODE;
    RegWriteValue = STK8xxx_VAL_SUSPEND;
    WriteAccRegister(RegAddr, RegWriteValue);
}
bool STK8xxx_Check_chipid() {
    uint8_t RegAddr = STK_REG_CHIPID;
    int i = 0, pid_num = (sizeof(stk8xxx_pid_list) / sizeof(stk8xxx_pid_list[0]));

    ReadAccRegister(RegAddr, &chipid_temp);
    for (i = 0; i < pid_num; i++)
    {
        if (chipid_temp == stk8xxx_pid_list[i])
        {
        	//("read stkchip id ok, chip_id = 0x%x", chipid_temp);
            return true;
        }
    }
	//ERRLN("read stkchip id fail!");
    return false;
}

void stk8321_anymotion_init() {
    ESP_LOGI(TAG, "Set Mode to Anymotion");
    uint8_t ARegAddr, ARegWriteValue;

	/* Enable X Y Z-axis any-motion (slope) interrupt */
    ARegAddr       = STK8xxx_REG_INTEN1;
    ARegWriteValue = STK8xxx_VAL_SLP_EN_X | STK8xxx_VAL_SLP_EN_Y | STK8xxx_VAL_SLP_EN_Z;
    WriteAccRegister(ARegAddr, ARegWriteValue);

	/* Set anymotion Interrupt trigger threshold */
    ARegAddr       = STK8xxx_REG_SLOPETHD;
    ARegWriteValue = STK8xxx_VAL_SLP_DFLT;
    WriteAccRegister(ARegAddr, ARegWriteValue);

	/* Enable any-motion */
    ARegAddr       = STK8xxx_REG_SIGMOT2;
    ARegWriteValue = STK8xxx_VAL_ANY_MOT_EN;
    WriteAccRegister(ARegAddr, ARegWriteValue);

	/* Map any-motion (slope) interrupt to INT1 */
    ARegAddr       = STK8xxx_REG_INTMAP1;
    ARegWriteValue = STK8xxx_VAL_ANYMOT2INT1;
    WriteAccRegister(ARegAddr, ARegWriteValue);
}
void stk8321_sigmotion_init() {
    ESP_LOGI(TAG, "Set Mode to Sigmotion");
    uint8_t SRegAddr, SRegWriteValue;

	/* Enable X Y Z-axis sig-motion (slope) interrupt */
    SRegAddr       = STK8xxx_REG_INTEN1;
    SRegWriteValue = STK8xxx_VAL_SLP_EN_X | STK8xxx_VAL_SLP_EN_Y | STK8xxx_VAL_SLP_EN_Z;
    WriteAccRegister(SRegAddr, SRegWriteValue);

	/* Set sig-motion Interrupt trigger threshold */
    SRegAddr       = STK8xxx_REG_SLOPETHD;
    SRegWriteValue = STK8xxx_VAL_SLP_DFLT;
    WriteAccRegister(SRegAddr, SRegWriteValue);

	/* Enable significant motion */
    SRegAddr       = STK8xxx_REG_SIGMOT2;
    SRegWriteValue = STK8xxx_VAL_SIG_MOT_EN;
    WriteAccRegister(SRegAddr, SRegWriteValue);

	/* Map significant motion interrupt to INT1 */
    SRegAddr       = STK8xxx_REG_INTMAP1;
    SRegWriteValue = STK8xxx_VAL_SIGMOT2INT1;
    WriteAccRegister(SRegAddr, SRegWriteValue);
}
void stk8321_disable_motion() {
    uint8_t ARegAddr, ARegWriteValue;

	/* Disable X Y Z-axis motion (slope) interrupt */
    ARegAddr       = STK8xxx_REG_INTEN1;
    ARegWriteValue = 0x00;
    WriteAccRegister(ARegAddr, ARegWriteValue);

	/* Disable motion */
    ARegAddr       = STK8xxx_REG_SIGMOT2;
    ARegWriteValue = 0x00;
    WriteAccRegister(ARegAddr, ARegWriteValue);
}
int stk8321_init() {
    ESP_LOGI(TAG, "Init STK8321...");
    uint8_t RegAddr, RegWriteValue;

	if(!STK8xxx_Check_chipid())
	{
		return -1;
    }

    /* soft-reset */
	RegAddr       = STK8xxx_REG_SWRST;
    RegWriteValue = STK8xxx_VAL_RST_DFLTS;
    WriteAccRegister(RegAddr, RegWriteValue);
	vTaskDelay(50/portTICK_PERIOD_MS);//unit ms

    /* set range, resolution */
    RegAddr       = STK8xxx_REG_RANGESEL;
    RegWriteValue = STK8xxx_RANGE_4G; // range = +/-4g
    WriteAccRegister(RegAddr, RegWriteValue);

	/* set power mode */
	RegAddr 	  = STK8xxx_REG_POWMODE;
	RegWriteValue = STK8xxx_VAL_SLEEP_05;	// active mode
	WriteAccRegister(RegAddr, RegWriteValue);

	/* set bandwidth */
    RegAddr       = STK8xxx_REG_BWSEL;
    RegWriteValue = STK8xxx_VAL_BW_1000; // bandwidth = 1000Hz
    WriteAccRegister(RegAddr, RegWriteValue);

	//STK8xxx_Anymotion_init();
	//STK8xxx_Sigmotion_init();

    /* set i2c watch dog */
    RegAddr       = STK8xxx_REG_INTFCFG;
    RegWriteValue = STK8xxx_VAL_I2C_WDT_EN; // enable watch dog
    WriteAccRegister(RegAddr, RegWriteValue);

    /* int config */
    RegAddr       = STK8xxx_REG_INTCFG1;
    RegWriteValue = STK8xxx_VAL_INT_LV; // INT1/INT2 push-pull, active high
    WriteAccRegister(RegAddr, RegWriteValue);
    ESP_LOGI(TAG, "Init STK8321 done");
	return chipid_temp;
}
int stk8321_get_sensitivity() {
    int sensitivity = 0;
	if(0x86 == chipid_temp)
	{
	   //resolution = 10 bit
       sensitivity = 1 << 9;
	}
    else
    {
       //resolution = 12 bit
       sensitivity = 1 << 11;
	}
    //range = +/-4g
    sensitivity = sensitivity / 4;
    return sensitivity;
}
void stk8321_get_motion_data(float *X_DataOut, float *Y_DataOut, float *Z_DataOut) {
    uint8_t RegAddr, RegReadValue[2];
    int16_t x, y, z;

    RegAddr      	= STK8xxx_REG_XOUT1;
    RegReadValue[0] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[0]);
    RegAddr      	= STK8xxx_REG_XOUT2;
    RegReadValue[1] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[1]);
	x = (short int)(RegReadValue[1] << 8 | RegReadValue[0]);

	RegAddr      	= STK8xxx_REG_YOUT1;
    RegReadValue[0] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[0]);
    RegAddr      	= STK8xxx_REG_YOUT2;
    RegReadValue[1] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[1]);
	y = (short int)(RegReadValue[1] << 8 | RegReadValue[0]);

	RegAddr      	= STK8xxx_REG_ZOUT1;
    RegReadValue[0] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[0]);
    RegAddr      	= STK8xxx_REG_ZOUT2;
    RegReadValue[1] = 0x00;
	ReadAccRegister(RegAddr, &RegReadValue[1]);
	z = (short int)(RegReadValue[1] << 8 | RegReadValue[0]);

	if(0x86 == chipid_temp)
	{
		//resolution = 10 bit
        x >>= 6;
        *X_DataOut = (float) x / stk8321_get_sensitivity();

        y >>= 6;
        *Y_DataOut = (float) y / stk8321_get_sensitivity();

        z >>= 6;
        *Z_DataOut = (float) z / stk8321_get_sensitivity();
	}
    else
    {
        //resolution = 12 bit
        x >>= 4;
        *X_DataOut = (float) x / stk8321_get_sensitivity();

        y >>= 4;
        *Y_DataOut = (float) y / stk8321_get_sensitivity();

        z >>= 4;
        *Z_DataOut = (float) z / stk8321_get_sensitivity();
	}
}

void eduboard_init_stk8321(void)
{
	gpi2c_init(GPIO_I2C_SDA, GPIO_I2C_SCL, 400000);
    stk8321_init();
}