#pragma once
#include "../eduboard2_config.h"

#ifdef CONFIG_ENABLE_SENSOR_STK8321
void stk8321_anymotion_init();
void stk8321_sigmotion_init();
void stk8321_disable_motion();
int  stk8321_get_sensitivity();
void stk8321_get_motion_data(float *X_DataOut, float *Y_DataOut, float *Z_DataOut);
void eduboard_init_stk8321(void);
#endif

#ifdef CONFIG_ENABLE_SENSOR_TMP112
float tmp112_get_value(void);
void  eduboard_init_tmp112(void);
#endif