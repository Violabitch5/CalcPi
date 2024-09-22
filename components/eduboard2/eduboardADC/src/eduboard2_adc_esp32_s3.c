#include "../../eduboard2.h"
#include "../eduboard2_adc.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "ADC_Driver"

SemaphoreHandle_t hADCMutex;

static int adc_raw;
static int voltage;


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void adcTask(void * parameter) {
    ESP_LOGI(TAG, "init ADC...");
    hADCMutex = xSemaphoreCreateMutex();

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
#ifdef ADC_ATTEN_DB_12
        .atten = ADC_ATTEN_DB_12,
#else
        .atten = ADC_ATTEN_DB_11,
#endif
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, AN0_CHANNEL, &config));
    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_handle = NULL;
#ifdef ADC_ATTEN_DB_12
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_cali_handle);
#else
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
#endif
    ESP_LOGI(TAG, "ADC init done");
    for(;;) {
        xSemaphoreTake(hADCMutex, portMAX_DELAY);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, AN0_CHANNEL, &adc_raw));

#ifdef CONFIG_ADC_DEBUG
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, AN0_CHANNEL, adc_raw);
#endif
        if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
#ifdef CONFIG_ADC_DEBUG
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, AN0_CHANNEL, voltage);
#endif
        }
        xSemaphoreGive(hADCMutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

uint32_t adc_get_raw() {
    uint32_t returnValue = 0;
    xSemaphoreTake(hADCMutex, portMAX_DELAY);
    returnValue = adc_raw;
    xSemaphoreGive(hADCMutex);
    return returnValue;
}
uint32_t adc_get_voltage_mv() {
    uint32_t returnValue = 0;
    xSemaphoreTake(hADCMutex, portMAX_DELAY);
    returnValue = voltage;
    xSemaphoreGive(hADCMutex);
    return returnValue;
}
void eduboard_init_adc() {
    xTaskCreate(adcTask, "adcTask", 2 * 2048, NULL, 5, NULL);
}