#pragma once

#if defined(PLATFORM_ESP32)

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"


static inline bool ESP32AdcCalibration_create(int pin, adc_atten_t attenuation, adc_cali_handle_t *handle)
{
    if (!handle)
        return false;

    *handle = nullptr;

    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    if (adc_oneshot_io_to_channel(pin, &unit, &channel) != ESP_OK)
        return false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t config = {};
    config.unit_id = unit;
    config.chan = channel;
    config.atten = attenuation;
    config.bitwidth = ADC_BITWIDTH_12;
    return adc_cali_create_scheme_curve_fitting(&config, handle) == ESP_OK;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t config = {};
    config.unit_id = unit;
    config.atten = attenuation;
    config.bitwidth = ADC_BITWIDTH_12;
#if CONFIG_IDF_TARGET_ESP32
    config.default_vref = 0;
#endif
    return adc_cali_create_scheme_line_fitting(&config, handle) == ESP_OK;
#else
    (void)pin;
    (void)attenuation;
    return false;
#endif
}

static inline void ESP32AdcCalibration_destroy(adc_cali_handle_t handle)
{
    if (!handle)
        return;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

static inline bool ESP32AdcCalibration_rawToMilliVolts(adc_cali_handle_t handle, uint32_t raw, uint32_t *milliVolts)
{
    if (!handle || !milliVolts)
        return false;

    int voltage = 0;
    if (adc_cali_raw_to_voltage(handle, raw, &voltage) != ESP_OK || voltage < 0)
        return false;

    *milliVolts = (uint32_t)voltage;
    return true;
}

#endif
