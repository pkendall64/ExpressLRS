#include "VbatCalibration.h"

#include <Arduino.h>
#include "ESP32AdcCalibration.h"


#if defined(PLATFORM_ESP8266)
static constexpr uint8_t VOLTAGE_SOURCE_COUNT = 1;
static constexpr uint16_t ADC_MAX_VALUE = 1023;
static constexpr uint16_t ADC_SATURATION_MARGIN = 6;
#else
static constexpr uint8_t VOLTAGE_SOURCE_COUNT =4;
static constexpr uint16_t ADC_MAX_VALUE = 4095;
static constexpr uint16_t ADC_SATURATION_MARGIN = 24;
#endif

#if defined(PLATFORM_ESP32)
static constexpr int CALIBRATED_ATTENUATION_START = 4;
#endif

typedef struct {
    const char *id;
    const char *label;
    nameType hardwarePin;
    nameType offset;
    nameType scale;
    nameType atten;
    nameType noReading;
    nameType calMin;
    nameType calMax;
} voltageSourceCalibration_t;

static const voltageSourceCalibration_t voltageSources[VOLTAGE_SOURCE_COUNT] = {
    {"vbat", "VBat", HARDWARE_vbat, HARDWARE_vbat_offset, HARDWARE_vbat_scale, HARDWARE_vbat_atten, HARDWARE_vbat_noreading, HARDWARE_vbat_cal_min, HARDWARE_vbat_cal_max},
#if defined(PLATFORM_ESP32)
    {"vsrc1", "VSrc1", HARDWARE_vsrc1, HARDWARE_vsrc1_offset, HARDWARE_vsrc1_scale, HARDWARE_vsrc1_atten, HARDWARE_vsrc1_noreading, HARDWARE_vsrc1_cal_min, HARDWARE_vsrc1_cal_max},
    {"vsrc2", "VSrc2", HARDWARE_vsrc2, HARDWARE_vsrc2_offset, HARDWARE_vsrc2_scale, HARDWARE_vsrc2_atten, HARDWARE_vsrc2_noreading, HARDWARE_vsrc2_cal_min, HARDWARE_vsrc2_cal_max},
    {"vsrc3", "VSrc3", HARDWARE_vsrc3, HARDWARE_vsrc3_offset, HARDWARE_vsrc3_scale, HARDWARE_vsrc3_atten, HARDWARE_vsrc3_noreading, HARDWARE_vsrc3_cal_min, HARDWARE_vsrc3_cal_max}
#endif
};

static bool sourceIsDefined(const uint8_t sourceIdx)
{
    return hardware_pin(voltageSources[sourceIdx].hardwarePin) != UNDEF_PIN;
}

#if defined(PLATFORM_ESP32)
static bool useCalibratedReading(const int atten)
{
    return atten >= CALIBRATED_ATTENUATION_START;
}

static adc_atten_t getSamplingAttenuation(const int atten)
{
    if (useCalibratedReading(atten))
        return (adc_atten_t)(atten - CALIBRATED_ATTENUATION_START);
    if (atten == -1)
        return ADC_ATTEN_DB_12;
    return (adc_atten_t)atten;
}
#endif


static void sortSamples(uint16_t *values, const uint8_t count)
{
    for (uint8_t i = 1; i < count; ++i)
    {
        const uint16_t value = values[i];
        int pos = i - 1;
        while (pos >= 0 && values[pos] > value)
        {
            values[pos + 1] = values[pos];
            --pos;
        }
        values[pos + 1] = value;
    }
}

static void summarizeSamples(uint16_t *rawValues, uint16_t *adcValues, const uint8_t count, voltage_source_sample_t *sample)
{
    sortSamples(rawValues, count);
    sortSamples(adcValues, count);
    sample->rawMax = rawValues[count-1];
    if ((count & 1) == 0)
    {
        sample->rawMedian = (rawValues[(count / 2) - 1] + rawValues[count / 2]) / 2;
        sample->adcMedian = (adcValues[(count / 2) - 1] + adcValues[count / 2]) / 2;
    }
    else
    {
        sample->rawMedian = rawValues[count / 2];
        sample->adcMedian = adcValues[count / 2];
    }
}

uint8_t VbatCalibration_getSourceCount()
{
    return VOLTAGE_SOURCE_COUNT;
}

bool VbatCalibration_findSource(const char *sourceId, uint8_t *sourceIdx)
{
    if (!sourceId)
        return false;

    for (uint8_t idx = 0; idx < VOLTAGE_SOURCE_COUNT; ++idx)
    {
        if (strcmp(sourceId, voltageSources[idx].id) == 0)
        {
            if (sourceIdx)
                *sourceIdx = idx;
            return true;
        }
    }

    return false;
}

bool VbatCalibration_isSourceDefined(const uint8_t sourceIdx)
{
    return sourceIdx < VOLTAGE_SOURCE_COUNT && sourceIsDefined(sourceIdx);
}

void VbatCalibration_getSourceConfig(const uint8_t sourceIdx, voltage_source_config_t *config)
{
    if (!config || sourceIdx >= VOLTAGE_SOURCE_COUNT)
        return;

    config->id = voltageSources[sourceIdx].id;
    config->label = voltageSources[sourceIdx].label;
    config->pin = hardware_pin(voltageSources[sourceIdx].hardwarePin);
    config->offset = hardware_int(voltageSources[sourceIdx].offset);
    config->scale = hardware_int(voltageSources[sourceIdx].scale);
    config->atten = hardware_int(voltageSources[sourceIdx].atten);
    config->noReading = hardware_int(voltageSources[sourceIdx].noReading);
    config->calMin = hardware_int(voltageSources[sourceIdx].calMin);
    config->calMax = hardware_int(voltageSources[sourceIdx].calMax);
}

bool VbatCalibration_sampleSource(const uint8_t sourceIdx, int atten, uint8_t samples, voltage_source_sample_t *sample)
{
    if (!sample || sourceIdx >= VOLTAGE_SOURCE_COUNT || !sourceIsDefined(sourceIdx))
        return false;

    samples = constrain(samples, 24, 64);
    memset(sample, 0, sizeof(*sample));

#if defined(PLATFORM_ESP32)
    adc_cali_handle_t calibration = nullptr;
    const int sourcePin = hardware_pin(voltageSources[sourceIdx].hardwarePin);
    analogReadResolution(12);
    const adc_atten_t samplingAttenuation = getSamplingAttenuation(atten);
    analogSetPinAttenuation(sourcePin, (adc_attenuation_t)samplingAttenuation);
    if (useCalibratedReading(atten))
        ESP32AdcCalibration_create(sourcePin, samplingAttenuation, &calibration);
#else
    const int sourcePin = hardware_pin(voltageSources[sourceIdx].hardwarePin);
#endif

    uint16_t rawValues[samples] {};
    uint16_t adcValues[samples] {};
    for (uint8_t i = 0; i < samples; ++i)
    {
        const uint16_t raw = analogRead(sourcePin);
        rawValues[i] = raw;

        uint16_t adc = raw;
#if defined(PLATFORM_ESP32)
        if (useCalibratedReading(atten))
        {
            uint32_t calibrated = 0;
            if (ESP32AdcCalibration_rawToMilliVolts(calibration, raw, &calibrated))
                adc = calibrated;
            else
                adc = analogReadMilliVolts(sourcePin);
        }
#endif
        adcValues[i] = adc;
    }

#if defined(PLATFORM_ESP32)
    ESP32AdcCalibration_destroy(calibration);
#endif

    summarizeSamples(rawValues, adcValues, samples, sample);
    sample->saturated = sample->rawMax >= (ADC_MAX_VALUE - ADC_SATURATION_MARGIN);
    const int noReadingThreshold = hardware_int(voltageSources[sourceIdx].noReading);
    sample->hasReading = (noReadingThreshold < 0) || (sample->rawMedian > (uint16_t)noReadingThreshold);
    return true;
}
