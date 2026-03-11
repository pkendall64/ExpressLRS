#include "devAnalogVbat.h"

#include "CRSFRouter.h"
#include "logging.h"
#include "median.h"
#include <Arduino.h>

// Sample 5x samples over 500ms (unless SlowUpdate)
#define VBAT_SMOOTH_CNT         5
#if defined(DEBUG_VBAT_ADC)
#define VBAT_SAMPLE_INTERVAL    20U // faster updates in debug mode
#else
#define VBAT_SAMPLE_INTERVAL    100U
#endif

#define VBAT_MIN_CRSFRATE 5000      // send VBat telemetry on change but at least every 5000ms
static uint32_t lastVBatSentMs = 0; // last time VBat was sent
static int32_t lastVBatValue = 0;   // last measured VBat value

typedef uint16_t vbatAnalogStorage_t;
static MedianAvgFilter<vbatAnalogStorage_t, VBAT_SMOOTH_CNT> vbatSmooth;
static uint8_t vbatUpdateScale;

#if defined(PLATFORM_ESP32)
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_cali_handle_t vbat_cali_handle = nullptr;
static bool vbatAdcUnitCharacteristics = false;
#endif

/**
 * @brief: Enable SlowUpdate mode to reduce the frequency Vbat telemetry is sent
 ***/
void Vbat_enableSlowUpdate(bool enable)
{
    vbatUpdateScale = enable ? 2 : 1;
}

static bool initialize()
{
    return GPIO_ANALOG_VBAT != UNDEF_PIN;
}

static int start()
{
    vbatUpdateScale = 1;
#if defined(PLATFORM_ESP32)
    analogReadResolution(12);

    int atten = hardware_int(HARDWARE_vbat_atten);
    if (atten != -1)
    {
        // if the configured value is higher than the max item (11dB, it indicates to use cal_characterize)
        const bool useCal = atten > ADC_ATTEN_DB_12;
        analogSetPinAttenuation(GPIO_ANALOG_VBAT, (adc_attenuation_t)atten);
        if (useCal)
        {
            atten -= (ADC_ATTEN_DB_12 + 1);

            int8_t channel = digitalPinToAnalogChannel(GPIO_ANALOG_VBAT);
            adc_unit_t unit = (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)) ? ADC_UNIT_2 : ADC_UNIT_1;
#if CONFIG_IDF_TARGET_ESP32
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = (adc_atten_t)atten,
                .bitwidth = ADC_BITWIDTH_12,
            };
            vbatAdcUnitCharacteristics = adc_cali_create_scheme_line_fitting(&cali_config, &vbat_cali_handle) == ESP_OK;
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S2
            const adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = (adc_channel_t)channel, // Required for curve fitting
                .atten = (adc_atten_t)atten,
                .bitwidth = ADC_BITWIDTH_12,
            };
            vbatAdcUnitCharacteristics = adc_cali_create_scheme_curve_fitting(&cali_config, &vbat_cali_handle) == ESP_OK;
#endif
        }
    }
#endif

    return VBAT_SAMPLE_INTERVAL;
}

static void reportVbat()
{
    int32_t adc = vbatSmooth.calc();

#if defined(PLATFORM_ESP32) && !defined(DEBUG_VBAT_ADC)
    if (vbatAdcUnitCharacteristics)
    {
        int temp;
        adc_cali_raw_to_voltage(vbat_cali_handle, adc, &temp);
        adc = temp;
    }
#endif

    int32_t vbat_mV;
    // For negative offsets, anything between abs(OFFSET) and 0 is considered 0
    if (ANALOG_VBAT_OFFSET < 0 && adc <= -ANALOG_VBAT_OFFSET)
    {
        vbat_mV = 0;
    }
    else
    {
        vbat_mV = ((adc - ANALOG_VBAT_OFFSET) * 10000) / ANALOG_VBAT_SCALE;
    }

    uint32_t now = millis();

    // send packet only if min rate timer expired or VBat value has changed
    if ((now - lastVBatSentMs >= VBAT_MIN_CRSFRATE) || (vbat_mV != lastVBatValue))
    {
        // send battery packets (0x08) only if no external decive is sending 0x08 packets
        if (!crsfBatterySensorDetected)
        {
            // CRSF_FRAMETYPE_BATTERY (0x08)
            CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfbatt{};
            crsfbatt.p.voltage = htobe16((uint16_t)vbat_mV / 100);  // VBat, 100mV resolution, BigEndian
                                                                    // No values for current, capacity, or remaining available
            crsfRouter.SetHeaderAndCrc(&crsfbatt.h, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)));
            crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfbatt.h);
        }

        // CRSF_FRAMETYPE_CELLS (0x0E)
        CRSF_MK_FRAME_T(crsf_sensor_cells_t) crsfcells{};
        crsfcells.p.source_id = 128 + 0;                        // Volt sensor ID 0
        crsfcells.p.cell[0] = htobe16((uint16_t)(vbat_mV));     // VBat, 1mV resolution, BigEndian
        constexpr size_t payloadLen = sizeof(crsfcells.p.source_id) + sizeof(crsfcells.p.cell[0]);
        crsfRouter.SetHeaderAndCrc(&crsfcells.h, CRSF_FRAMETYPE_CELLS, CRSF_FRAME_SIZE(payloadLen));
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfcells.h);

        lastVBatSentMs = now;
    }

    lastVBatValue = vbat_mV;
}

static int timeout()
{
    uint32_t adc = analogRead(GPIO_ANALOG_VBAT);

#if defined(PLATFORM_ESP32) && defined(DEBUG_VBAT_ADC)
    // When doing DEBUG_VBAT_ADC, every value is adjusted (for logging)
    // in normal mode only the final value is adjusted to save CPU cycles
    if (vbatAdcUnitCharacteristics)
    {
        int temp = adc;
        adc_cali_raw_to_voltage(vbat_cali_handle, adc, &temp);
        adc = temp;
    }
    DBGLN("$ADC,%u", adc);
#endif

    unsigned int idx = vbatSmooth.add(adc);
    if (idx == 0 && connectionState == connected)
        reportVbat();

    return VBAT_SAMPLE_INTERVAL * vbatUpdateScale;
}

device_t AnalogVbat_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
    .subscribe = EVENT_NONE
};
