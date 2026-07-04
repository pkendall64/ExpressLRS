#include "devAnalogVbat.h"

#if defined(TARGET_RX)

#include "CRSFRouter.h"
#include "logging.h"
#include "median.h"
#include "config.h"
#include <Arduino.h>
#include "ESP32AdcCalibration.h"

// Sample 5x samples over 500ms (unless SlowUpdate)
#define VBAT_SMOOTH_CNT         5
#if defined(DEBUG_VBAT_ADC)
#define VBAT_SAMPLE_INTERVAL    20U // faster updates in debug mode
#else
#define VBAT_SAMPLE_INTERVAL    100U
#endif

#define VBAT_MIN_CRSFRATE 5000      // send VBat telemetry on change but at least every 5000ms

typedef uint16_t voltageAnalogStorage_t;
#if defined(PLATFORM_ESP8266)
constexpr uint8_t VOLTAGE_SOURCE_COUNT = 1;
#else
constexpr uint8_t VOLTAGE_SOURCE_COUNT = 4;
#endif

typedef struct {
    nameType hardwarePin;
    nameType offset;
    nameType scale;
    nameType atten;
    nameType noReading;
} voltageSource_t;

static MedianAvgFilter<voltageAnalogStorage_t, VBAT_SMOOTH_CNT> voltageSmooth[VOLTAGE_SOURCE_COUNT];
static bool voltageSourceConnected[VOLTAGE_SOURCE_COUNT];
static uint8_t vbatUpdateScale;
static bool calibrationActive;

static const voltageSource_t voltageSources[VOLTAGE_SOURCE_COUNT] = {
    {HARDWARE_vbat, HARDWARE_vbat_offset, HARDWARE_vbat_scale, HARDWARE_vbat_atten, HARDWARE_vbat_noreading},
#if defined(PLATFORM_ESP32)
    {HARDWARE_vsrc1, HARDWARE_vsrc1_offset, HARDWARE_vsrc1_scale, HARDWARE_vsrc1_atten, HARDWARE_vsrc1_noreading},
    {HARDWARE_vsrc2, HARDWARE_vsrc2_offset, HARDWARE_vsrc2_scale, HARDWARE_vsrc2_atten, HARDWARE_vsrc2_noreading},
    {HARDWARE_vsrc3, HARDWARE_vsrc3_offset, HARDWARE_vsrc3_scale, HARDWARE_vsrc3_atten, HARDWARE_vsrc3_noreading}
#endif
};
#if defined(PLATFORM_ESP32)
static adc_cali_handle_t voltageAdcCalibration[VOLTAGE_SOURCE_COUNT];
#endif


/**
 * @brief: Enable SlowUpdate mode to reduce the frequency Vbat telemetry is sent
 ***/
void Vbat_enableSlowUpdate(bool enable)
{
    vbatUpdateScale = enable ? 2 : 1;
}

void Vbat_setCalibrationActive(bool active)
{
    calibrationActive = active;
}

static bool sourceIsDefined(uint8_t sourceIdx)
{
    return hardware_pin(voltageSources[sourceIdx].hardwarePin) != UNDEF_PIN;
}

static bool sourceHasReading(uint8_t sourceIdx)
{
    return sourceIsDefined(sourceIdx) && voltageSourceConnected[sourceIdx];
}
#if defined(PLATFORM_ESP32)
static bool useCalibratedReading(uint8_t sourceIdx)
{
    return hardware_int(voltageSources[sourceIdx].atten) > ADC_ATTEN_DB_12;
}

static adc_atten_t getSamplingAttenuation(uint8_t sourceIdx)
{
    int atten = hardware_int(voltageSources[sourceIdx].atten);
    if (useCalibratedReading(sourceIdx))
        atten -= (ADC_ATTEN_DB_12 + 1);
    if (atten == -1)
        atten = ADC_ATTEN_DB_12;
    return (adc_atten_t)atten;
}
#endif

static int32_t adcToMilliVolts(uint8_t sourceIdx, uint32_t adc)
{
    int offset = hardware_int(voltageSources[sourceIdx].offset);
    int scale = hardware_int(voltageSources[sourceIdx].scale);

    // For negative offsets, anything between abs(OFFSET) and 0 is considered 0
    if (offset < 0 && adc <= (uint32_t)(-offset))
    {
        return 0;
    }

    return (((int32_t)adc - offset) * 10000) / scale;
}

static uint32_t readSmoothedAdc(uint8_t sourceIdx)
{
    return voltageSmooth[sourceIdx].calc();
}

static bool initialize()
{
    for (uint8_t sourceIdx = 0; sourceIdx < VOLTAGE_SOURCE_COUNT; ++sourceIdx)
    {
        if (sourceIsDefined(sourceIdx))
            return true;
    }

    return false;
}

static int start()
{
    vbatUpdateScale = 1;
#if defined(PLATFORM_ESP32)
    analogReadResolution(12);

    for (uint8_t sourceIdx = 0; sourceIdx < VOLTAGE_SOURCE_COUNT; ++sourceIdx)
    {
        ESP32AdcCalibration_destroy(voltageAdcCalibration[sourceIdx]);
        voltageAdcCalibration[sourceIdx] = nullptr;

        if (!sourceIsDefined(sourceIdx))
            continue;

        const int sourcePin = hardware_pin(voltageSources[sourceIdx].hardwarePin);
        const adc_atten_t samplingAttenuation = getSamplingAttenuation(sourceIdx);
        analogSetPinAttenuation(sourcePin, (adc_attenuation_t)samplingAttenuation);
        if (useCalibratedReading(sourceIdx) &&
            !ESP32AdcCalibration_create(sourcePin, samplingAttenuation, &voltageAdcCalibration[sourceIdx]))
            ERRLN("ADC calibration unavailable for source %u pin %d", sourceIdx, sourcePin);
    }
#endif

    return VBAT_SAMPLE_INTERVAL;
}

static void reportVbat()
{
    int32_t sourceMilliVolts[VOLTAGE_SOURCE_COUNT] = {0};
    static int32_t lastSourceMilliVolts[VOLTAGE_SOURCE_COUNT] = {0};
    static uint32_t lastTelemSentMs = 0;
    bool triggerPacket = false;

    for (uint8_t sourceIdx = 0; sourceIdx < VOLTAGE_SOURCE_COUNT; ++sourceIdx)
    {
        if (!sourceIsDefined(sourceIdx))
            continue;

        if (sourceHasReading(sourceIdx))
        {
            sourceMilliVolts[sourceIdx] = adcToMilliVolts(sourceIdx, readSmoothedAdc(sourceIdx));

            if (sourceMilliVolts[sourceIdx] != lastSourceMilliVolts[sourceIdx])
            {
                lastSourceMilliVolts[sourceIdx] = sourceMilliVolts[sourceIdx];

                triggerPacket = true;
            }
        }
    }

    uint32_t now = millis();

    // send packet only if min rate timer expired or VBat value has changed
    if (triggerPacket || (now - lastTelemSentMs >= VBAT_MIN_CRSFRATE))
    {
        // send battery packets (0x08) only if no external decive is sending 0x08 packets
        if (!crsfBatterySensorDetected && config.GetSerialProtocol() != PROTOCOL_MAVLINK)
        {
            // CRSF_FRAMETYPE_BATTERY (0x08)
            CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfbatt = { 0 };
            crsfbatt.p.voltage = htobe16((uint16_t)sourceMilliVolts[0] / 100); // 100mV resolution, BigEndian
                                                                                   // No values for current, capacity, or remaining available
            crsfRouter.SetHeaderAndCrc(&crsfbatt.h, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)));
            crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfbatt.h);
        }

        // CRSF_FRAMETYPE_CELLS (0x0E)
        CRSF_MK_FRAME_T(crsf_sensor_cells_t) crsfcells = { 0 };
        size_t cellCount = 0;
        crsfcells.p.source_id = 128 + 0; // Volt sensor ID 0
        for (uint8_t sourceIdx = 0; sourceIdx < VOLTAGE_SOURCE_COUNT; ++sourceIdx)
        {
            if (!sourceIsDefined(sourceIdx))
                continue;

            crsfcells.p.cell[cellCount++] = htobe16((uint16_t)sourceMilliVolts[sourceIdx]);
        }

        if (cellCount > 0)
        {
            size_t payloadLen = sizeof(crsfcells.p.source_id) + (cellCount * sizeof(crsfcells.p.cell[0]));
            crsfRouter.SetHeaderAndCrc(&crsfcells.h, CRSF_FRAMETYPE_CELLS, CRSF_FRAME_SIZE(payloadLen));
            crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfcells.h);
        }

        triggerPacket = false;

        lastTelemSentMs = now;
    }
}

static int timeout()
{
    if (calibrationActive)
        return VBAT_SAMPLE_INTERVAL * vbatUpdateScale;

    bool shouldReport = false;
    for (uint8_t sourceIdx = 0; sourceIdx < VOLTAGE_SOURCE_COUNT; ++sourceIdx)
    {
        if (!sourceIsDefined(sourceIdx))
            continue;

        const int sourcePin = hardware_pin(voltageSources[sourceIdx].hardwarePin);
        const uint32_t raw = analogRead(sourcePin);
        int noReadingThreshold = hardware_int(voltageSources[sourceIdx].noReading);
        voltageSourceConnected[sourceIdx] = (noReadingThreshold < 0) || (raw > (uint32_t)noReadingThreshold);

        uint32_t adc = raw;
#if defined(PLATFORM_ESP32)
        if (useCalibratedReading(sourceIdx))
        {
            if (!ESP32AdcCalibration_rawToMilliVolts(voltageAdcCalibration[sourceIdx], raw, &adc))
                adc = analogReadMilliVolts(sourcePin);
        }
#endif

#if defined(DEBUG_VBAT_ADC)
        DBGLN("$ADC,%u,%u", sourceIdx, adc);
#endif

        if (voltageSourceConnected[sourceIdx] && voltageSmooth[sourceIdx].add(adc) == 0)
            shouldReport = true;
    }

    if (shouldReport && connectionState == connected)
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

#endif
