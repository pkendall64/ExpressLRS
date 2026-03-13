#include "thermal.h"
#include "logging.h"

#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
#include "lm75a.h"
LM75A lm75a;
#if defined(PLATFORM_ESP32_S3)
#include "driver/temperature_sensor.h"
static temperature_sensor_handle_t temp_handle;
#endif

static const uint8_t thermal_threshold_data[] = {
    THERMAL_FAN_DEFAULT_HIGH_THRESHOLD,
    THERMAL_FAN_DEFAULT_LOW_THRESHOLD,
    THERMAL_FAN_ALWAYS_ON_HIGH_THRESHOLD,
    THERMAL_FAN_ALWAYS_ON_LOW_THRESHOLD,
    THERMAL_FAN_OFF_HIGH_THRESHOLD,
    THERMAL_FAN_OFF_LOW_THRESHOLD
};

static Thermal_Status_t thermal_status = THERMAL_STATUS_FAIL;

void Thermal::init()
{
    int status = -1;
    if (OPT_HAS_THERMAL_LM75A)
    {
        status = lm75a.init();
    }
#if defined(PLATFORM_ESP32_S3)
    if (status == -1)
    {
        temperature_sensor_config_t temp_sensor = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        temperature_sensor_install(&temp_sensor, &temp_handle);
        temperature_sensor_enable(temp_handle);
    }
#else
    if (status == -1)
    {
        ERRLN("Thermal failed!");
        return;
    }
#endif
    DBGLN("Thermal OK!");
    temp_value = 0;
    thermal_status = THERMAL_STATUS_NORMAL;
    update_threshold(0);
}

void Thermal::handle()
{
    temp_value = read_temp();
}

uint8_t Thermal::read_temp()
{
    if(thermal_status != THERMAL_STATUS_NORMAL)
    {
        ERRLN("thermal not ready!");
        return 0;
    }
    if (OPT_HAS_THERMAL_LM75A)
    {
        return lm75a.read_lm75a();
    }

#if defined(PLATFORM_ESP32_S3)
    float result = 0;
    temperature_sensor_get_celsius(temp_handle, &result);
    return static_cast<int>(result);
#else
    return 0;
#endif
}

void Thermal::update_threshold(int index)
{
    static int prevIndex = -1;
    if (index == prevIndex)
    {
        return;
    }
    prevIndex = index;
    if(thermal_status != THERMAL_STATUS_NORMAL)
    {
        ERRLN("thermal not ready!");
        return;
    }
    constexpr int size = sizeof(thermal_threshold_data)/sizeof(thermal_threshold_data[0]);
    if(index > size/2)
    {
        ERRLN("thermal index out of range!");
        return;
    }
    if (OPT_HAS_THERMAL_LM75A)
    {
        const uint8_t high = thermal_threshold_data[2*index];
        const uint8_t low = thermal_threshold_data[2*index+1];
        lm75a.update_lm75a_threshold(high, low);
    }
}
#endif
