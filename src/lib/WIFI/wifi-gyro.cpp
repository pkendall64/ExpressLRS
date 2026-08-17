#include "targets.h"

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <cstring>

#include "common.h"
#include "config.h"
#include "devGyro.h"
#include "gyro.h"
extern void gyroQuickModelSetup(int wingType, int tailType);
static const char *GyroChannelFunctionName(uint8_t function)
{
    switch ((gyro_output_channel_function_t)function)
    {
    case FN_AILERON:
        return "Aileron";
    case FN_ELEVATOR:
        return "Elevator";
    case FN_RUDDER:
        return "Rudder";
    case FN_ELEVON:
        return "Elevon";
    case FN_ELEVON_R:
        return "Elevon R";
    case FN_VTAIL:
        return "V-Tail";
    case FN_VTAIL_R:
        return "V-Tail R";
    case FN_GYRO_MODE:
        return "Gyro Mode";
    case FN_GYRO_GAIN:
        return "Gyro Gain";
    case FN_NONE:
    default:
        return "None";
    }
}

static uint16_t GetChannelInputUs(uint8_t channel)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(channel);
    const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
    return crsfVal == CRSF_CHANNEL_VALUE_UNSET ? 0 : CRSF_to_US(crsfVal);
}

static void AppendGyroModeMap(JsonObject json)
{
    const uint8_t positions = gyroConfig->GetGyroModePositions();
    json["mode_switch_positions"] = positions;
    const auto modeMap = json["mode_map"].to<JsonArray>();
    for (uint8_t position = 0; position < positions; ++position)
    {
        modeMap.add(gyroConfig->GetGyroMode(position));
    }
}

static void AppendGyroChannelFunctions(JsonObject json, bool sparse)
{
    const auto channelFunctions = json["channel_functions"].to<JsonArray>();
    for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
    {
        const rx_config_gyro_channel_t *channelConfig = gyroConfig->GetGyroChannel(channel);
        if (sparse && channelConfig->val.output_mode == FN_NONE)
        {
            continue;
        }
        const rx_config_pwm_limits_t *limits = gyroConfig->GetPwmChannelLimits(channel);
        const auto channelFunction = channelFunctions.add<JsonObject>();
        channelFunction["channel"] = channel + 1;
        channelFunction["functionId"] = channelConfig->val.output_mode;
        channelFunction["function"] = GyroChannelFunctionName(channelConfig->val.output_mode);
        channelFunction["master"] = (bool)channelConfig->val.master;
        channelFunction["invert"] = (bool)channelConfig->val.inverted;
        channelFunction["min"] = limits->val.min;
        channelFunction["mid"] = limits->val.mid;
        channelFunction["max"] = limits->val.max;
    }
}

static void AppendGyroModes(JsonObject json)
{
    const auto gyroModes = json["gyro_modes"].to<JsonArray>();
    for (uint8_t mode = GYRO_MODE_RATE; mode <= GYRO_MODE_LAST_ACTIVE; ++mode)
    {
        const rx_config_gyro_fmode_t *gyroMode = gyroConfig->GetGyroFMode((gyro_mode_t)mode);
        const auto modeJson = gyroModes.add<JsonObject>();
        modeJson["modeId"] = mode;
        modeJson["useRate"] = (bool)gyroMode->val.useRate;
        modeJson["stickPriority"] = gyroMode->val.stickPri;
        modeJson["gainFactor"] = gyroMode->val.gainFactor;
        modeJson["pitchLimit"] = gyroMode->val.maxAnglePitch;
        modeJson["rollLimit"] = gyroMode->val.maxAngleRoll;
        modeJson["trimPitch"] = gyro_trim_decode(gyroMode->val.trimPitch);
        modeJson["trimRoll"] = gyro_trim_decode(gyroMode->val.trimRoll);
        modeJson["gainPitch"] = gyroMode->val.gainPitch;
        modeJson["gainRoll"] = gyroMode->val.gainRoll;
        modeJson["gainYaw"] = gyroMode->val.gainYaw;
    }
}

static uint8_t GyroPidAxisCount(gyro_pidgroup_t group)
{
    return group == GYRO_PID_GROUP_MADGWICK ? 1 : GYRO_N_AXES;
}

static void AppendGyroPids(JsonObject json)
{
    const auto gyroPids = json["gyro_pids"].to<JsonArray>();
    for (uint8_t group = GYRO_PID_GROUP_RATE; group <= GYRO_PID_GROUP_LAST_ACTIVE; ++group)
    {
        const auto pidGroup = (gyro_pidgroup_t)group;
        for (uint8_t axis = GYRO_AXIS_ROLL; axis < GyroPidAxisCount(pidGroup); ++axis)
        {
            const rx_config_gyro_PID_t *pid = gyroConfig->GetGyroPID(pidGroup, (gyro_axis_t)axis);
            const auto pidJson = gyroPids.add<JsonObject>();
            pidJson["groupId"] = group;
            pidJson["axisId"] = axis;
            pidJson["p"] = pid->val.p;
            pidJson["i"] = pid->val.i;
            pidJson["d"] = pid->val.d;
        }
    }
}
enum GyroCalibrationStep
{
    GYRO_CALIBRATION_IDLE,
    GYRO_CALIBRATION_ORIENTATION_HORIZONTAL,
    GYRO_CALIBRATION_STICKS_CENTERED,
    GYRO_CALIBRATION_STICKS_RANGE,
};

static GyroCalibrationStep gyroCalibrationStep = GYRO_CALIBRATION_IDLE;


static void AppendGyroRuntimeChannels(JsonObject json)
{
    const auto channels = json["channels"].to<JsonArray>();
    for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
    {
        channels.add(GetChannelInputUs(channel));
    }
}

static void AppendGyroStickCalibrationLimits(JsonObject json)
{
    const auto limits = json["stick_limits"].to<JsonArray>();
    for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
    {
        const rx_config_pwm_limits_t *limit = gyro.getStickCalibrationLimits(channel);
        const auto output = limits.add<JsonObject>();
        output["min"] = limit->val.min;
        output["mid"] = limit->val.mid;
        output["max"] = limit->val.max;
    }
}
static AsyncWebSocket gyroRuntimeSocket("/gyro-runtime");
static constexpr uint32_t GYRO_RUNTIME_INTERVAL_MS = 40;
static uint32_t gyroRuntimeLastSentMs = 0;

static void AppendGyroState(JsonObject json)
{
    json["status"] = gyro.getStatus();
    json["status_bits"] = gyro.getStatusBits();
    json["next_action"] = gyro.getNextAction();
    json["error"] = gyro.lastErrorText;
    json["read-errors"] = gyro.ahrs->read_errors;
    json["int-errors"] = gyro.ahrs->int_errors;
    json["link"] = connectionState;
    json["rate"] = ExpressLRS_currAirRate_Modparams->enum_rate;
    json["angle_r"] = radToDeg(gyro.ahrs->angle_rpy[0]);
    json["angle_p"] = radToDeg(gyro.ahrs->angle_rpy[1]);
    json["angle_y"] = radToDeg(gyro.ahrs->angle_rpy[2]);
    AppendGyroRuntimeChannels(json);
    if (gyroCalibrationStep >= GYRO_CALIBRATION_STICKS_CENTERED)
    {
        AppendGyroStickCalibrationLimits(json);
    }
    json["mode"] = gyro.getMode();
    json["mode_position"] = gyro.getModePosition();
}

static bool GyroRuntimeSocketIsReady()
{
    for (const auto &client : gyroRuntimeSocket.getClients())
    {
        if (client.status() == WS_CONNECTED && client.queueLen() != 0)
        {
            return false;
        }
    }
    return gyroRuntimeSocket.count() != 0;
}

static bool JsonIntegerIsInRange(JsonVariantConst value, int min, int max)
{
    if (value.isNull())
    {
        return true;
    }
    return value.is<int>() && value.as<int>() >= min && value.as<int>() <= max;
}

static bool GyroChannelFunctionUsesLimits(uint8_t function)
{
    return function != FN_GYRO_MODE && function != FN_GYRO_GAIN;
}

static bool GyroChannelFunctionConfigurationIsValid(const uint8_t *functions, const bool *masters)
{
    bool controls[FN_GYRO_GAIN + 1] = {};
    bool masterFunctions[FN_GYRO_GAIN + 1] = {};
    for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
    {
        const uint8_t function = functions[channel];
        if (function > FN_GYRO_GAIN)
        {
            return false;
        }
        if (function == FN_GYRO_MODE || function == FN_GYRO_GAIN)
        {
            if (controls[function])
            {
                return false;
            }
            controls[function] = true;
        }
        if (GyroChannelFunctionUsesLimits(function) && masters[channel])
        {
            if (masterFunctions[function])
            {
                return false;
            }
            masterFunctions[function] = true;
        }
    }
    return true;
}

static void AppendGyroSummary(JsonObject json)
{
    json["version"] = GYRO_CODE_VERSION;
    json["config_version"] = gyroConfig->GetGyroConfigVersion();
    json["imu"] = gyro.ahrs->getImuDriver()->GetMPUName();
    json["enabled"] = gyroConfig->GetGyroEnabled();
}

static void GetGyroState(AsyncWebServerRequest *request)
{
    auto *response = new AsyncJsonResponse();
    AppendGyroState(response->getRoot());
    response->setLength();
    request->send(response);
}

static void GetGyroConfig(AsyncWebServerRequest *request)
{
    auto *response = new AsyncJsonResponse();
    const auto json = response->getRoot();
    const bool exporting = request->hasParam("export");
    AppendGyroSummary(json);
    AppendGyroModeMap(json);
    AppendGyroChannelFunctions(json, exporting);
    AppendGyroModes(json);
    AppendGyroPids(json);
    if (exporting)
    {
        response->addHeader("Content-Disposition", "attachment; filename=\"gyro-config.json\"");
    }
    response->setLength();
    request->send(response);
}

static void SetGyroConfig(AsyncWebServerRequest *request, JsonVariant &json)
{
    if (!json.is<JsonObject>())
    {
        request->send(400, "text/plain", "Invalid gyro configuration");
        return;
    }
    const bool importing = request->hasParam("import");
    if ((!json["quick_setup"].isNull() && !json["quick_setup"].is<JsonObject>())
        || (!json["mode_map"].isNull() && !json["mode_map"].is<JsonArray>())
        || (!json["channel_functions"].isNull() && !json["channel_functions"].is<JsonArray>())
        || (!json["gyro_modes"].isNull() && !json["gyro_modes"].is<JsonArray>())
        || (!json["gyro_pids"].isNull() && !json["gyro_pids"].is<JsonArray>()))
    {
        request->send(400, "text/plain", "Invalid gyro configuration");
        return;
    }

    if (importing
        && (json["enabled"].isNull()
            || json["mode_switch_positions"].isNull()
            || json["mode_map"].as<JsonArray>().isNull()
            || json["channel_functions"].as<JsonArray>().isNull()
            || json["gyro_modes"].as<JsonArray>().isNull()
            || json["gyro_pids"].as<JsonArray>().isNull()))
    {
        request->send(400, "text/plain", "Incomplete gyro configuration");
        return;
    }

    const JsonObject quickSetup = json["quick_setup"].as<JsonObject>();
    if (!quickSetup.isNull())
    {
        const JsonVariantConst wing = quickSetup["wing"];
        const JsonVariantConst tail = quickSetup["tail"];
        if (importing
            || !json["enabled"].isNull()
            || !json["mode_switch_positions"].isNull()
            || !json["mode_map"].isNull()
            || !json["channel_functions"].isNull()
            || !json["gyro_modes"].isNull()
            || !json["gyro_pids"].isNull()
            || !JsonIntegerIsInRange(wing, 0, 3)
            || !JsonIntegerIsInRange(tail, 0, 4)
            || wing.isNull()
            || tail.isNull())
        {
            request->send(400, "text/plain", "Invalid quick setup");
            return;
        }
        gyroQuickModelSetup(wing.as<int>(), tail.as<int>());
        request->send(200, "text/plain", "Gyro configuration updated");
        return;
    }

    struct PendingGyroConfig
    {
        bool channelFunctionsChanged;
        uint8_t functions[GYRO_MAX_CHANNELS];
        bool inverted[GYRO_MAX_CHANNELS];
        bool masters[GYRO_MAX_CHANNELS];
        uint16_t minValues[GYRO_MAX_CHANNELS];
        uint16_t midValues[GYRO_MAX_CHANNELS];
        uint16_t maxValues[GYRO_MAX_CHANNELS];

        bool enabledChanged;
        bool enabled;

        bool modeSwitchPositionsChanged;
        uint8_t modeSwitchPositions;
        bool modeMapChanged;
        gyro_mode_t modeMap[6];

        bool gyroModesChanged;
        bool gyroModeChanged[GYRO_MODE_MAX];
        rx_config_gyro_fmode_t gyroModes[GYRO_MODE_MAX];

        bool gyroPidsChanged;
        bool gyroPidChanged[GYRO_PID_GROUP_MAX][GYRO_N_AXES];
        rx_config_gyro_PID_t gyroPids[GYRO_PID_GROUP_MAX][GYRO_N_AXES];
    };
    PendingGyroConfig pending = {};
    pending.modeSwitchPositions = gyroConfig->GetGyroModePositions();

    const JsonArray channelFunctions = json["channel_functions"].as<JsonArray>();
    if (!channelFunctions.isNull())
    {
        bool channelSeen[GYRO_MAX_CHANNELS] = {};
        for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
        {
            pending.functions[channel] = FN_NONE;
            const rx_config_pwm_limits_t *limits = gyroConfig->GetPwmChannelLimits(channel);
            pending.minValues[channel] = limits->val.min;
            pending.midValues[channel] = limits->val.mid;
            pending.maxValues[channel] = limits->val.max;
        }

        for (JsonObject channelFunction : channelFunctions)
        {
            const JsonVariantConst channelValue = channelFunction["channel"];
            if (!JsonIntegerIsInRange(channelValue, 1, GYRO_MAX_CHANNELS) || channelValue.isNull())
            {
                request->send(400, "text/plain", "Invalid channel");
                return;
            }
            const uint8_t index = channelValue.as<int>() - 1;
            if (channelSeen[index])
            {
                request->send(400, "text/plain", "Duplicate channel");
                return;
            }
            channelSeen[index] = true;

            JsonVariantConst functionValue = channelFunction["functionId"];
            if (functionValue.isNull())
            {
                functionValue = channelFunction["function_id"];
            }
            if (!functionValue.isNull() && !JsonIntegerIsInRange(functionValue, FN_NONE, FN_GYRO_GAIN))
            {
                request->send(400, "text/plain", "Invalid channel function");
                return;
            }
            const uint8_t functionId = functionValue.isNull() ? FN_NONE : functionValue.as<int>();

            JsonVariantConst invertedValue = channelFunction["invert"];
            if (invertedValue.isNull())
            {
                invertedValue = channelFunction["inverted"];
            }
            const JsonVariantConst masterValue = channelFunction["master"];
            if ((!invertedValue.isNull() && !invertedValue.is<bool>())
                || (!masterValue.isNull() && !masterValue.is<bool>()))
            {
                request->send(400, "text/plain", "Invalid channel function flags");
                return;
            }

            const JsonVariantConst minValue = channelFunction["min"];
            const JsonVariantConst midValue = channelFunction["mid"];
            const JsonVariantConst maxValue = channelFunction["max"];
            if (importing
                && (functionValue.isNull()
                    || invertedValue.isNull()
                    || masterValue.isNull()
                    || minValue.isNull()
                    || midValue.isNull()
                    || maxValue.isNull()))
            {
                request->send(400, "text/plain", "Incomplete channel function");
                return;
            }
            if (!JsonIntegerIsInRange(minValue, GYRO_US_MIN, GYRO_US_MID + 1)
                || !JsonIntegerIsInRange(midValue, 1000, 2000)
                || !JsonIntegerIsInRange(maxValue, GYRO_US_MID + 1, GYRO_US_MAX))
            {
                request->send(400, "text/plain", "Invalid channel limits");
                return;
            }

            pending.functions[index] = functionId;
            pending.inverted[index] = invertedValue | false;
            pending.minValues[index] = minValue | pending.minValues[index];
            pending.midValues[index] = midValue | pending.midValues[index];
            pending.maxValues[index] = maxValue | pending.maxValues[index];
            pending.masters[index] = masterValue | false;
            if (!GyroChannelFunctionUsesLimits(functionId))
            {
                pending.masters[index] = false;
            }
            if (!(pending.minValues[index] <= pending.midValues[index]
                && pending.midValues[index] <= pending.maxValues[index]))
            {
                request->send(400, "text/plain", "Invalid channel limits");
                return;
            }
        }

        if (!GyroChannelFunctionConfigurationIsValid(pending.functions, pending.masters))
        {
            request->send(400, "text/plain", "Invalid channel function combination");
            return;
        }
        pending.channelFunctionsChanged = true;
    }

    const JsonVariantConst enabled = json["enabled"];
    if (!enabled.isNull())
    {
        if (!enabled.is<bool>())
        {
            request->send(400, "text/plain", "Invalid gyro enabled value");
            return;
        }
        pending.enabled = enabled.as<bool>();
        pending.enabledChanged = true;
    }


    const JsonVariantConst modeSwitchPositions = json["mode_switch_positions"];
    if (!modeSwitchPositions.isNull())
    {
        if (!JsonIntegerIsInRange(modeSwitchPositions, 2, 6))
        {
            request->send(400, "text/plain", "Invalid mode switch position count");
            return;
        }
        pending.modeSwitchPositions = modeSwitchPositions.as<int>();
        pending.modeSwitchPositionsChanged = true;
    }

    const JsonArray modeMap = json["mode_map"].as<JsonArray>();
    if (!modeMap.isNull())
    {
        if (modeMap.size() < pending.modeSwitchPositions)
        {
            request->send(400, "text/plain", "Incomplete mode map");
            return;
        }
        for (uint8_t index = 0; index < pending.modeSwitchPositions; ++index)
        {
            const JsonVariantConst modeValue = modeMap[index];
            if (!JsonIntegerIsInRange(modeValue, GYRO_MODE_OFF, GYRO_MODE_LAST_ACTIVE) || modeValue.isNull())
            {
                request->send(400, "text/plain", "Invalid gyro mode");
                return;
            }
            pending.modeMap[index] = (gyro_mode_t)modeValue.as<int>();
        }
        pending.modeMapChanged = true;
    }

    const JsonArray gyroModes = json["gyro_modes"].as<JsonArray>();
    if (!gyroModes.isNull())
    {
        bool modeSeen[GYRO_MODE_MAX] = {};
        for (JsonObject gyroMode : gyroModes)
        {
            const JsonVariantConst modeIdValue = gyroMode["modeId"];
            if (!JsonIntegerIsInRange(modeIdValue, GYRO_MODE_RATE, GYRO_MODE_LAST_ACTIVE) || modeIdValue.isNull())
            {
                request->send(400, "text/plain", "Invalid gyro mode");
                return;
            }
            const auto mode = (gyro_mode_t)modeIdValue.as<int>();
            if (modeSeen[mode])
            {
                request->send(400, "text/plain", "Duplicate gyro mode");
                return;
            }
            modeSeen[mode] = true;

            const bool useRateVisible = gyroIsVisible(mode, GYRO_UI_USE_RATE);
            const bool stickPriorityVisible = gyroIsVisible(mode, GYRO_UI_STICK_PRIORITY);
            const bool angleLimitsVisible = gyroIsVisible(mode, GYRO_UI_MAX_ANGLE);
            const bool trimsVisible = gyroIsVisible(mode, GYRO_UI_TRIMS);
            const JsonVariantConst useRate = gyroMode["useRate"];
            if (importing
                && (useRate.isNull()
                    || gyroMode["stickPriority"].isNull()
                    || gyroMode["gainFactor"].isNull()
                    || gyroMode["pitchLimit"].isNull()
                    || gyroMode["rollLimit"].isNull()
                    || gyroMode["trimPitch"].isNull()
                    || gyroMode["trimRoll"].isNull()
                    || gyroMode["gainPitch"].isNull()
                    || gyroMode["gainRoll"].isNull()
                    || gyroMode["gainYaw"].isNull()))
            {
                request->send(400, "text/plain", "Incomplete gyro mode");
                return;
            }
            if (useRateVisible && !useRate.isNull() && !useRate.is<bool>())
            {
                request->send(400, "text/plain", "Invalid Use Rate value");
                return;
            }
            if (!JsonIntegerIsInRange(gyroMode["gainFactor"], GYRO_GAIN_FACTOR_0_5X, GYRO_GAIN_FACTOR_2X))
            {
                request->send(400, "text/plain", "Invalid gyro gain factor");
                return;
            }
            if (stickPriorityVisible
                && !JsonIntegerIsInRange(gyroMode["stickPriority"], STICK_PRIORITY_100, STICK_PRIORITY_25))
            {
                request->send(400, "text/plain", "Invalid stick priority");
                return;
            }
            if (angleLimitsVisible
                && (!JsonIntegerIsInRange(gyroMode["pitchLimit"], 10, 50)
                    || !JsonIntegerIsInRange(gyroMode["rollLimit"], 30, 90)))
            {
                request->send(400, "text/plain", "Invalid gyro angle limit");
                return;
            }
            if (trimsVisible
                && (!JsonIntegerIsInRange(gyroMode["trimPitch"], -30, 30)
                    || !JsonIntegerIsInRange(gyroMode["trimRoll"], -30, 30)))
            {
                request->send(400, "text/plain", "Invalid gyro trim");
                return;
            }
            if (!JsonIntegerIsInRange(gyroMode["gainPitch"], 0, 250)
                || !JsonIntegerIsInRange(gyroMode["gainRoll"], 0, 250)
                || !JsonIntegerIsInRange(gyroMode["gainYaw"], 0, 250))
            {
                request->send(400, "text/plain", "Invalid gyro gain");
                return;
            }

            rx_config_gyro_fmode_t config = {};
            config.raw = gyroConfig->GetGyroFMode(mode)->raw;
            if (useRateVisible)
            {
                config.val.useRate = useRate | (bool)config.val.useRate;
            }
            if (stickPriorityVisible)
            {
                config.val.stickPri = gyroMode["stickPriority"] | config.val.stickPri;
            }
            config.val.gainFactor = gyroMode["gainFactor"] | config.val.gainFactor;
            if (angleLimitsVisible)
            {
                config.val.maxAnglePitch = gyroMode["pitchLimit"] | config.val.maxAnglePitch;
                config.val.maxAngleRoll = gyroMode["rollLimit"] | config.val.maxAngleRoll;
            }
            if (trimsVisible)
            {
                config.val.trimPitch = gyro_trim_encode((int8_t)(gyroMode["trimPitch"] | gyro_trim_decode(config.val.trimPitch)));
                config.val.trimRoll = gyro_trim_encode((int8_t)(gyroMode["trimRoll"] | gyro_trim_decode(config.val.trimRoll)));
            }
            config.val.gainPitch = gyroMode["gainPitch"] | config.val.gainPitch;
            config.val.gainRoll = gyroMode["gainRoll"] | config.val.gainRoll;
            config.val.gainYaw = gyroMode["gainYaw"] | config.val.gainYaw;
            pending.gyroModes[mode] = config;
            pending.gyroModeChanged[mode] = true;
            pending.gyroModesChanged = true;
        }
        if (importing)
        {
            for (uint8_t mode = GYRO_MODE_RATE; mode <= GYRO_MODE_LAST_ACTIVE; ++mode)
            {
                if (!modeSeen[mode])
                {
                    request->send(400, "text/plain", "Incomplete gyro modes");
                    return;
                }
            }
        }
    }

    const JsonArray gyroPids = json["gyro_pids"].as<JsonArray>();
    if (!gyroPids.isNull())
    {
        bool pidSeen[GYRO_PID_GROUP_MAX][GYRO_N_AXES] = {};
        for (JsonObject gyroPid : gyroPids)
        {
            const JsonVariantConst groupValue = gyroPid["groupId"];
            if (!JsonIntegerIsInRange(groupValue, GYRO_PID_GROUP_RATE, GYRO_PID_GROUP_LAST_ACTIVE)
                || groupValue.isNull())
            {
                request->send(400, "text/plain", "Invalid PID group");
                return;
            }
            const auto group = (gyro_pidgroup_t)groupValue.as<int>();

            const JsonVariantConst axisValue = gyroPid["axisId"];
            if (!JsonIntegerIsInRange(axisValue, GYRO_AXIS_ROLL, GyroPidAxisCount(group) - 1)
                || axisValue.isNull())
            {
                request->send(400, "text/plain", "Invalid PID axis");
                return;
            }
            const auto axis = (gyro_axis_t)axisValue.as<int>();
            if (pidSeen[group][axis])
            {
                request->send(400, "text/plain", "Duplicate PID");
                return;
            }
            pidSeen[group][axis] = true;

            const JsonVariantConst pValue = gyroPid["p"];
            const JsonVariantConst iValue = gyroPid["i"];
            const JsonVariantConst dValue = gyroPid["d"];
            if (importing && (pValue.isNull() || iValue.isNull() || dValue.isNull()))
            {
                request->send(400, "text/plain", "Incomplete PID");
                return;
            }
            const int pMin = group == GYRO_PID_GROUP_MADGWICK ? 10 : 0;
            const int pMax = group == GYRO_PID_GROUP_MADGWICK ? 60 : 100;
            if (!JsonIntegerIsInRange(pValue, pMin, pMax)
                || !JsonIntegerIsInRange(iValue, 0, 100)
                || !JsonIntegerIsInRange(dValue, 0, 100))
            {
                request->send(400, "text/plain", "Invalid PID value");
                return;
            }

            rx_config_gyro_PID_t config = {};
            config.raw = gyroConfig->GetGyroPID(group, axis)->raw;
            config.val.p = pValue | config.val.p;
            config.val.i = iValue | config.val.i;
            config.val.d = dValue | config.val.d;
            pending.gyroPids[group][axis] = config;
            pending.gyroPidChanged[group][axis] = true;
            pending.gyroPidsChanged = true;
        }

        if (importing)
        {
            for (uint8_t group = GYRO_PID_GROUP_RATE; group <= GYRO_PID_GROUP_LAST_ACTIVE; ++group)
            {
                const auto pidGroup = (gyro_pidgroup_t)group;
                for (uint8_t axis = GYRO_AXIS_ROLL; axis < GyroPidAxisCount(pidGroup); ++axis)
                {
                    if (!pidSeen[group][axis])
                    {
                        request->send(400, "text/plain", "Incomplete PIDs");
                        return;
                    }
                }
            }
        }
    }

    const bool changed = pending.channelFunctionsChanged
        || pending.enabledChanged
        || pending.modeSwitchPositionsChanged
        || pending.modeMapChanged
        || pending.gyroModesChanged
        || pending.gyroPidsChanged;
    if (!changed)
    {
        request->send(400, "text/plain", "Missing gyro config changes");
        return;
    }

    // Every request field has been validated. Apply the staged values only
    // after validation completes so rejected requests cannot modify live state.
    if (pending.channelFunctionsChanged)
    {
        for (uint8_t channel = 0; channel < GYRO_MAX_CHANNELS; ++channel)
        {
            gyroConfig->SetGyroChannel(channel, pending.functions[channel], pending.masters[channel], pending.inverted[channel]);
            gyroConfig->SetPwmChannelLimits(channel, pending.minValues[channel], pending.maxValues[channel], pending.midValues[channel]);
        }
    }
    if (pending.enabledChanged)
    {
        gyroConfig->SetGyroEnabled(pending.enabled);
    }
    if (pending.modeSwitchPositionsChanged)
    {
        gyroConfig->SetGyroModePositions(pending.modeSwitchPositions);
    }
    if (pending.modeMapChanged)
    {
        for (uint8_t index = 0; index < pending.modeSwitchPositions; ++index)
        {
            gyroConfig->SetGyroModePos(index, pending.modeMap[index]);
        }
    }
    if (pending.gyroModesChanged)
    {
        for (uint8_t mode = GYRO_MODE_RATE; mode <= GYRO_MODE_LAST_ACTIVE; ++mode)
        {
            if (pending.gyroModeChanged[mode])
            {
                gyroConfig->SetGyroFModeRaw((gyro_mode_t)mode, pending.gyroModes[mode].raw);
            }
        }
    }
    if (pending.gyroPidsChanged)
    {
        for (uint8_t group = GYRO_PID_GROUP_RATE; group <= GYRO_PID_GROUP_LAST_ACTIVE; ++group)
        {
            const auto pidGroup = (gyro_pidgroup_t)group;
            for (uint8_t axis = GYRO_AXIS_ROLL; axis < GyroPidAxisCount(pidGroup); ++axis)
            {
                if (pending.gyroPidChanged[group][axis])
                {
                    const rx_config_gyro_PID_t &pid = pending.gyroPids[group][axis];
                    gyroConfig->SetGyroPIDRate(pidGroup, (gyro_axis_t)axis, GYRO_RATE_VARIABLE_P, pid.val.p);
                    gyroConfig->SetGyroPIDRate(pidGroup, (gyro_axis_t)axis, GYRO_RATE_VARIABLE_I, pid.val.i);
                    gyroConfig->SetGyroPIDRate(pidGroup, (gyro_axis_t)axis, GYRO_RATE_VARIABLE_D, pid.val.d);
                }
            }
        }
        gyroConfig->ValidateMadwick();
    }

    gyroConfig->Commit();
    gyro.reloadConfig();
    request->send(200, "text/plain", "Gyro configuration updated");
}


static void SetGyroCalibration(AsyncWebServerRequest *request, JsonVariant &json)
{
    const char *action = json["action"].as<const char *>();
    if (action == nullptr)
    {
        request->send(400, "text/plain", "Missing gyro calibration action");
        return;
    }

    if (strcmp(action, "orientation-horizontal") == 0 && gyroCalibrationStep == GYRO_CALIBRATION_IDLE)
    {
        gyro.pause();
        gyro.ahrs->OrientationHorizontalExecute();
        gyroCalibrationStep = GYRO_CALIBRATION_ORIENTATION_HORIZONTAL;
    }
    else if (strcmp(action, "orientation-vertical") == 0 && gyroCalibrationStep == GYRO_CALIBRATION_ORIENTATION_HORIZONTAL)
    {
        gyro.ahrs->OrientationVerticalExecute();
        gyroConfig->Commit();
        gyro.reload();
        gyroCalibrationStep = GYRO_CALIBRATION_IDLE;
    }
    else if (strcmp(action, "level") == 0 && gyroCalibrationStep == GYRO_CALIBRATION_IDLE)
    {
        gyro.pause();
        gyro.calibrate();
        gyroConfig->Commit();
        gyro.reload();
    }
    else if (strcmp(action, "sticks-center") == 0 && gyroCalibrationStep == GYRO_CALIBRATION_IDLE)
    {
        gyro.StickCenterCalibration();
        gyroCalibrationStep = GYRO_CALIBRATION_STICKS_CENTERED;
    }
    else if (strcmp(action, "sticks-range-start") == 0
             && (gyroCalibrationStep == GYRO_CALIBRATION_IDLE || gyroCalibrationStep == GYRO_CALIBRATION_STICKS_CENTERED))
    {
        if (gyroCalibrationStep == GYRO_CALIBRATION_IDLE)
        {
            gyro.StickCenterCalibration();
        }
        else if (!gyro.isStickCenterCalibrationComplete())
        {
            request->send(409, "text/plain", "Waiting for centered-stick samples");
            return;
        }
        gyro.StickLimitCalibration(false);
        gyroCalibrationStep = GYRO_CALIBRATION_STICKS_RANGE;
    }
    else if (strcmp(action, "sticks-range-finish") == 0 && gyroCalibrationStep == GYRO_CALIBRATION_STICKS_RANGE)
    {
        gyro.StickLimitCalibration(true);
        gyro.reload();
        gyroCalibrationStep = GYRO_CALIBRATION_IDLE;
    }
    else if (strcmp(action, "cancel") == 0)
    {
        gyro.reload();
        gyroCalibrationStep = GYRO_CALIBRATION_IDLE;
    }
    else
    {
        request->send(400, "text/plain", "Invalid gyro calibration action");
        return;
    }

    GetGyroState(request);
}

void publishGyroRuntime(uint32_t now)
{
    if (!gyroDetected() || !GyroRuntimeSocketIsReady() || now - gyroRuntimeLastSentMs < GYRO_RUNTIME_INTERVAL_MS)
    {
        return;
    }

    gyroRuntimeLastSentMs = now;
    JsonDocument json;
    AppendGyroState(json.to<JsonObject>());
    const size_t length = measureJson(json);
    auto *message = gyroRuntimeSocket.makeBuffer(length);
    if (message != nullptr)
    {
        serializeJson(json, message->get(), length);
        gyroRuntimeSocket.textAll(message);
    }
}

void appendGyroConfig(JsonObject json)
{
    if (!gyroDetected())
    {
        return;
    }

    AppendGyroSummary(json["gyro"].to<JsonObject>());
}

void addGyroHandlers(AsyncWebServer &server)
{
    server.on("/gyro.json", HTTP_GET, GetGyroState);
    server.on("/gyro-config.json", HTTP_GET, GetGyroConfig);
    server.addHandler(new AsyncCallbackJsonWebHandler("/gyro-config.json", SetGyroConfig));
    server.addHandler(new AsyncCallbackJsonWebHandler("/gyro-calibration.json", SetGyroCalibration));
    server.addHandler(&gyroRuntimeSocket);
}

#endif
