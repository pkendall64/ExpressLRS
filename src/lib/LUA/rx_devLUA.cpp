#ifdef TARGET_RX

#include "rxtx_devLua.h"
#include "helpers.h"
#include "devServoOutput.h"
#include "deferred.h"
#ifdef HAS_GYRO
#include "gyro.h"
#endif

#define RX_HAS_SERIAL1 (GPIO_PIN_SERIAL1_TX != UNDEF_PIN || OPT_HAS_SERVO_OUTPUT)

extern void reconfigureSerial();
#if defined(PLATFORM_ESP32)
extern void reconfigureSerial1();
#endif
extern bool BindingModeRequest;

static char modelString[] = "000";
#if defined(GPIO_PIN_PWM_OUTPUTS)
static char pwmModes[] = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot;Serial RX;Serial TX;I2C SCL;I2C SDA;Serial2 RX;Serial2 TX";
#endif

#if defined(HAS_GYRO)
// Must match gyro_sensor_align_t
static const char *gyroAlign = "0;90;180;270;Flip;F90;F180;F270";
// Must match mixer.h: gyro_input_channel_function_t
static const char *gyroInputChannelModes = "None;Roll;Pitch;Yaw;Mode;Gain";
// Must match mixer.h: gyro_output_channel_function_t
static const char *gyroOutputChannelModes = "None;Aileron;Elevator;Rudder;Elevon;V Tail";
// Must match gyro.h gyro_mode_t
static const char *gyroModes = "Off;Rate;SAFE;Level;Launch;Hover";
// Must match gyro_axis_t
static const char *gyroAxis = "Roll;Pitch;Yaw";
#endif

static struct luaItem_selection luaSerialProtocol = {
    {"Protocol", CRSF_TEXT_SELECTION},
    0, // value
#if defined(PLATFORM_STM32)
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro",
#else
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;MAVLink",
#endif
    STR_EMPTYSPACE
};

#if defined(PLATFORM_ESP32)
static struct luaItem_selection luaSerial1Protocol = {
    {"Protocol2", CRSF_TEXT_SELECTION},
    0, // value
    "Off;CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;Tramp;SmartAudio",
    STR_EMPTYSPACE
};
#endif

static struct luaItem_selection luaSBUSFailsafeMode = {
    {"SBUS failsafe", CRSF_TEXT_SELECTION},
    0, // value
    "No Pulses;Last Pos",
    STR_EMPTYSPACE
};

#if !defined(PLATFORM_STM32)
static struct luaItem_int8 luaTargetSysId = {
  {"Target SysID", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value - default to 1
      (uint8_t)1,       // min
      (uint8_t)255,     // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaSourceSysId = {
  {"Source SysID", CRSF_UINT8},
  {
    {
      (uint8_t)255,       // value - default to 255
      (uint8_t)1,         // min
      (uint8_t)255,       // max
    }
  },
  STR_EMPTYSPACE
};
#endif

#if defined(POWER_OUTPUT_VALUES)
static struct luaItem_selection luaTlmPower = {
    {"Tlm Power", CRSF_TEXT_SELECTION},
    0, // value
    strPowerLevels,
    "mW"
};
#endif

#if defined(GPIO_PIN_ANT_CTRL)
static struct luaItem_selection luaAntennaMode = {
    {"Ant. Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Antenna A;Antenna B;Diversity",
    STR_EMPTYSPACE
};
#endif

// Gemini Mode
#if defined(GPIO_PIN_NSS_2)
static struct luaItem_selection luaDiversityMode = {
    {"Rx Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Diversity;Gemini",
    STR_EMPTYSPACE
};
#endif

static struct luaItem_folder luaTeamraceFolder = {
    {"Team Race", CRSF_FOLDER},
};

static struct luaItem_selection luaTeamraceChannel = {
    {"Channel", CRSF_TEXT_SELECTION},
    0, // value
    "AUX2;AUX3;AUX4;AUX5;AUX6;AUX7;AUX8;AUX9;AUX10;AUX11;AUX12",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaTeamracePosition = {
    {"Position", CRSF_TEXT_SELECTION},
    0, // value
    "Disabled;1/Low;2;3;Mid;4;5;6/High",
    STR_EMPTYSPACE
};

//----------------------------Info-----------------------------------

static struct luaItem_string luaModelNumber = {
    {"Model Id", CRSF_INFO},
    modelString
};

static struct luaItem_string luaELRSversion = {
    {version, CRSF_INFO},
    commit
};

//----------------------------Info-----------------------------------

//---------------------------- WiFi -----------------------------


//---------------------------- WiFi -----------------------------

// --------------------------- Gyro Setup ---------------------------------

#if defined(HAS_GYRO)

static struct luaItem_int8 luaGyroLaunchAngle = {
  {"Launch Angle", CRSF_UINT8},
  {
    {
      (uint8_t)10, // value, not zero-based
      0,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroSAFEPitch = {
  {"SAFE Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroSAFERoll = {
  {"SAFE Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroLevelPitch = {
  {"Level Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroLevelRoll = {
  {"Level Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroHoverStrength = {
  //------------ Max length on RM Pocket
  {"Hover Auth", CRSF_UINT8},
  {
    {
      (uint8_t)8,  // value, not zero-based
      0,           // min
      15,          // max
    }
  },
  STR_EMPTYSPACE
};

static void luaparamGyroAlign(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroSensorAlignment((gyro_sensor_align_t) arg); }

static void luaparamGyroSAFEPitch(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroSAFEPitch(arg); }

static void luaparamGyroSAFERoll(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroSAFERoll(arg); }

static void luaparamGyroLevelPitch(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLevelPitch(arg); }

static void luaparamGyroLevelRoll(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLevelRoll(arg); }

static void luaparamGyroLaunchAngle(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLaunchAngle(arg); }

static void luaparamGyroHoverStrength(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroHoverStrength(arg); }

static void luaparamGyroCalibrate(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Calibrate gyro?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Calibrating gyro";
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
    gyro_event = GYRO_EVENT_CALIBRATE;
    devicesTriggerEvent();
    return;
  }
  else if (arg == lcsQuery)
  {
    msg = "Calibrating gyro";
    newStep = lcsExecuting;
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

static void luaparamGyroSubtrims(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set subtrims?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting subtrims";
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
    gyro_event = GYRO_EVENT_SUBTRIMS;
    devicesTriggerEvent();
    return;
  }
  else if (arg == lcsQuery)
  {
    msg = "Setting subtrims";
    newStep = lcsExecuting;
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

static struct luaItem_command luaGyroCalibrate = {
    {"Calibrate Gyro", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct luaItem_command luaGyroSubtrims = {
    {"Set Subtrims", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroModesFolder = {
    {"Gyro Modes", CRSF_FOLDER},
};

static struct luaItem_folder luaGyroGainFolder = {
    {"Gyro Gains", CRSF_FOLDER},
};

static struct luaItem_folder luaGyroInputFolder = {
    {"Gyro Inputs", CRSF_FOLDER},
};

static struct luaItem_int8 luaGyroInputChannel = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroInputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroInputChannelModes,
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroOutputFolder = {
    {"Gyro Outputs", CRSF_FOLDER},
};

static struct luaItem_int8 luaGyroOutputChannel = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroOutputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroOutputChannelModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroOutputInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos1 = {
    {"Position 1", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos2 = {
    {"Position 2", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos3 = {
    {"Position 3", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos4 = {
    {"Position 4", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos5 = {
    {"Position 5", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroSettingsFolder = {
    {"Gyro Settings", CRSF_FOLDER},
};

static void luaparamGyroInputChannel(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaUint8Value(&luaGyroInputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}
static void luaparamGyroInputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroInputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.input_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroOutputChannel(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaUint8Value(&luaGyroOutputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}
static void luaparamGyroOutputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.output_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroOutputInverted(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
  rx_config_gyro_channel_t newCh;
  newCh.raw = config.GetGyroChannel(ch)->raw;
  newCh.val.inverted = arg;

  config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroModePos1(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(0, (gyro_mode_t) arg); }
static void luaparamGyroModePos2(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(1, (gyro_mode_t) arg); }
static void luaparamGyroModePos3(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(2, (gyro_mode_t) arg); }
static void luaparamGyroModePos4(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(3, (gyro_mode_t) arg); }
static void luaparamGyroModePos5(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(4, (gyro_mode_t) arg); }

static struct luaItem_selection luaGyroAlign = {
    {"Gyro Align", CRSF_TEXT_SELECTION},
    0, // value
    gyroAlign,
    STR_EMPTYSPACE
};

// contents of "Gyro Gains" folder, per axis subfolders
static struct luaItem_selection luaGyroGainAxis = {
    {"Gyro Axis", CRSF_TEXT_SELECTION},
    0, // value
    gyroAxis,
    STR_EMPTYSPACE
};

static void luaparamGyroGainAxis(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaTextSelectionValue(&luaGyroGainAxis, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}

static struct luaItem_int8 luaGyroPIDRateP = {
  {"P Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaGyroPIDRateI = {
  {"I Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaGyroPIDRateD = {
  {"D Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaGyroPIDGain = {
  {"Axis Gain", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      255            // max
    }
  },
  STR_EMPTYSPACE
};

static void luaparamGyroPIDRateP(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_P, arg);
}

static void luaparamGyroPIDRateI(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_I, arg);
}

static void luaparamGyroPIDRateD(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_D, arg);
}

static void luaparamGyroPIDGain(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDGain(axis, arg);
}

#endif // USE_GYRO

// --------------------------- Gyro Setup ---------------------------------

//---------------------------- Output Mapping -----------------------------

#if defined(GPIO_PIN_PWM_OUTPUTS)
static struct luaItem_folder luaMappingFolder = {
    {"Output Mapping", CRSF_FOLDER},
};

static struct luaItem_int8 luaMappingChannelOut = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)5,       // value - start on AUX1, value is 1-16, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaMappingChannelIn = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      0,                 // value
      1,                 // min
      CRSF_NUM_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingOutputMode = {
    {"Output Mode", CRSF_TEXT_SELECTION},
    0, // value
    pwmModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static struct luaItem_command luaSetFailsafe = {
    {"Set Failsafe Pos", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

constexpr char STR_US[] = "us";
static struct luaItem_int16 luaMappingChannelLimitMin = {
  {"Limit Min", CRSF_UINT16},
  {
    .u = {
      0,  // value
      885,  // min
      2115, // max
    }
  },
  STR_US
};

static struct luaItem_int16 luaMappingChannelLimitMax = {
  {"Limit Max", CRSF_UINT16},
  {
    .u = {
      0, // value
      885, // min
      2115, // max
    }
  },
  STR_US
};

#endif // GPIO_PIN_PWM_OUTPUTS

//---------------------------- Output Mapping -----------------------------

static struct luaItem_selection luaBindStorage = {
    {"Bind Storage", CRSF_TEXT_SELECTION},
    0, // value
    "Persistent;Volatile;Returnable;Administered",
    STR_EMPTYSPACE
};

static struct luaItem_command luaBindMode = {
    {STR_EMPTYSPACE, CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

#if defined(GPIO_PIN_PWM_OUTPUTS)
static void luaparamMappingChannelOut(struct luaPropertiesCommon *item, uint8_t arg)
{
    bool sclAssigned = false;
    bool sdaAssigned = false;
#if defined(PLATFORM_ESP32)
    bool serial1rxAssigned = false;
    bool serial1txAssigned = false;
#endif

    const char *no1Option    = ";";
    const char *no2Options   = ";;";
    const char *dshot        = ";DShot";
    const char *serial_RX    = ";Serial RX";
    const char *serial_TX    = ";Serial TX";
    const char *i2c_SCL      = ";I2C SCL;";
    const char *i2c_SDA      = ";;I2C SDA";
    const char *i2c_BOTH     = ";I2C SCL;I2C SDA";
#if defined(PLATFORM_ESP32)
    const char *serial1_RX   = ";Serial2 RX;";
    const char *serial1_TX   = ";;Serial2 TX";
    const char *serial1_BOTH = ";Serial2 RX;Serial2 TX";
#endif

    const char *pModeString;


    // find out if use once only modes have already been assigned
    for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
    {
      if (ch == (arg -1))
        continue;

      eServoOutputMode mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;

      if (mode == somSCL)
        sclAssigned = true;

      if (mode == somSDA)
        sdaAssigned = true;

#if defined(PLATFORM_ESP32)
      if (mode == somSerial1RX)
        serial1rxAssigned = true;

      if (mode == somSerial1TX)
        serial1txAssigned = true;
#endif
    }

    setLuaUint8Value(&luaMappingChannelOut, arg);

    // When the selected output channel changes, update the available PWM modes for that pin
    // Truncate the select options before the ; following On/Off
    pwmModes[50] = '\0';

#if defined(PLATFORM_ESP32)
    // DShot output (1 option)
    // ;DShot
    // ESP8266 enum skips this, so it is never present
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] != 0)   // DShot doesn't work with GPIO0, exclude it
    {
        pModeString = dshot;
    }
    else
#endif
    {
        pModeString = no1Option;
    }
    strcat(pwmModes, pModeString);

    // SerialIO outputs (1 option)
    // ;[Serial RX] | [Serial TX]
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0RXD_GPIO_NUM)
    {
        pModeString = serial_RX;
    }
    else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0TXD_GPIO_NUM)
    {
        pModeString = serial_TX;
    }
    else
    {
        pModeString = no1Option;
    }
    strcat(pwmModes, pModeString);

    // I2C pins (2 options)
    // ;[I2C SCL] ;[I2C SDA]
    if (GPIO_PIN_SCL != UNDEF_PIN || GPIO_PIN_SDA != UNDEF_PIN)
    {
        // If the target defines SCL/SDA then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SCL)
        {
            pModeString = i2c_SCL;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SDA)
        {
            pModeString = i2c_SDA;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    else
    {
        // otherwise allow any pin to be either SCL or SDA but only once
        if (sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_SDA;
        }
        else if (sdaAssigned && !sclAssigned)
        {
            pModeString = i2c_SCL;
        }
        else if (!sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_BOTH;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);

    // nothing to do for unsupported somPwm mode
    strcat(pwmModes, no1Option);

#if defined(PLATFORM_ESP32)
    // secondary Serial pins (2 options)
    // ;[SERIAL2 RX] ;[SERIAL2_TX]
    if (GPIO_PIN_SERIAL1_RX != UNDEF_PIN || GPIO_PIN_SERIAL1_TX != UNDEF_PIN)
    {
        // If the target defines Serial2 RX/TX then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_RX)
        {
            pModeString = serial1_RX;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_TX)
        {
            pModeString = serial1_TX;
        }
        else
        { 
            pModeString = no2Options;
        }
    } 
    else
    {   // otherwise allow any pin to be either RX or TX but only once
        if (serial1txAssigned && !serial1rxAssigned)
        {
            pModeString = serial1_RX;
        }        
        else if (serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_TX;
        }

        else if (!serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_BOTH;
        } 
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);
#endif

    // trim off trailing semicolons (assumes pwmModes has at least 1 non-semicolon)
    for (auto lastPos = strlen(pwmModes)-1; pwmModes[lastPos] == ';'; lastPos--)
    {
        pwmModes[lastPos] = '\0';
    }

    // Trigger an event to update the related fields to represent the selected channel
    devicesTriggerEvent();
}

static void luaparamMappingChannelIn(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inputChannel = arg - 1; // convert 1-16 -> 0-15

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void configureSerialPin(uint8_t sibling, uint8_t oldMode, uint8_t newMode)
{
  for (int ch=0 ; ch<GPIO_PIN_PWM_OUTPUTS_COUNT ; ch++)
  {
    if (GPIO_PIN_PWM_OUTPUTS[ch] == sibling)
    {
      // Retain as much of the sibling's current config as possible
      rx_config_pwm_t siblingPinConfig;
      siblingPinConfig.raw = config.GetPwmChannel(ch)->raw;

      // If the new mode is serial, the sibling is also forced to serial
      if (newMode == somSerial)
      {
        siblingPinConfig.val.mode = somSerial;
      }
      // If the new mode is not serial, and the sibling is serial, set the sibling to PWM (50Hz)
      else if (siblingPinConfig.val.mode == somSerial)
      {
        siblingPinConfig.val.mode = som50Hz;
      }

      config.SetPwmChannelRaw(ch, siblingPinConfig.raw);
      break;
    }
  }

  if (oldMode != newMode)
  {
    deferExecutionMillis(100, [](){
      reconfigureSerial();
    });
  }
}

static void luaparamMappingOutputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  uint8_t oldMode = newPwmCh.val.mode;
  newPwmCh.val.mode = arg;

  // Check if pin == 1/3 and do other pin adjustment accordingly
  if (GPIO_PIN_PWM_OUTPUTS[ch] == 1)
  {
    configureSerialPin(3, oldMode, newPwmCh.val.mode);
  }
  else if (GPIO_PIN_PWM_OUTPUTS[ch] == 3)
  {
    configureSerialPin(1, oldMode, newPwmCh.val.mode);
  }
  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamMappingInverted(struct luaPropertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inverted = arg;

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamSetFailsafe(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set failsafe to curr?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting failsafe";

    for (int ch=0; ch<GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
      rx_config_pwm_t newPwmCh;
      newPwmCh.raw = config.GetPwmChannel(ch)->raw;
      newPwmCh.val.failsafe = CRSF_to_US(constrain(ChannelData[config.GetPwmChannel(ch)->val.inputChannel], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX));
      //DBGLN("FSCH(%u) crsf=%u us=%u", ch, ChannelData[ch], newPwmCh.val.failsafe);
      config.SetPwmChannelRaw(ch, newPwmCh.raw);
    }
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

static void luaparamMappingChannelLimitMin(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  auto limits = config.GetPwmChannelLimits(ch);
  config.SetPwmChannelLimits(ch, arg, limits->val.max);
}

static void luaparamMappingChannelLimitMax(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  auto limits = config.GetPwmChannelLimits(ch);
  config.SetPwmChannelLimits(ch, limits->val.min, arg);
}
#endif // GPIO_PIN_PWM_OUTPUTS

#if defined(POWER_OUTPUT_VALUES)

static void luaparamSetPower(struct luaPropertiesCommon* item, uint8_t arg)
{
  UNUSED(item);
  uint8_t newPower = arg + POWERMGNT::getMinPower();
  if (newPower > POWERMGNT::getMaxPower())
  {
    newPower = PWR_MATCH_TX;
  }

  config.SetPower(newPower);
  // POWERMGNT::setPower() will be called in updatePower() in the main loop
}

#endif // POWER_OUTPUT_VALUES

static void registerLuaParameters()
{
  registerLUAParameter(&luaSerialProtocol, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetSerialProtocol((eSerialProtocol)arg);
    if (config.IsModified()) {
      deferExecutionMillis(100, [](){
        reconfigureSerial();
      });
    }
  });

#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    registerLUAParameter(&luaSerial1Protocol, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetSerial1Protocol((eSerial1Protocol)arg);
      if (config.IsModified()) {
        deferExecutionMillis(100, [](){
          reconfigureSerial1();
        });
      }
    });
  }
#endif

  registerLUAParameter(&luaSBUSFailsafeMode, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetFailsafeMode((eFailsafeMode)arg);
  });

#if !defined(PLATFORM_STM32)
  registerLUAParameter(&luaTargetSysId, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetTargetSysId((uint8_t)arg);
  });
  registerLUAParameter(&luaSourceSysId, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetSourceSysId((uint8_t)arg);
  });
#endif

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    registerLUAParameter(&luaAntennaMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg);
    });
  }

  // Gemini Mode
  if (isDualRadio())
  {
    registerLUAParameter(&luaDiversityMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
    });
  }

#if defined(POWER_OUTPUT_VALUES)
  luadevGeneratePowerOpts(&luaTlmPower);
  registerLUAParameter(&luaTlmPower, &luaparamSetPower);
#endif

  // Teamrace
  registerLUAParameter(&luaTeamraceFolder);
  registerLUAParameter(&luaTeamraceChannel, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetTeamraceChannel(arg + AUX2);
  }, luaTeamraceFolder.common.id);
  registerLUAParameter(&luaTeamracePosition, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetTeamracePosition(arg);
  }, luaTeamraceFolder.common.id);

#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    luaparamMappingChannelOut(&luaMappingOutputMode.common, luaMappingChannelOut.properties.u.value);
    registerLUAParameter(&luaMappingFolder);
    registerLUAParameter(&luaMappingChannelOut, &luaparamMappingChannelOut, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingChannelIn, &luaparamMappingChannelIn, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingOutputMode, &luaparamMappingOutputMode, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingInverted, &luaparamMappingInverted, luaMappingFolder.common.id);
    registerLUAParameter(&luaSetFailsafe, &luaparamSetFailsafe);
    registerLUAParameter(&luaMappingChannelLimitMin, &luaparamMappingChannelLimitMin, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingChannelLimitMax, &luaparamMappingChannelLimitMax, luaMappingFolder.common.id);

    #if defined(HAS_GYRO)
    registerLUAParameter(&luaGyroModesFolder);
    registerLUAParameter(&luaGyroModePos1, &luaparamGyroModePos1, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos2, &luaparamGyroModePos2, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos3, &luaparamGyroModePos3, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos4, &luaparamGyroModePos4, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos5, &luaparamGyroModePos5, luaGyroModesFolder.common.id);

    registerLUAParameter(&luaGyroGainFolder);
    registerLUAParameter(&luaGyroGainAxis, &luaparamGyroGainAxis, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateP, &luaparamGyroPIDRateP, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateI, &luaparamGyroPIDRateI, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateD, &luaparamGyroPIDRateD, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDGain, &luaparamGyroPIDGain, luaGyroGainFolder.common.id);

    registerLUAParameter(&luaGyroInputFolder);
    registerLUAParameter(&luaGyroInputChannel, &luaparamGyroInputChannel, luaGyroInputFolder.common.id);
    registerLUAParameter(&luaGyroInputMode, &luaparamGyroInputMode, luaGyroInputFolder.common.id);

    registerLUAParameter(&luaGyroOutputFolder);
    registerLUAParameter(&luaGyroOutputChannel, &luaparamGyroOutputChannel, luaGyroOutputFolder.common.id);
    registerLUAParameter(&luaGyroOutputMode, &luaparamGyroOutputMode, luaGyroOutputFolder.common.id);
    registerLUAParameter(&luaGyroOutputInverted, &luaparamGyroOutputInverted, luaGyroOutputFolder.common.id);

    registerLUAParameter(&luaGyroSettingsFolder);
    registerLUAParameter(&luaGyroAlign, &luaparamGyroAlign, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroCalibrate, &luaparamGyroCalibrate, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSubtrims, &luaparamGyroSubtrims, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSAFEPitch, &luaparamGyroSAFEPitch, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSAFERoll, &luaparamGyroSAFERoll, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLevelPitch, &luaparamGyroLevelPitch, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLevelRoll, &luaparamGyroLevelRoll, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLaunchAngle, &luaparamGyroLaunchAngle, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroHoverStrength, &luaparamGyroHoverStrength, luaGyroSettingsFolder.common.id);
    #endif
  }
#endif

  registerLUAParameter(&luaBindStorage, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetBindStorage((rx_config_bindstorage_t)arg);
  });
  registerLUAParameter(&luaBindMode, [](struct luaPropertiesCommon* item, uint8_t arg){
    // Complete when TX polls for status i.e. going back to idle, because we're going to lose connection
    if (arg == lcsQuery) {
      deferExecutionMillis(200, EnterBindingModeSafely);
    }
    sendLuaCommandResponse(&luaBindMode, arg < 5 ? lcsExecuting : lcsIdle, arg < 5 ? "Entering..." : "");
  });

  registerLUAParameter(&luaModelNumber);
  registerLUAParameter(&luaELRSversion);
}

static void updateBindModeLabel()
{
  if (config.IsOnLoan())
    luaBindMode.common.name = "Return Model";
  else
    luaBindMode.common.name = "Enter Bind Mode";
}

static int event()
{
  setLuaTextSelectionValue(&luaSerialProtocol, config.GetSerialProtocol());
#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    setLuaTextSelectionValue(&luaSerial1Protocol, config.GetSerial1Protocol());
  }
#endif
  
  setLuaTextSelectionValue(&luaSBUSFailsafeMode, config.GetFailsafeMode());

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    setLuaTextSelectionValue(&luaAntennaMode, config.GetAntennaMode());
  }

  // Gemini Mode
  if (isDualRadio())
  {
    setLuaTextSelectionValue(&luaDiversityMode, config.GetAntennaMode()); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
  }

#if defined(POWER_OUTPUT_VALUES)
  // The last item (for MatchTX) will be MaxPower - MinPower + 1
  uint8_t luaPwrVal = (config.GetPower() == PWR_MATCH_TX) ? POWERMGNT::getMaxPower() + 1 : config.GetPower();
  setLuaTextSelectionValue(&luaTlmPower, luaPwrVal - POWERMGNT::getMinPower());
#endif

  // Teamrace
  setLuaTextSelectionValue(&luaTeamraceChannel, config.GetTeamraceChannel() - AUX2);
  setLuaTextSelectionValue(&luaTeamracePosition, config.GetTeamracePosition());

#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    const rx_config_pwm_t *pwmCh = config.GetPwmChannel(luaMappingChannelOut.properties.u.value - 1);
    setLuaUint8Value(&luaMappingChannelIn, pwmCh->val.inputChannel + 1);
    setLuaTextSelectionValue(&luaMappingOutputMode, pwmCh->val.mode);
    setLuaTextSelectionValue(&luaMappingInverted, pwmCh->val.inverted);
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(luaMappingChannelOut.properties.u.value - 1);
    setLuaUint16Value(&luaMappingChannelLimitMin, (uint16_t) limits->val.min);
    setLuaUint16Value(&luaMappingChannelLimitMax, (uint16_t) limits->val.max);

    #if defined(HAS_GYRO)
    const rx_config_gyro_channel_t *gyroChIn = config.GetGyroChannel(luaGyroInputChannel.properties.u.value - 1);
    setLuaTextSelectionValue(&luaGyroInputMode, gyroChIn->val.input_mode);
    const rx_config_gyro_channel_t *gyroChOut = config.GetGyroChannel(luaGyroOutputChannel.properties.u.value - 1);
    setLuaTextSelectionValue(&luaGyroOutputMode, gyroChOut->val.output_mode);
    setLuaTextSelectionValue(&luaGyroOutputInverted, gyroChOut->val.inverted);

    const rx_config_gyro_mode_pos_t *gyroModes = config.GetGyroModePos();
    setLuaTextSelectionValue(&luaGyroModePos1, gyroModes->val.pos1);
    setLuaTextSelectionValue(&luaGyroModePos2, gyroModes->val.pos2);
    setLuaTextSelectionValue(&luaGyroModePos3, gyroModes->val.pos3);
    setLuaTextSelectionValue(&luaGyroModePos4, gyroModes->val.pos4);
    setLuaTextSelectionValue(&luaGyroModePos5, gyroModes->val.pos5);

    const rx_config_gyro_gains_t *gyroGains = config.GetGyroGains((gyro_axis_t) (luaGyroGainAxis.value));
    setLuaUint8Value(&luaGyroPIDRateP, gyroGains->p);
    setLuaUint8Value(&luaGyroPIDRateI, gyroGains->i);
    setLuaUint8Value(&luaGyroPIDRateD, gyroGains->d);
    setLuaUint8Value(&luaGyroPIDGain, gyroGains->gain);

    setLuaTextSelectionValue(&luaGyroAlign, config.GetGyroSensorAlignment());
    setLuaUint8Value(&luaGyroSAFEPitch, config.GetGyroSAFEPitch());
    setLuaUint8Value(&luaGyroSAFERoll, config.GetGyroSAFERoll());
    setLuaUint8Value(&luaGyroLevelPitch, config.GetGyroLevelPitch());
    setLuaUint8Value(&luaGyroLevelRoll, config.GetGyroLevelRoll());
    setLuaUint8Value(&luaGyroLaunchAngle, config.GetGyroLaunchAngle());
    setLuaUint8Value(&luaGyroHoverStrength, config.GetGyroHoverStrength());
    #endif // HAS_GYRO
  }
#endif

  if (config.GetModelId() == 255)
  {
    setLuaStringValue(&luaModelNumber, "Off");
  }
  else
  {
    itoa(config.GetModelId(), modelString, 10);
    setLuaStringValue(&luaModelNumber, modelString);
  }
  setLuaTextSelectionValue(&luaBindStorage, config.GetBindStorage());
  updateBindModeLabel();

#if !defined(PLATFORM_STM32)
  if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
  {
    setLuaUint8Value(&luaSourceSysId, config.GetSourceSysId() == 0 ? 255 : config.GetSourceSysId());  //display Source sysID if 0 display 255 to mimic logic in SerialMavlink.cpp
    setLuaUint8Value(&luaTargetSysId, config.GetTargetSysId() == 0 ? 1 : config.GetTargetSysId());  //display Target sysID if 0 display 1 to mimic logic in SerialMavlink.cpp
    LUA_FIELD_SHOW(luaSourceSysId)
    LUA_FIELD_SHOW(luaTargetSysId)
  }
  else
  {
    LUA_FIELD_HIDE(luaSourceSysId)
    LUA_FIELD_HIDE(luaTargetSysId)
  }
#endif

  return DURATION_IMMEDIATELY;
}

static int timeout()
{
  luaHandleUpdateParameter();
  // Receivers can only `UpdateParamReq == true` every 4th packet due to the transmitter cadence in 1:2
  // Channels, Downlink Telemetry Slot, Uplink Telemetry (the write command), Downlink Telemetry Slot...
  // (interval * 4 / 1000) or 1 second if not connected
  return (connectionState == connected) ? ExpressLRS_currAirRate_Modparams->interval / 250 : 1000;
}

static int start()
{
  registerLuaParameters();
  event();
  return DURATION_IMMEDIATELY;
}

device_t LUA_device = {
  .initialize = nullptr,
  .start = start,
  .event = event,
  .timeout = timeout
};

#endif
