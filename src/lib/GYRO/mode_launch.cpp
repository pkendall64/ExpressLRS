#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_launch.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 */

void LaunchController::initialize()
{
    configure_pids(1.0, 1.0, 1.0);
    pitch_bias = FusionDegreesToRadians(config.GetGyroLaunchAngle());
}

#endif
