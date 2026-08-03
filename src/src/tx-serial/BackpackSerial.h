#pragma once

#include "device.h"

extern device_t BackpackSerial_device;

/**
 * @brief send MAVLink telemetry packet (inside a CRSF packet) to the backpack.
 *
 * @param data the MAVLink telemetry packet to send.
 */
extern void sendMAVLinkTelemetryToBackpack(const uint8_t *data, uint8_t count);

/**
 * @brief send CRSF telemetry packet to the backpack.
 *
 * The CRSF packet is wrapped in an MSP packet of type `MSP_ELRS_BACKPACK_CRSF_TLM`
 * and forwarded to the backpack.
 *
 * @param data the CRSF telemetry packet to send.
 */
extern void sendCRSFTelemetryToBackpack(const uint8_t *data);

/**
 * @brief perform check to see if a backpack firmware update has been requested.
 *
 * If a backpack update has been requested then all devices are stopped and the UARTs
 * are configured for passthrough flashing.
 */
extern void checkBackpackUpdate();

/**
 * @brief perform check to see if a backpack firmware update has been requested by detecting ESP sync packets on the serial port.
 *
 * If a backpack update has been requested then all devices are stopped and the UARTs
 * are configured for passthrough flashing.
 */
extern void checkForUpdateSync(const uint8_t *data, uint16_t count);
