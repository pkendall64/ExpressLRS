/***
 * This file defines the interface from device units to functions in
 * either rx_main or tx_main (or rxtx_common but exposed to other units)
 * Use this instead of drectly declaring externs in your unit
 ***/

#include "common.h"

/***
 * In both RX and TX builds
 */
void EnterBindingModeSafely();
void scheduleRebootTime(unsigned long inMs);

/***
 * TX interface
 ***/
#if defined(TARGET_TX)
void SetSyncSpam();

/**
 * @brief send MSP packet to the backpack.
 *
 * @param packet the MSP packet to send.
 */
void sendMSPToBackpack(const void *packet);
const char *getBackpackVersion();
#endif

/***
 * RX interface
 ***/
#if defined(TARGET_RX)
uint8_t getLq();
#endif
