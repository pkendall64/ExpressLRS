#pragma once

#include "targets.h"
#include "options.h"
#include "common.h"

// CONFIG_MAGIC is ORed with CONFIG_VERSION in the version field
#define CONFIG_MAGIC_MASK   (0b11U << 30)
#define TX_CONFIG_MAGIC     (0b01U << 30)
#define RX_CONFIG_MAGIC     (0b10U << 30)

#if defined(TARGET_TX)
#include "tx_config.h"
#else
#include "rx_config.h"
#endif
