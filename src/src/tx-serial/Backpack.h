#pragma once

#include <sys/_stdint.h>

void resetBackpackChannelData();
void ParseMSPData(uint8_t *buf, uint8_t size);
void ProcessPendingCommands();
void ProcessEvents(bool disabled);
