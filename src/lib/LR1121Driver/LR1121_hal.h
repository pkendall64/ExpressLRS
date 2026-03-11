#pragma once

#include "LR1121_Regs.h"
#include "LR1121.h"

class LR1121Hal
{
public:
    static LR1121Hal *instance;

    LR1121Hal();

    void init();
    void end();
    void reset(bool bootloader = false);

    void WriteCommand(uint16_t opcode, SX12XX_Radio_Number_t radioNumber);
    void WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);

    void ReadCommand(uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);

    bool WaitOnBusy(SX12XX_Radio_Number_t radioNumber);

    static void dioISR_1();
    static void dioISR_2();
    void (*IsrCallback_1)();
    void (*IsrCallback_2)();
};
