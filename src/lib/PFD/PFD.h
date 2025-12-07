#pragma once
#include <Arduino.h>
#include <stdio.h>
#include "../../src/include/targets.h"

class PFD
{
public:
    void extEvent(const int32_t time) // reference (external osc)
    {
        extEventTime = time;
        gotExtEvent = true;
    }

    void intEvent(const uint32_t time) // internal osc event
    {
        intEventTime = time;
        gotIntEvent = true;
    }

    void reset()
    {
        gotExtEvent = false;
        gotIntEvent = false;
    }

    int32_t calcResult() const
    {
        // Assumes caller has verified hasResult()
        return (int32_t)(extEventTime - intEventTime);
    }

    bool hasResult() const
    {
        return gotExtEvent && gotIntEvent;
    }

    uint32_t getIntEventTime() const { return intEventTime; }
    uint32_t getExtEventTime() const { return extEventTime; }

private:
    uint32_t intEventTime = 0;
    uint32_t extEventTime = 0;
    bool gotExtEvent = false;
    bool gotIntEvent = false;
};
