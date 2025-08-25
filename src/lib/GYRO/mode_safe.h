#pragma once
#include "controller.h"

class SafeController : public Controller
{
public:
    void initialize() override;
    void update() override;

private:
    uint32_t prev_us = 0;
    EdgeSmoother roll_smooth, pitch_smooth;
};
