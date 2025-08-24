#pragma once
#include "controller.h"

class LevelController : public Controller
{
public:
    void initialize() override;
    void update() override;

protected:
    float pitch_bias = 0.0f;
};
