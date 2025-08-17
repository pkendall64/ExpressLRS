#pragma once
#include "controller.h"

class LaunchController : public Controller
{
public:
    void initialize() override;
    void update() override;
};
