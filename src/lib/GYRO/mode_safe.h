#pragma once
#include "controller.h"

class SafeController : public Controller
{
public:
    void initialize() override;
    void update() override;
};
