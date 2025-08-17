#pragma once
#include "controller.h"

class RateController : public Controller
{
public:
    void initialize() override;
    void update() override;
};
