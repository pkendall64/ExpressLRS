#pragma once
#include "controller.h"

class HoverController final : public Controller
{
public:
    void initialize() override;
    void update() override;
};
