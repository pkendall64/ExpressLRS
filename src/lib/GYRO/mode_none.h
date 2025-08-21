#pragma once
#include "controller.h"

class NoneController : public Controller
{
public:
    void initialize() override
    {
    }
    void update() override
    {
        setOutput(GYRO_AXIS_ROLL, get_command(GYRO_AXIS_ROLL));
        setOutput(GYRO_AXIS_PITCH, get_command(GYRO_AXIS_PITCH));
        setOutput(GYRO_AXIS_YAW, get_command(GYRO_AXIS_YAW));
    }
};
