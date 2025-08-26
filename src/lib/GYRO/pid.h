#pragma once

class PID
{
public:
    PID() {}
    PID(float max, float min, float Kp, float Ki, float Kd);
    void configure(float Kp, float Ki, float Kd, float max = 1.0, float min = -1.0);
    float calculate(float setpoint, float pv);
    void reset();

    float output = 0;

    float setpoint;
    float pv;
    float Iout;
    float Dout = 0;
    float error;

private:
    unsigned long last_update = 0;
    float _maximum;
    float _minimum;
    float _Kp;
    float _Ki;
    float _Kd;
    float _integral = 0;

    unsigned long t_delta = 0;

    /* Derivative low-pass filter time constant */
    float tau = 0;
    float prevMeasurement = 0;
};
