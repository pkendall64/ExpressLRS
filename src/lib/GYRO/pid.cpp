#include "targets.h"
#include "pid.h"



PID::PID(const float max, const float min, const float Kp, const float Ki, const float Kd)
    : _maximum(max),
      _minimum(min),
      _Kp(Kp),
      _Ki(Ki),
      _Kd(Kd)
{
}

void PID::configure(const float Kp, const float Ki, const float Kd, const float max, const float min)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd * 10;
    _maximum = max;
    _minimum = min;
    tau = 0.02;
}

void PID::reset()
{
    _integral = 0;
    Dout = 0;
    output = 0;
}

float PID::calculate(const float _setpoint, const float _pv)
{
    const unsigned long now = micros();
    t_delta = now - last_update;
    t_delta = t_delta == 0 ? 1 : t_delta; // Stop any chance of div/0
    const float _dt = 1.0f / t_delta;
    last_update = now;

    setpoint = _setpoint;
    pv = _pv;

    // Calculate error
    error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;

    // Limit the 'I' accumulation within min/max
    _integral = (_integral * _Ki) > _maximum
                    ? _maximum / _Ki
                : (_integral * _Ki) < _minimum
                    ? _minimum / _Ki
                    : _integral;

    Iout = _Ki * _integral;

    // Derivative term
    Dout = -(2.0f * _Kd * (pv - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
           + (2.0f * tau - t_delta) * Dout)
           / (2.0f * tau + t_delta);
    prevMeasurement = pv;

    // Calculate total output
    output = Pout + Iout + Dout;

    // Limit output
    output = constrain(output, _minimum, _maximum);

    return output;
}
