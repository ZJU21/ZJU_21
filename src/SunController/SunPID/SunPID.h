#ifndef SUN_PID_H
#define SUN_PID_H

#include "Arduino.h"

class PID
{
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double Compute(float setpoint, float measured_value);
        void UpdateTunings(float kp, float ki, float kd);

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double integral_;
        double derivative_;
        double prev_error_;
};

#endif
