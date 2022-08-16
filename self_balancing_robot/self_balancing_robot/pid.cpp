#include"pid.h"

PID::PID(int kp,int ki, int kd,float* input, float *output, float* setpoint){
    KP= kp;
    KD = kd;
    KI= ki;
    Input = input;
    Output = output;
    Setpoint = setpoint;
}