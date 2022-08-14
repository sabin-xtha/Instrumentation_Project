#include"pid.h"
/* for continious PID */
float continuous_PID:: Compute(float Setpoint,float Input)
{
    if (!inAuto) return 0;
    unsigned long now= HAL_GetTick();
    unsigned long timeChange = (now - lastTime);
    if (timeChange >= SampleTime)
    {   
        /*Compute all the working error variables*/
        currInput = Input;
        float error = Setpoint - Input;
        ITerm += (ki * error);
        float dInput = (Input - lastInput);
        if (!pOnE) ITerm -= kp * dInput;
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;

        /*Compute P-Term*/
        if (pOnE) Output = kp * error;
        else Output = 0;

        /*Compute Rest of PID Output*/
        Output += ITerm - kd * dInput;
        if (Output > outMax) Output = outMax;
        else if (Output < outMin) Output = outMin;

        /*Remember some variables for next time*/
        lastInput = Input;
        lastTime = now;
    }
    return Output;
}

void continuous_PID:: SetTunings(float Kp, float Ki, float Kd,int pOn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    pOnE = (pOn == P_ON_E);

    float SampleTimeInSec = (float)SampleTime / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if (controllerDirection == REVERSE)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void continuous_PID:: SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        float ratio = (float)NewSampleTime / (float)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (int)NewSampleTime;
    }
}

void continuous_PID:: SetOutputLimits(float Min, float Max)
{
    if (Min > Max) return;
    outMin = Min;
    outMax = Max;
}

void continuous_PID:: SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void continuous_PID:: Initialize()
{
    lastInput = currInput;
    ITerm = Output;
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;
}

void continuous_PID:: SetControllerDirection(int Direction)
{
    controllerDirection = Direction;
}
/* end of continous pid*/
/* descrete pid */
float discrete_PID:: Compute(float Setpoint,float Input)
{
    unsigned long now= HAL_GetTick();
    unsigned long timeChange = (now - lastTime);
    if (timeChange >= SampleTime)
    {  /*Compute all the working error variables*/
        float error = Setpoint - Input;
        l_output+=(kp+ki+kd)*error+(ki-kp-2*kd)*l_err+ kd*ll_err;
        if ( l_output > outMax)  l_output = outMax;
        else if ( l_output< outMin)  l_output = outMin;

        /*Remember some variables for next time*/
        ll_err = l_err;
        l_err = error;
        lastTime = now;
    }
    return  l_output;
}

void discrete_PID:: SetTunings(float Kp, float Ki, float Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) return;
    float SampleTimeInSec = (float)SampleTime / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

void discrete_PID:: SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        float ratio = (float)NewSampleTime / (float)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (int)NewSampleTime;
    }
}

void discrete_PID:: SetOutputLimits(float Min, float Max)
{
    if (Min > Max) return;
    outMin = Min;
    outMax = Max;
}
/* end of descrete pid*/