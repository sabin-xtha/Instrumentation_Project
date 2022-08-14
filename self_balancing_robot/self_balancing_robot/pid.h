#pragma once
#ifndef _PID_H_
#define _PID_H_
#include"defines.h"
//define necessary global varibles for PID constrain
#define P_ON_M 0
#define P_ON_E 1
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1
class continuous_PID 
{
private:
	float kp, ki, kd;
	float lastInput,currInput,ITerm,Output;
	float outMin, outMax;
	unsigned long lastTime, SampleTime;
	bool pOnE, inAuto;
	int controllerDirection;
public:
	continuous_PID ()
	{  
		Output=0;
		currInput=0;
		SampleTime=10;
		inAuto=false;
		SetOutputLimits(-255, 255);
		SetMode(AUTOMATIC);
		SetControllerDirection(DIRECT);
		SetTunings(0, 0, 0, P_ON_E);
		lastTime =0;

	}
	void SetTunings(float, float, float, int);
	void SetSampleTime(int);
	void SetOutputLimits(float, float);
	void SetMode(int);
	void Initialize(void);
	void SetControllerDirection(int);
	float Compute(float,float);

};
class discrete_PID
{ private:
    float l_output;
    float l_err;
    float ll_err;
	float outMin, outMax;
	float kp, ki, kd;
	unsigned long lastTime, SampleTime;
  public:
    discrete_PID()
	{ l_output=0;
      l_err=0;
      ll_err=0;
	  SetOutputLimits(-255, 255);
	  SetTunings(0, 0, 0);
	  lastTime =0;
	  SampleTime=10;
	}
	void SetTunings(float, float, float);
	 void SetSampleTime(int);
	void SetOutputLimits(float, float);
	float Compute(float,float);
};
#endif //!_PID_H_