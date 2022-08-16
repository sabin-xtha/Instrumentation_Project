#ifndef _PID_H
#define _PID_H
class PID{
    private:
    //turning parameters
    int KP;
    int KD;
    int KI;
    // direction 
    bool direction;
    //variable to store the input output and the setpoint 
    //being pointer it can be accessed at any time without invoking any function or operation 
    float *Input;
    float *Output;
    float *Setpoint;
    public:
    PID(int, int, int,float*,float*,float*);
    

};
#endif