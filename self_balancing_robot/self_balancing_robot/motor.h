#ifndef _MOTORS_H_
#define IN_MOTORS_H_
#include<stdint.h>
//enumeration to define the direction of the motor;
enum Direction{
    CLOCKWISE,  
    ANTICLOCKWISE,
    STOP
};


// this structure defines pins and direction for individual motors;
struct Motor_control{
    uint8_t outputpin1;
    uint8_t outputpin2;
    uint8_t speedpin;
    uint8_t speed;// this is pwm value 0-255
    Direction direction;
    float speedFactor;// this value optimizes the speed of motor and makes the multiple motor same
};

//this sturcture defines the configuration of 2 motors in a single structure
struct Q//*Q12Motor_config{
    Motor_control motor1; 
    Motor_control motor2;
};


//this class defines the operations for the motors
class motor{
    private:
    Motor_config *motor_;
    public:
    motor();
    motor(Motor_config *motor_conf);
    void motors_init();
    void runmotors();
    void setDirection(Direction dir1,Direction dir2=-11110);
    void setSpeed(uint8_t speed);
    
};
void motorsetup(Motor_control *motor);
void runmotor(Motor_control *motor);
#endif