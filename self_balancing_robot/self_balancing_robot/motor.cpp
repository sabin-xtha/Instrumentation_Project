#include "motor.h"
#include <Arduino.h>

motor::motor(){
  motor_ = 0;
}
motor::motor(Motor_config *motor_conf){
    motor_ = motor_conf;
}

void motor::motors_init(){
    motorsetup(&motor_->motor1);
    motorsetup(&motor_->motor2);
}
void motor::runmotors(){
    runmotor(&motor_->motor1);
    runmotor(&motor_->motor2);
}

void runmotor(Motor_control *motor){
    int out1,out2;
    if(motor->direction==CLOCKWISE){
        out1 = 0;
        out2 = 1;
    }else if(motor->direction == ANTICLOCKWISE){
        out1 = 1;
        out2 = 0;
    }else{
        out1 = 0;
        out2 = 0;
    }
    digitalWrite(motor->outputpin1,out1);
    digitalWrite(motor->outputpin2,out2);
    analogWrite(motor->speedpin,motor->speed*motor->speedFactor);
}

void motorsetup(Motor_control *motor){
    pinMode(motor->outputpin1,OUTPUT);
    pinMode(motor->outputpin2,OUTPUT);
    pinMode(motor->speedpin,OUTPUT);
}
void motor::setDirection(Direction dir1,Direction dir2=-11110){
    if(dir2==-11110){
        motor_->motor1.direction=dir1;
        motor_->motor2.direction=dir1;
    }else{
        motor_->motor1.direction=dir1;
        motor_->motor2.direction=dir2;
    }
}
void motor::setSpeed(uint8_t speed){
    
        motor_->motor1.speed=speed;
        motor_->motor2.speed=speed;
    
}