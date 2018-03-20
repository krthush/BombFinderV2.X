#ifndef _DC_MOTOR_H
#define _DC_MOTOR_H

#define _XTAL_FREQ 8000000

#include <xc.h>

struct DC_motor { //definition of DC_motor structure
    char power;         //motor power, out of 100
    char direction;     //motor direction, forward(1), reverse(0)
    unsigned char *dutyLowByte; //PWM duty low byte address
    unsigned char *dutyHighByte; //PWM duty high byte address
    char dir_pin; // pin that controls direction on PORTB
    int PWMperiod; //base period of PWM cycle
};

//function prototypes
void initMotorPWM(); // function to setup PWM
void setMotorPWM(struct DC_motor *m); // function to set PWM output from the values in the motor structure
void setMotorFullSpeed(struct DC_motor *m); //increases a motor to full power over a period of time
void stopMotor(struct DC_motor *m); //function to stop a motor gradually 
void stop(struct DC_motor *mL, struct DC_motor *mR); //function to stop the robot gradually 
void turnLeft(struct DC_motor *mL, struct DC_motor *mR, unsigned char power); //function to make the robot turn left
void turnRight(struct DC_motor *mL, struct DC_motor *mR, unsigned char power); //function to make the robot turn right
void fullSpeed(struct DC_motor *mL, struct DC_motor *mR, unsigned char power); //function to make the robot power on the motors to the specified power without
//changing direction
void fullSpeedBack(struct DC_motor *mL, struct DC_motor *mR, unsigned char power); //function to make the robot go backward at a given power setting

#endif
