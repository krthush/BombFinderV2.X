#ifndef _DC_MOTOR_H
#define _DC_MOTOR_H

#define _XTAL_FREQ 8000000

#include <xc.h>

struct DC_motor { // Definition of DC_motor structure.
    char power; // Motor power, out of 100.
    char direction; // Motor direction, forward(1), reverse(0).
    unsigned char *dutyLowByte; // PWM duty low byte address.
    unsigned char *dutyHighByte; // PWM duty high byte address.
    char dir_pin; // Pin that controls direction on PORTB.
    int PWMperiod; // Base period of PWM cycle.
};

// Function prototypes

// Function to setup PWM
void initMotorPWM();

// Function to set PWM output from the values in the motor structure.
void setMotorPWM(struct DC_motor *m);

// Increases a motor to full power over a period of time.
void setMotorFullSpeed(struct DC_motor *m);

// Function to stop a motor gradually.
void stopMotor(struct DC_motor *m);

// Function to stop the robot gradually.
void stop(struct DC_motor *mL, struct DC_motor *mR); 

// Function to make the robot turn left.
void turnLeft(struct DC_motor *mL, struct DC_motor *mR, unsigned char power);

// Function to make the robot turn right.
void turnRight(struct DC_motor *mL, struct DC_motor *mR, unsigned char power);

// Function to make the robot power on the motors to the specified power without
// changing direction.
void fullSpeed(struct DC_motor *mL, struct DC_motor *mR, unsigned char power);

// Function to make the robot go backward at a given power setting.
void fullSpeedBack(struct DC_motor *mL, struct DC_motor *mR, unsigned char power);

#endif
