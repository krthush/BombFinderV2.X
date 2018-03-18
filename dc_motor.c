#include <xc.h>
#include "dc_motor.h"

void initMotorPWM(){
	// your code to set up the PWM module
    PTCON0 = 0b00000000; // free running mode, 1:1 prescaler
    PTCON1 = 0b10000000; // enable PWM timer
    PWMCON0 = 0b01101111; // PWM1 and PWM3 set to output (pin6-4=110), PWM0/1 enabled, all independent mode
    PWMCON1 = 0x00; // special features, all 0 (default)
    //PWMperiod is 199
    PTPERL = 0b11000111; // base PWM period low byte
    PTPERH = 0b00000000; // base PWM period high byte 
    PDC0L = 0<<2;
    PDC0H = 0>>6;
    PDC1L = 0<<2;
    PDC1H = 0>>6;
    
    // set bits as outputs (Used for motors!) 
    TRISBbits.RB0=0;
    TRISBbits.RB1=0;
    TRISBbits.RB2=0;
    TRISBbits.RB3=0;
}

// function to set PWM output from the values in the motor structure
void setMotorPWM(struct DC_motor *m)
{
    int PWMduty; //tmp variable to store PWM duty cycle

    PWMduty = (m->power*m->PWMperiod)/100;  //calculate duty cycle (value between 0 and PWMperiod)
    
    if (m->direction) //if forward direction
    {
        LATB=LATB|(1<<(m->dir_pin)); //set dir_pin high in LATB
		PWMduty=m->PWMperiod-PWMduty; //need to invert duty cycle as direction is high (100% power is a duty cycle of 0)
    }
    else //if reverse direction
    {
        LATB=LATB&(~(1<<(m->dir_pin))); //set dir_pin low in LATB
    }

	//write duty cycle value to appropriate registers
    *(m->dutyLowByte)=PWMduty<<2;
    *(m->dutyHighByte)=PWMduty>>6;
}

//increases a motor to full power over a period of time
void setMotorFullSpeed(struct DC_motor *m)
{
	for (m->power; (m->power)<=100; m->power++){ //increase motor power until 100
		setMotorPWM(m);	//pass pointer to m to setMotorSpeed function (not &m)
		__delay_ms(1);	//delay of 1 ms (100 ms from 0 to 100 full power)
	}
}

//function to stop a motor gradually 
void stopMotor(struct DC_motor *m)
{
	// a loop to slow the motor power down to zero
     for (m->power; (m->power)>0; m->power--){ //increase motor power until 100
        setMotorPWM(m); //pass pointer to setMotorSpeed function (not &m here)
        __delay_ms(5); //delay of 5 ms (500 ms from 0 to 100 full power)
     }
}

//function to stop the robot gradually 
void stop(struct DC_motor *mL, struct DC_motor *mR)
{
	// a loop to slow both motors down to zero power
    while(mL->power>0 || mR->power>0){
        if(mL->power>0){
            mL->power--;            
        }
        if(mR->power>0){
            mR->power--;            
        }
        setMotorPWM(mL);
        setMotorPWM(mR);
        __delay_us(50);
    }
    mL->direction=1;
    mR->direction=1;
}

//function to make the robot turn left
// PLEASE NOTE: stop(motors) needs to called be correct use of this, WHY?!
void turnLeft(struct DC_motor *mL, struct DC_motor *mR)
{
    stop(mL, mR);
	//remember to change the power gradually
    mL->direction=0;
    mR->direction=1;
    fullSpeedAhead(mL, mR);
}

//function to make the robot turn right
// PLEASE NOTE: stop(motors) needs to called be correct use of this, WHY?!
void turnRight(struct DC_motor *mL, struct DC_motor *mR)
{
    stop(mL, mR);
	//remember to change the power gradually
    mL->direction=1;
    mR->direction=0;
    fullSpeedAhead(mL, mR);
}

//function to make the robot go straight
void fullSpeedAhead(struct DC_motor *mL, struct DC_motor *mR)
{
	//remember to change the power gradually
    while(mL->power<100 || mR->power<100){
        if(mL->power<100){
            mL->power++;            
        }
        if(mR->power<100){
            mR->power++;            
        }
        setMotorPWM(mL);
        setMotorPWM(mR);
        __delay_us(50);
    }    
}

//function to make the robot go backward
void fullSpeedBack(struct DC_motor *mL, struct DC_motor *mR)
{
    //remember to change the power gradually
    mL->direction=0;
    mR->direction=0;
    fullSpeedAhead(mL, mR);
}
