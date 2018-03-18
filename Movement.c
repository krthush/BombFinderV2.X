#include <xc.h>
#include "dc_motor.h"
#define _XTAL_FREQ 8000000
#include "IR_Reading.h"
#include "Movement.h"
#include "LCD.h"

void initTimer(void){
    //timer setup
    T0CONbits.TMR0ON=0; // turn off timer0
    T0CONbits.T016BIT=0; // 16-bit mode
    T0CONbits.T0CS=0; // use internal clock (Fosc/4)
    T0CONbits.PSA=0; // disable prescaler
    T0CONbits.T0PS=0b111; // 1:256 Prescale
    // With 8 MHz clock this gives the clock incrementing every 0.000128 s
    
    INTCONbits.TMR0IE=0; // Disable interrupt on overflow
}

// Function to delay in seconds
//__delay_ms() is limited to a maximum delay of 89ms with an 8Mhz
//clock so you need to write a function to make longer delays
void delay_s(char seconds) {
    unsigned int i=0;
    for (i=1; i<=seconds*20; i++) {
        // repeat 50ms delay 20 times to match no. of seconds
        __delay_ms(50);
    }
}

// Function similar to delay in seconds, but for a 1/10th of a second
void delay_tenth_s(char tenth_seconds) {
    unsigned int i=0;
    for (i=1; i<=tenth_seconds*2; i++) {
        // repeat 50ms delay 20 times to match no. of tenth seconds
        __delay_ms(50);
    }
}

// Simple search routine that compares the signal strength of the left IR and
// right IR readers to figure out the direction of the IR beacon.
// Function can be toggled to be:
// - continuous, the drone moves while scanning until there is a change
// - stop n scan, the drone stops every time it scans
char ScanIR(struct DC_motor *mL, struct DC_motor *mR, unsigned char *Move, char *MoveTime, char *MoveType){
    
    // Initialise variable that is used to judge the strength of signals
    unsigned int SensorResult[2]={0,0};
    char buf[40]; // Buffer for characters for LCD
    // USERVARIABLE TOLERANCES
    // minimum signal strength required for both sensors to be considered directly aimed at beacon while moving
    const unsigned int DirectionMoveThreshold=2500; 
    
    // Scan Data
    SensorResult[0]=grabLeftIR();
    SensorResult[1]=grabRightIR();

    // Reset the timers to avoid same reading being picked up if there is
    // no signal.
    CAP1BUFH=0;
    CAP1BUFL=0;
    CAP2BUFH=0;
    CAP2BUFL=0;       

    // Output signal strength to LCD
    SendLCD(0b00000001,0); //Clear Display
    __delay_us(50); //Delay to let display clearing finish
    SendLCD(0b00000010,0); // move cursor to home
    __delay_ms(2);
    SetLine(1); //Set Line 1
    LCD_String("     ScanIR");
    SetLine(2); //Set Line 2, for signal strength readings
    sprintf(buf,"     %04d, %04d",SensorResult[0],SensorResult[1]);
    LCD_String(buf);
    
    // If there is significant signal from both sensors keep going
    // Average of signals - tolerance vs. threshold
    if ((SensorResult[0]>DirectionMoveThreshold)&&(SensorResult[1]>DirectionMoveThreshold)) {
        return 2; // Direction of bomb is roughly directly ahead
    } else {
        stop(mL,mR);
        return 0; // No clear signal found
    }
}

// NEW ROUTINE: This route scans given range in very small time increments
// INPROG
char ScanWithRange(struct DC_motor *mL, struct DC_motor *mR, int milliseconds, char *MoveTimeEntry) {
    
    // Initialise variable that is used to judge the strength of signals
    unsigned int SensorResult[2]={0,0};
    unsigned int LeftFlag=0;
    unsigned int RightFlag=0;
    char buf[40]; // Buffer for characters for LCD
    unsigned int i=0;
    unsigned int n=0;
    unsigned char TimeAboveThreshold=0;
    // USERVARIABLE TOLERANCES
    // minimum signal strength required for sensor to be considered directly aimed at beacon
    const unsigned int DirectionFoundThreshold=3000;
    
    // Flip right before starting scan from left side
//    for (i=1; i<=(milliseconds>>1); i++) {
//        turnRight(mL,mR);
//        __delay_ms(1);
//        stop(mL,mR);
//    }
    // THIS CAN BE MADE BETTER
    turnLeft(mL,mR, 100);
    delay_tenth_s(5);
    stop(mL,mR);
    
    // Turn right slowly for scanning
    turnRight(mL,mR, 40);
    
    // Initialise Timer0 vales to 0
    TMR0L = 0;
    TMR0H = 0;
    T0CONbits.TMR0ON=1; // turn on timer0
    // This loop is turning Right, while scanning
    for (i=1; i<=milliseconds; i++) {
        
        // Scan Data
        SensorResult[0]=grabLeftIR();
        SensorResult[1]=grabRightIR();

        // Reset the timers to avoid same reading being picked up if there is
        // no signal.
        CAP1BUFH=0;
        CAP1BUFL=0;
        CAP2BUFH=0;
        CAP2BUFL=0;
        
// COMMENTED FOR EFFICIENCY
        // Output signal strength to LCD
        SendLCD(0b00000001,0); //Clear Display
        __delay_us(50); //Delay to let display clearing finish
        SendLCD(0b00000010,0); // move cursor to home
        __delay_ms(2);
        SetLine(1); //Set Line 1
        LCD_String("     ScanIR");
        SetLine(2); //Set Line 2, for signal strength readings
        sprintf(buf,"     %04d, %04d",SensorResult[0],SensorResult[1]);
        LCD_String(buf);
        
        if (RightFlag==0){
            if (SensorResult[1]>DirectionFoundThreshold) {
                RightFlag= (TMR0H<<8)+TMR0L;
            }
        }
        
        if (LeftFlag==0){
            if (SensorResult[0]>DirectionFoundThreshold) {
                LeftFlag=(TMR0H<<8)+TMR0L;
            }
        }
        
        // Increment counter if any of the IR sensors has seen the beacon
//        if ((LeftFlag>0)||(RightFlag>0)) {
//            TimeAboveThreshold++;
//        }
        
        if (LeftFlag>0) {  
            // Both Sensors have seen the beacon, travel back to
            // half the length of the FlagCounter and go!
            if (RightFlag>0) {
                TimeAboveThreshold = LeftFlag - RightFlag;
                TMR0L = 0; //Reset the timer
                TMR0H = 0;
                stop(mL,mR);
//                for (n=1; n<=(TimeAboveThreshold>>1); n++) {
                while (((TMR0H<<8)+TMR0L)<(TimeAboveThreshold>1)) {
//                    stop(mL,mR);
                    turnLeft(mL,mR, 100);
//                    __delay_ms(1);
//                    stop(mL,mR);
                }
                T0CONbits.TMR0ON=0; // Stop the timer
                stop(mL,mR);
                return 2; // Direction of bomb is directly ahead
            } else {
                // Signal was only found once, just go in that direction roughly
                T0CONbits.TMR0ON=0; // Stop the timer
                stop(mL,mR);
                return 2; // Direction of bomb is roughly ahead
            }     
        } 
    }
    
    // No clear signal found, rotate and move a bit and hope to find it!
    turnRight(mL,mR, 100);
    delay_tenth_s(2);
    stop(mL,mR);
    return -1; // No clear signal found
}