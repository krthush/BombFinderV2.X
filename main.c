#include <xc.h>
#include "dc_motor.h"
#include "RFID_Reader.h"
#include "Movement.h"
#include "IR_Reading.h"
#include "LCD.h"

#pragma config MCLRE=OFF, LVP=OFF, OSC = IRCIO, WDTEN = OFF //internal oscillator, WDT off

#define PWMcycle 1 //need to calculate this

volatile unsigned char ReceivedString[16]; //Global variable to read from RFID
volatile unsigned char i=0;
volatile unsigned char RFID_Read=0;
volatile signed char mode=0; // Robot mode - see switch case tree in main loop

// High priority interrupt routine
void interrupt low_priority InterruptHandlerLow ()
{
    if (PIR1bits.RCIF) {
        ReceivedString[i]=RCREG;
        RFID_Read=1;
        if (i==15){
            i=0;
        }else{
            i++;  
        }
        PIR1bits.RCIF=0; // clear the flag
    }
}

// Low priority interrupt routine for button 
// Switches between inert mode vs. other modes
void interrupt InterruptHandlerHigh () {
    if (INTCONbits.INT0IF) {
        if (mode==-1) { // If in inert mode
            // Start searching and then perform tasks   
            mode=1;
        } else if (mode==0) { // If in start up mode
            // Do nothing! Needs to wait till bot is started.
        } else { // If in any other mode...
            // Enter inert mode
            mode=-1;
        }
        // Small delay to de-bounce switch interrupt
        delay_tenth_s(2);
        INTCONbits.INT0IF=0; // clear the flag
    }
}

void main(void){
    
    //Initialise Variables
    unsigned char Message[10]; // Code on RFID Card
    unsigned char i=0; // Counter variable
    signed char DirectionFound=0; // Flag for if the robot has decided it knows where the bomb is
    int MoveTime[100]; // Array to store time spent on each type of movement
    char MoveType[100]; // Array to store movement types - 0 is forwards based 
    //on tenth-second delays, 1 is left/right based on timer, 2 is left/right 
    //based on tenth second delays
    char Move=0; // Move counter
    unsigned int SensorResult[2]={0,0};
    char buf[40]; // Buffer for characters for LCD
    // USERVARIABLE TOLERANCES
    unsigned char ScanAngle=60; //MAX VALUE: 255, P.S. This needs to be set slightly over 360 for spiral to work
    
    // Enable general interrupt stuff
    INTCONbits.GIEH=1; // Global Interrupt Enable bit
    INTCONbits.GIEL=1; // Peripheral/Low priority Interrupt Enable bit
    INTCONbits.PEIE=1; // Enable Peripheral  interrupts
    RCONbits.IPEN=1; // Enable interrupt priority
    
    // Interrupt registers for EUSART
    IPR1bits.RCIP=0; // Low Priority 
    PIE1bits.RCIE=1; // Enable interrupt on serial reception
    
    // Interrupt registers for button
    TRISCbits.RC3=1; //set RC3 pin to be an input pin to recognise the button press
    INTCONbits.INT0IE=1; // Enables the INT0 external interrupt
    
    // Clear interrupt flags
    PIR1bits.RC1IF=0;// clear EUSART interrupt flag
    INTCONbits.INT0IF=0;// clear flag on the button interrupt
    
    // Initialise Motor Structures
    struct DC_motor mL, mR; //declare 2 motor structures
    mL.power=0; //zero power to start
    mL.direction=1; //set default motor direction
    mL.dutyLowByte=(unsigned char *)(&PDC0L); //store address of PWM duty low byte
    mL.dutyHighByte=(unsigned char *)(&PDC0H); //store address of PWM duty high byte
    mL.dir_pin=0; //pin RB0/PWM0 controls direction
    mL.PWMperiod=199; //store PWMperiod for motor
    //same for motorR but different PWM registers and direction pin
    mR.power=0; //zero power to start
    mR.direction=1; //set default motor direction
    mR.dutyLowByte=(unsigned char *)(&PDC1L); //store address of PWM duty low byte
    mR.dutyHighByte=(unsigned char *)(&PDC1H); //store address of PWM duty high byte
    mR.dir_pin=2; //pin RB0/PWM0 controls direction
    mR.PWMperiod=199; //store PWMperiod for motor

    OSCCON = 0x72; //8MHz clock
    while(!OSCCONbits.IOFS); //wait until stable
    
    while(1){
       
       switch (mode) {
           case -1: //Inert Mode
               
                // Robot has finished start-up, and now ready to go!
                // Robot also enters this mode after successfully finishing the task.
                // If button is pressed while robot is performing, it will return to inert mode.
                // If button is pressed while robot is in inert mode, it will start performing.
                // Robot will also display IR values for easy calibration
                stop(&mL, &mR);
                RFID_Read=0;
                
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
                LCD_String("      Inert Mode");
                SetLine(2); //Set Line 2, for signal strength readings
                if (RFID_Read) {
                    LCD_String(Message);
                } else {
                    sprintf(buf,"      %04d, %04d",SensorResult[0],SensorResult[1]);
                    LCD_String(buf);
                }
                
               
                break;
               
           case 0 : //Start-up Mode
               
                //Initialise EVERYTHING
                initMotorPWM();  //setup PWM registers
                initTimer();
                initRFID();
                initLCD();
                initIR();
              
                enableSensor(0, 1); // DEBUG ONLY - enable sensors to test signals
                enableSensor(1, 1); // DEBUG ONLY - enable sensors to test signals
                
                // Small movement to signify initialise code has been successful
                fullSpeedAhead(&mL, &mR, 100);
                delay_tenth_s(1);
                
                mode=-1;  //TODO: Make mode change on button press
                
                break;
               
           case 1 : //Search Mode
               
                // TODO: do calibration routine here
                // Pseudo code:
                // if(!calibrated){calibrate()}
               
                SetLine(1); //Set Line 1
                LCD_String("Searching");
                               
                if (DirectionFound==-1) {
                    // Robot is completely lost, move a bit a hope to find it.
                    // PLEASE NOTE: this movement in combination with the
                    // rotation in ScanWithRange causes the robot to spiral 
                    // outwards such that it will ALWAYS get close enough to signal
                    Move++;
                    MoveType[Move]=0;
                    MoveTime[Move]=6;
                    fullSpeedAhead(&mL, &mR, 100);
                    delay_tenth_s(6);
                    stop(&mL,&mR);
                    DirectionFound=0;
                } else if (DirectionFound==0) {
                    // Scans a wide range if it's unsure about direction
                    Move++;
                    DirectionFound=ScanWithRange(&mL, &mR, ScanAngle,
                            &MoveTime[Move], &RFID_Read);
                    MoveType[Move]=1;
                } else if (DirectionFound==1) {
                     // Keeps direction and just scans, robot thinks it's close
                    DirectionFound=ScanIR(&mL, &mR);
                } else if (DirectionFound==2) {
                     // Robot thinks its on track, switch to move mode
                     mode=2;
                     MoveType[Move]=1;
                }
               
                break;
               
            case 2 : // Go forward mode
                // Bot knows direction of bomb.
                // Move forward until RFID read and verified or a certain time
                // has elapsed.

                if (RFID_Read) {
                    stop(&mL, &mR);
                    if (ReceivedString[0]==0x02 & ReceivedString[15]==0x03){ //If we have a valid ASCII signal
                        if (VerifySignal(ReceivedString)){ //and if the checksum is correct
                            //Put the RFID data into the Message variable
                            for (i=0; i<10; i++){
                                Message[i] = ReceivedString[i+1]; 
                            }
//                             LCDString(Message); //Display code on LCD
                            //Clear the received string 
                            for (i=0; i<16; i++) {
                                ReceivedString[i]=0;
                            }
                            mode=3; //Return to home!

                        } else { //If the signal doesn't check out
                            fullSpeedBack(&mL,&mR, 100); //Go back a bit then stop
                            delay_tenth_s(5);
                            stop(&mL,&mR);
                            fullSpeedAhead(&mL,&mR, 100); //Try again
                        }  
                    }
                } else {
                    DirectionFound=1;
                    mode=1;
                    // Bot needs to head for the bomb
                    fullSpeedAhead(&mL,&mR, 100);
                    delay_tenth_s(5);
                    MoveType[Move] = 0;
                    MoveTime[Move] += 5;
                }
                
                break;
               
            case 3 : //Return Mode
                //Return to original position using MoveType and MoveTime
                
                SetLine(1); //Set Line 1
                LCD_String(Message);
                SetLine(2);
                LCD_String("Going Home");
                stop(&mL,&mR);
                
                for (Move=Move; Move>=0; Move--) { //Go backwards along the moves
                    if (MoveType[Move]==0) { //If move was forwards
                        fullSpeedBack(&mL,&mR);
                        delay_tenth_s(MoveTime[Move]);
                    } else if (MoveType[Move]==1) { //If move was left/right
                        if (MoveTime[Move]>0) { //If left turn
                            turnRight(&mL,&mR);
                            delay_tenth_s(MoveTime[Move]);
                        } else {
                            turnLeft(&mL,&mR);
                            delay_tenth_s(MoveTime[Move]);
                        }
                    }
                }
                mode=-1;

                break;
       }      
   }
}
