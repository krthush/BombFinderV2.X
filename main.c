#include <xc.h>
#include "dc_motor.h"
#include "RFID_Reader.h"
#include "Movement.h"
#include "IR_Reading.h"
#include "LCD.h"

#pragma config MCLRE=OFF, LVP=OFF, OSC = IRCIO, WDTEN = OFF // Internal 
// oscillator, WDT off.

#define PWMcycle 1 // This has been calculated.

//----------------------------------------------------------------------------//
//Initialize Global Variables//
//----------------------------------------------------------------------------//
volatile unsigned char ReceivedString[16]; // Global variable to read from RFID.
volatile unsigned char i=0; // For loop iterator.
volatile unsigned char RFID_Read=0; // To know if RFID has been read.
volatile signed char mode=0; // Robot mode - see switch case tree in main loop.
volatile unsigned int millis=0; // Approximate millisecond timer,
// see use of time0 in initTimer in Movement source file.

//----------------------------------------------------------------------------//
//Interrupt Routines//
//----------------------------------------------------------------------------//

// Low priority interrupt routine for RFID reader.
// Confirms when RFID is read and stores the given serial string.
void interrupt low_priority InterruptHandlerLow ()
{
    if (PIR1bits.RCIF) { // If the serial receive interrupt is active
        ReceivedString[i]=RCREG; // Read the data byte.
        RFID_Read=1; // Robot know it has reached RFID now.
        if (i==15){ // If end of signal is reached
            i=0; // Reset the counter to go again if needed.
        }else{
            i++; // Increment the counter so the next character can be filled.  
        }
        PIR1bits.RCIF=0; // Clear the flag.
    }
}

// High priority interrupt routine for button and timer overflow.
// Switches between inert mode vs. other modes and increments a timer counter.
void interrupt InterruptHandlerHigh () {
    if (INTCONbits.INT0IF) { // If button pressed
        if (mode==-1) { // If in inert mode
            // Start searching and then perform tasks.   
            mode=1;
        } else if (mode==0) { // If in start up mode
            // Do nothing! Needs to wait till bot is started.
        } else { // If in any other mode
            // Enter inert mode.
            mode=-1;
        }
        // Small delay to de-bounce switch interrupt.
        delay_tenth_s(2);
        INTCONbits.INT0IF=0; // Clear the flag.
    }
    // Also handle the timer overflowing.
    if (INTCONbits.TMR0IF) { // If timer has overflowed
        millis++; // Increment millisecond counter.
        INTCONbits.TMR0IF = 0; // Clear the flag.
    }
}

//----------------------------------------------------------------------------//
//Main Program Loop//
//----------------------------------------------------------------------------//

void main(void){
    
    //------------------------------------------------------------------------//
    //Initialize Main Loop Variables//
    //------------------------------------------------------------------------//

    unsigned char Message[10]; // Code on RFID Card.
    unsigned char i=0; // Counter variable.
    signed char DirectionFound=0; // Flag for if the direction of the bomb is
    // known.
    signed int MoveTime[50] = { 0 }; // Array to store time spent on each type 
    // of movement.
    // For left/right, left is defined as positive. For forwards/backwards,
    // forwards is positive.
    // Reduce this as well as MoveType if memory is needed.
    unsigned char MoveType[50] = { 0 }; // Array to store movement types:
        // 0 is forwards based on tenth-second delays,
        // 1 is left/right based on timer,
        // 2 is left/right based on tenth second delays.
    signed char Move=0; // Move counter - signed to prevent overflow when 0 
    // reached.
    unsigned int SensorResult[2]={0,0}; // IR readings left and right.
    char buf[40]; // Buffer for characters for LCD.
    // USERVARIABLE TOLERANCES
    unsigned char ScanAngle=60; // MAX VALUE: 255, P.S. This currently 
    // determines the number of loops the bot scans while spinning.
    // Also this should be calibrated for the spiral effect -> if the robot
    // never finds a signal it spirals out from its location making sure it 
    // gets close enough to find it.
    const unsigned char MotorPower=40; // Adjusts the speed of the turning
    // while scanning, currently seems to lose clarity past 43ish.
    
    //------------------------------------------------------------------------//
    //Initialize Interrupts//
    //------------------------------------------------------------------------//
    
    INTCONbits.GIEH=1; // Global Interrupt Enable bit.
    INTCONbits.GIEL=1; // Peripheral/Low priority Interrupt Enable bit.
    INTCONbits.PEIE=1; // Enable Peripheral  interrupts.
    RCONbits.IPEN=1; // Enable interrupt priority.
    
    // Interrupt registers for EUSART
    IPR1bits.RCIP=0; // Low Priority. 
    PIE1bits.RCIE=1; // Enable interrupt on serial reception.
    
    // Interrupt registers for button
    TRISCbits.RC3=1; // Set RC3 pin to be an input pin to recognize the button 
    // press.
    INTCONbits.INT0IE=1; // Enables the INT0 external interrupt.
    
    // Clear interrupt flags
    PIR1bits.RC1IF=0;// Clear EUSART interrupt flag.
    INTCONbits.INT0IF=0;// Clear flag on the button interrupt.
    
    //------------------------------------------------------------------------//
    //Initialize Motor Structures//
    //------------------------------------------------------------------------//
    
    struct DC_motor mL, mR; // Declare 2 motor structures.
    mL.power=0; // Zero power to start.
    mL.direction=1; // Set default motor direction.
    mL.dutyLowByte=(unsigned char *)(&PDC0L); // Store address of PWM duty low 
    // byte.
    mL.dutyHighByte=(unsigned char *)(&PDC0H); // Store address of PWM duty high
    // byte.
    mL.dir_pin=0; // Pin RB0/PWM0 controls direction.
    mL.PWMperiod=199; // Store PWMperiod for motor.
    
    // Same for motorR but different PWM registers and direction pin.
    mR.power=0; // Zero power to start.
    mR.direction=1; // Set default motor direction.
    mR.dutyLowByte=(unsigned char *)(&PDC1L); // Store address of PWM duty low
    // byte.
    mR.dutyHighByte=(unsigned char *)(&PDC1H); // Store address of PWM duty high
    // byte.
    mR.dir_pin=2; // Pin RB0/PWM0 controls direction.
    mR.PWMperiod=199; // Store PWMperiod for motor.
    
    //------------------------------------------------------------------------//
    //CPU clock speed//
    //------------------------------------------------------------------------//
    
    OSCCON = 0b1110010; // 8MHz clock
    while(!OSCCONbits.IOFS); // Wait until stable.
    
    //------------------------------------------------------------------------//
    //Post Start Up Loop//
    //------------------------------------------------------------------------//
    
    while(1){ // Loop forever
       
       switch (mode) {

           case -1:
           //Inert Mode//
               
                // Robot has finished start-up, and now ready to go!
                // Robot also enters this mode after successfully finishing the 
                // task.
                // If button is pressed while robot is performing, it will 
                // return to inert mode.
                // If button is pressed while robot is in inert mode, it will
                // start performing.
                // Robot will also display IR values for easy calibration.
               
                stop(&mL, &mR); // Make sure we're stationary in this mode.
                
                // Reset Crucial Starting Variables
                RFID_Read=0;
                Move=0;
                
                // Scan Data
                SensorResult[0]=grabLeftIR();
                SensorResult[1]=grabRightIR();

<<<<<<< HEAD
                // Reset the timers to avoid same reading being picked up if 
                // there is no signal.
=======
                // Reset the CAP buffers to avoid same reading being picked up 
                // if there is no signal.
>>>>>>> ba4edfa72f828c7c56b5b47465ab778cc75fe6ae
                CAP1BUFH=0;
                CAP1BUFL=0;
                CAP2BUFH=0;
                CAP2BUFL=0;       

                // Output signal strength to LCD
                SendLCD(0b00000001,0); // Clear Display.
                __delay_us(50); // Delay to let display clearing finish.
                SendLCD(0b00000010,0); // Move cursor to home.
                __delay_ms(2);
                SetLine(1); // Set Line 1.
                if (Message[0]==0) { // If we haven't got the RFID.
                    LCD_String("      Inert Mode");
                } else { 
                    LCD_String(Message); // Display the RFID data for
                    // inspection.
                }
                
                // Display current IR readings on line 2
                SetLine(2); // Set Line 2, for signal strength readings.
                sprintf(buf,"      %04d, %04d",SensorResult[0],SensorResult[1]);
                 
                break;
               
            case 0 : 
            //Start-up Mode//
                
                // Initialize EVERYTHING
                
                initMotorPWM();  // Setup PWM registers.
                initTimer(); // Setup Timer0.
                initRFID(); // Setup RFID pins.
                initLCD(); // Initialize the LCD screen.
                initIR(); // Initialize the IR sensors.
              
                enableSensor(0, 1); // Enable sensors to test signals
                enableSensor(1, 1); // Enable sensors to test signals
                
                // Small movement to signify initialize code has been successful
                fullSpeed(&mL, &mR, 100);
                delay_tenth_s(1);
                
                mode=-1;  // Go immediately into inert mode.
                
                break;
               
            case 1 : 
            //Search Mode//
               
                SetLine(1); //Set LCD Line 1
                LCD_String("Searching");
                
<<<<<<< HEAD
                // Does different things depending on the sensor readings - if 
                // it hasn't got a strong reading it scans clockwise slowly.
                // If the beacon is in front it moves forward, checking as it 
                // goes.
                // If it's totally lost it starts spiraling outward so it 
                // eventually finds the beacon.
=======
                // Does different things depending on the sensor readings - 
                // if it hasn't got a strong reading it scans clockwise slowly
                // If the beacon is in front, it moves forward, checking as it goes
                // If it's totally lost it starts spiralling outward so it eventually
                // finds the beacon
>>>>>>> ba4edfa72f828c7c56b5b47465ab778cc75fe6ae
                if (DirectionFound==-1) {
                    // Robot is completely lost, move a bit a hope to find it.
                    // PLEASE NOTE: this movement in combination with the
                    // rotation in ScanWithRange causes the robot to spiral 
                    // outwards such that it will ALWAYS get close enough to
                    // signal.
                    MoveType[Move]=0; //Store the upcoming move in the buffers
                    MoveTime[Move]=6;
                    Move++;
                    fullSpeed(&mL, &mR, 100);
                    delay_tenth_s(6);
                    stop(&mL,&mR);
                    DirectionFound=0; //Go back to ScanWithRange
                } else if (DirectionFound==0) { //If it hasn't found the beacon
                    // Scans a wide range if it's unsure about direction
                    DirectionFound=ScanWithRange(&mL, &mR, ScanAngle,
                            &MoveTime, &Move, &MoveType, &RFID_Read, &millis);
                } else if (DirectionFound==1) {
                     // Keeps direction and just scans, robot thinks it's close
                    DirectionFound=ScanIR(&mL, &mR);
                } else if (DirectionFound==2) {
                     // Robot thinks it's on track
                     mode=2; //switch to move mode
                }
               
                break;
               
            case 2 :
            //Forward Motion Mode//
                
                // Bot knows direction of bomb.
                // Move forward until RFID read and verified or a certain time
                // has elapsed.

                if (RFID_Read) { // If the RFID interrupt has fired
                    stop(&mL, &mR);
<<<<<<< HEAD
                    if (ReceivedString[0]==0x02 & ReceivedString[15]==0x03){ 
                        // If we have a valid ASCII signal
                        if (VerifySignal(&ReceivedString)){
                            // And if the checksum is correct
                            // Put the RFID data into the Message variable.
=======
                    //If we have a valid ASCII signal...
                    if (ReceivedString[0]==0x02 & ReceivedString[15]==0x03){
                        // ...and if the checksum is correct...
                        if (VerifySignal(&ReceivedString)){ 
                            //...put the RFID data into the Message variable
>>>>>>> ba4edfa72f828c7c56b5b47465ab778cc75fe6ae
                            for (i=0; i<10; i++){
                                Message[i] = ReceivedString[i+1]; 
                            }
                            // Clear the received string.
                            for (i=0; i<16; i++) {
                                ReceivedString[i]=0;
                            }
                            mode=3; // Return to home!

                        } else { // If the signal doesn't check out.
                            fullSpeedBack(&mL,&mR, 100); // Go back a bit.
                            delay_tenth_s(5);
                            stop(&mL,&mR); // Then stop.
                            fullSpeed(&mL,&mR, 100); // Try again.
                            delay_tenth_s(5);
                            stop(&mL,&mR);
                        }  
                    }
                } else {
                    DirectionFound=1; // It's found the direction of the beacon.
                    mode=1; // Return to search mode for ScanIR().
                    // Bot needs to start heading for the bomb.
                    fullSpeed(&mL,&mR, 100);
                    delay_tenth_s(1);
                    //Store the movement forward
                    MoveType[Move] = 0;
                    MoveTime[Move] = 5;
                    Move++;
                }
                
                break;
               
            case 3 :
            //Return Mode//
                
<<<<<<< HEAD
                // Return to original position using MoveType and MoveTime
                SetLine(1); // Set Line 1.
                LCD_String(Message);
=======
                SetLine(1); //Set Line 1
                LCD_String(Message); //Display the RFID card data
>>>>>>> ba4edfa72f828c7c56b5b47465ab778cc75fe6ae
                SetLine(2);
                LCD_String("Going Home");
                
                for (Move; Move>=0; Move--) { // Go backwards along the moves.
                    stop(&mL,&mR);
                    if (MoveType[Move]==0) { // If move was forwards
                        fullSpeedBack(&mL,&mR,100);
                        delay_tenth_s(MoveTime[Move]);
                    } else if (MoveType[Move]==1) { // If timer left/right
                        T0CONbits.TMR0ON=0; // Stop the timer.
                        TMR0L = 0; // Reset the timer.
                        TMR0H = 0;
                        millis = 0;
                        if (MoveTime[Move]>0) { // If left turn
                            T0CONbits.TMR0ON=1; // Start the timer.
                            turnRight(&mL,&mR,MotorPower);
                            while (millis<MoveTime[Move]); // Delay 
                            // until it's turned as far as it did originally.
                            T0CONbits.TMR0ON=0; // Stop the timer.
                        } else { //If right turn
                            T0CONbits.TMR0ON=1; // Start the timer.
                            turnLeft(&mL,&mR,MotorPower);
                            while (millis<(-MoveTime[Move])); // Delay 
                            // until it's turned as far as it did originally.
                            T0CONbits.TMR0ON=0; // Stop the timer.
                        }
                    } else if (MoveType[Move]==2) { // If 0.1s left/right
                        if (MoveTime[Move]>0) { // If left turn
                            turnRight(&mL,&mR,78); // USERVARIABLE POWER
                            // power of 78% calibrated to account for differences
                            // in left/right motors.
                            delay_tenth_s(MoveTime[Move]);
                        } else { // If right turn
                            turnLeft(&mL,&mR,100);
                            delay_tenth_s(MoveTime[Move]);
                        }
                    }
                    if (mode==-1) { // Check if button has been pressed,
                        break; // exit the loop if it has.
                    }
                }
                stop(&mL,&mR);
                mode=-1; // Return to inert mode.

                break;
       }      
   }
}
