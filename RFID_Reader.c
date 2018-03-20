#include "RFID_Reader.h"
#include <xc.h>
#include <string.h>
#pragma config OSC = IRCIO
#define _XTAL_FREQ 8000000

void initRFID(void){
    TRISC = TRISC | 0b11000000; //set data direction registers
                        //both need to be 1 even though RC6
                        //is an output, check the datasheet!
        
    SPBRG=205; //set baud rate to 9600
    SPBRGH=0;
    BAUDCONbits.BRG16=1; //set baud rate scaling to 16 bit mode
    TXSTAbits.BRGH=1; //high baud rate select bit
    RCSTAbits.CREN=1; //continous receive mode
    RCSTAbits.SPEN=1; //enable serial port, other settings default
    TXSTAbits.SYNC=0; //Asynchronous
    RCSTAbits.RX9=0; //8-bit reception  
}

char getCharSerial(void){
while (!PIR1bits.RCIF); //wait for the data to arrive
    return RCREG; //return byte in RCREG
}

//String can be divided into 5 pairs of Hex numbers, e.g.
//
//123456789A -> 0x12 | 0x34 | 0x56 | 0x78 | 0x9A
//	=     Hex1 | Hex2 | Hex3 | Hex4 | Hex5
//The checksum is calculated by taking the XOR of each of these
//numbers with the result of those to is left, i.e.
//
//Step1 = XOR(Hex1,Hex2)
//Step2 = XOR(Step1, Hex3)
//etc...
//
//For our RFID card the checksum should be 0x60
unsigned char VerifySignal(unsigned char *ReceivedString){
    unsigned char i=0;
    unsigned char xorOutput=0;
    unsigned char translation[12];
    unsigned char ASCIICheckSum=0;
    memset(translation,0,12);
    
    //Decode the ASCII into hex - ASCII 0 == 0x30
    for(i=0;i<12;i++){
        if(ReceivedString[i]>'F'){ 
            //If we have a character greater than F it's not valid hex
            return 0; //So the signal is invalid
        } else if (ReceivedString[i]>='A') { //Treat anything above A differently
            translation[i]=ReceivedString[i]-'A'+10;            
        } else if (ReceivedString[i]>='0') { // anything else is just the 
            // difference between the value and the one for 0
            translation[i]=ReceivedString[i]-'0';
        }
    }
    
    // CHECKSUM - repeatedly XOR the hex bytes
    for (i=0;i<5;i++) {
        xorOutput^=(translation[2*i]<<4)+translation[2*i+1]; 
    }
    
    // And the checksum from the received signal
    ASCIICheckSum=(translation[10]<<4)+(translation[11]&0xF);
    
    if (ASCIICheckSum==xorOutput){ // If everything matches...
        return 1; // It worked!
    }
    
    return 0; // Otherwise it failed
    

//    //First run through - XOR first two hex bytes
//    hexByte = (RecievedString[3]<<8) + RecievedString[4];
//    checksum = ((RecievedString[1]<<8) + RecievedString[2]) ^ hexByte; //First 2 chars XOR second 2
//
//    //Loop through, XORing the previous result with the next hex byte in turn
////    for (i=5; i<10; i+=2){
////        hexByte = (Signal[i]<<8) + Signal[i+1];
////        checksum = checksum ^ strtol(hexByte, &ptr, 16);
////    }
//
////    if ((checksum==((Signal[11]<<8)+Signal[12]))){
//        return 1;
////    } else{
////        return 0;
////    }
}
