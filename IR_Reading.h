/* 
 * File:   IR_Reading.h
 * Author: guilds
 *
 * Created on 12 March 2018, 16:37
 */

#ifndef IR_READING_H
#define	IR_READING_H
#include <xc.h>

// PLEASE NOTE: Decided to not use CCP module, using motion capture module instead for IR tracking
// void getTimerVal();

void initIR(void);
unsigned int grabRightIR(void);
unsigned int grabLeftIR(void);
void enableSensor(char sensor, char status); //Turns IR sensor on or off




#endif	/* IR_READING_H */

