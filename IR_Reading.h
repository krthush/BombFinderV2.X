/* 
 * File:   IR_Reading.h
 * Author: guilds
 *
 * Created on 12 March 2018, 16:37
 */

#ifndef IR_READING_H
#define	IR_READING_H
#include <xc.h>

// Initializes IR sensor registers (including CAP module).
void initIR(void);

// Gives right IR sensor value.
unsigned int grabRightIR(void);

// Gives right IR sensor value.
unsigned int grabLeftIR(void);

// Turns IR sensor on or off.
void enableSensor(char sensor, char status); 

#endif	/* IR_READING_H */

