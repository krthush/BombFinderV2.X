/* 
 * File:   LCD.h
 * Author: tr514
 *
 * Created on 06 March 2018, 15:39
 */
#ifndef LCD_H
#define	LCD_H
#define _XTAL_FREQ 8000000

#include <string.h>
#include <stdio.h>
#pragma config OSC = IRCIO, WDTEN=OFF // internal oscillator

// define bits
#define LCD_E LATCbits.LATC0
#define LCD_DB4 LATCbits.LATC1
#define LCD_DB5 LATCbits.LATC2
#define LCD_DB6 LATDbits.LATD0
#define LCD_DB7 LATDbits.LATD1
#define LCD_RS LATAbits.LATA6

void initLCD(void);
void E_TOG(void);
void LCDout(unsigned char number);
void SendLCD(unsigned char Byte, char type);
void SetLine (char line);
void LCD_String(char *string);

#endif	/* LCD_H */

