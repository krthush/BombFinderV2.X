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

// Function to toggle enable bit on then off.
void initLCD(void);

// Function to send four bits to the LCD.
void E_TOG(void);

// Function to send data/commands over a 4bit interface.
void LCDout(unsigned char number);

// Function to initialize LCD display.
void SendLCD(unsigned char Byte, char type);

// Function to put cursor to start of line.
void SetLine (char line);

// Function that takes a string and sends it to be displayed on the LCD.
void LCD_String(char *string);

#endif	/* LCD_H */

