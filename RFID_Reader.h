/* 
 * File:   RFID_Reader.h
 * Author: tr514
 *
 * Created on 06 March 2018, 15:51
 */

#ifndef RFID_READER_H
#define	RFID_READER_H
#include <xc.h>

// Initialize bits for RFID readings
void initRFID(void);

 // Checks RFID Checksum.
unsigned char VerifySignal(unsigned char *Signal);

#endif	/* RFID_READER_H */

