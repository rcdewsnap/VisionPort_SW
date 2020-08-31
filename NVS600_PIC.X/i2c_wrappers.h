/* 
 * File:   i2c_wrappers.h
 * Author: vivek
 *
 * Created on May 30, 2018, 9:29 PM
 */


#ifndef I2C_WRAPPERS_H
#define	 I2C_WRAPPERS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "mcc_generated_files/mcc.h"
//#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/i2c2.h"

void i2c_sendcommand_dN(int busNum,int DEVADD, uint16_t length, uint8_t *databuffer);

/* send a byte over i2c */
void i2c_sendcommand_a1d1(int BusNum, int DEVADD, int REGADD, int DATABYTE);

//
// Sends a word on the data bus, note:
//  it sends bytes to set the address.
//
void i2c_sendcommand_a1d2(int busNum, int DEVADD, int REGADD, int DATAW);

// This function is for I2C transactions with a two byte address and one byte data
void i2c_sendcommand_a2d1(int busNum, int DEVADD, int REGADD, int DATAW);

// This function is for I2C transactions with a two byte address and two byte data
void i2c_sendcommand_a2d2(int busNum, int DEVADD, int REGADD, int DATAW);

void i2c_sendcommand_dN(int busNum,int DEVADD, uint16_t length, uint8_t *databuffer);

/* send a byte over i2c */
void i2c_sendcommand_a1d1(int BusNum, int DEVADD, int REGADD, int DATABYTE);

//
// Sends a word on the data bus, note:
//  it sends bytes to set the address.
//
void i2c_sendcommand_a1d2(int busNum, int DEVADD, int REGADD, int DATAW);

// This function is for I2C transactions with a two byte address and one byte data
void i2c_sendcommand_a2d1(int busNum, int DEVADD, int REGADD, int DATAW);

// This function is for I2C transactions with a two byte address and two byte data
void i2c_sendcommand_a2d2(int busNum, int DEVADD, int REGADD, int DATAW);

// This function is to read with one byte address and one byte data
// We encode the ack onto this by adding it to the high byte
// If no ACK is received then we make the high byte of returned data FF
// This leaves the code functional for older usage, and allows ack to be
// returned for newer function calls
int i2c_readdata_a1d1(int busNum, int DEVADD, int REGADD);

// This function is to read with one byte address and two bytes data
int i2c_readdata_a1d2(int busNum, int DEVADD, int REGADD);

// This function is to read with one byte address and N bytes data
void i2c_readdata_a1dN(int busNum, int DEVADD, int REGADD, int N, unsigned int buffer[]);

void i2c_readdata_dN(int busNum, int DEVADD, uint8_t length, uint8_t *pData);

// This function is to read with two byte address and one byte data
int i2c_readdata_a2d1(int busNum, int DEVADD, int REGADD);

// This function is to read with two byte address and two bytes data
int i2c_readdata_a2d2(int busNum, int DEVADD, int REGADD);

// Two bytes address, upto 0xFFFF bytes data
void i2c_readdata_a2dn(int busNum, int DEVADD, int REGADD, uint8_t *databuffer, uint16_t readlength);


#ifdef	__cplusplus
}
#endif

#endif	/* I2C_WRAPPERS_H */

