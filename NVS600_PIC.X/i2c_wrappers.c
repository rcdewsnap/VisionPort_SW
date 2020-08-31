/* 
 * File:   i2c_wrappers.c
 * Author: vivek
 *
 * Created on May 30, 2018, 9:29 PM
 */


#include "i2c_wrappers.h"
#include "main.h"


void i2c_sendcommand_dN(int busNum,int DEVADD, uint16_t length, uint8_t *databuffer)
{
    
    uint16_t slaveTimeOut;
    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

    uint16_t devaddr = DEVADD;
    
    // Send command
    I2C2_MasterWrite(databuffer,
            length,
            devaddr,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }
}


/* send a byte over i2c */
// BusNum is no longer used, this is hard tied to bus 2
void i2c_sendcommand_a1d1(int BusNum, int DEVADD, int REGADD, int DATABYTE)
{

    uint16_t slaveTimeOut = 0;

    uint8_t dbyte = DATABYTE;
    uint8_t regaddr = REGADD;
    uint16_t devaddr = DEVADD;
    
    uint8_t SourceData[32] = {regaddr, dbyte};     
    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;
    
    // Send command
    I2C2_MasterWrite(SourceData,
            0x02,
            devaddr,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }        
}

//
// Sends a word on the data bus, note:
//  it sends bytes to set the address.
//
void i2c_sendcommand_a1d2(int busNum, int DEVADD, int REGADD, int DATAW)
{

    uint16_t slaveTimeOut;

    uint8_t dbytelow = (DATAW&0x00FF);
    uint8_t dbytehigh = (DATAW>>8)&0x00FF;
    uint8_t regaddr = REGADD;
    uint16_t devaddr = DEVADD;

    // If this is a write to the FPGA then reverse byte order of the data
    if(devaddr == I2C_FPGA_W)
    {
        dbytelow = dbytehigh;
        dbytehigh = DATAW&0x00FF;
    }

    // Build the array to send
    uint8_t SourceData[32] = {regaddr, dbytelow, dbytehigh}; 
    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;
    
    // Send command
    I2C2_MasterWrite(SourceData,
            0x03,
            devaddr,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }
}

// This function is for I2C transactions with a two byte address and one byte data
void i2c_sendcommand_a2d1(int busNum, int DEVADD, int REGADD, int DATAW)
{

    uint16_t slaveTimeOut;

    uint8_t dbytelow = (DATAW&0x00FF);    
    uint8_t regaddrlow = (REGADD&0x00FF);
    uint8_t regaddrhigh = ((REGADD>>8)&0x00FF);
    uint16_t devaddr = DEVADD;
    
    uint8_t SourceData[32] = {regaddrhigh, regaddrlow, dbytelow}; 
    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;
    
    // Send command
    I2C2_MasterWrite(SourceData,
            0x03,
            devaddr,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }
}

// This function is for I2C transactions with a two byte address and two byte data
void i2c_sendcommand_a2d2(int busNum, int DEVADD, int REGADD, int DATAW)
{

    uint16_t slaveTimeOut;

    uint8_t dbytelow = (DATAW&0x00FF);    
    uint8_t dbytehigh = ((DATAW>>8)&0x00FF);
    uint8_t regaddrlow = (REGADD&0x00FF);
    uint8_t regaddrhigh = ((REGADD>>8)&0x00FF);
    uint16_t devaddr = DEVADD;
    
    uint8_t SourceData[32] = {regaddrhigh, regaddrlow, dbytehigh, dbytelow}; 
    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;
    
    // Send command
    I2C2_MasterWrite(SourceData,
            0x04,
            devaddr,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }
}


// This function is to read with one byte address and one byte data
// We encode the ack onto this by adding it to the high byte
// If no ACK is received then we make the high byte of returned data FF
// This leaves the code functional for older usage, and allows ack to be
// returned for newer function calls
int i2c_readdata_a1d1(int busNum, int DEVADD, int REGADD)
{
    uint16_t slaveTimeOut = 0;
    
    uint8_t SourceData[2] = {REGADD, 0x00}; // Only Lower byte is used
    uint8_t pResponse[2] = {0x00, 0x00};  // Only Lower byte is used
    uint8_t ResponseLen = 0x01;

    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

    // Send command
    I2C2_MasterWrite(SourceData,
            0x01,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }

    // Now read bytes
    I2C2_MasterRead(pResponse,
            ResponseLen,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            return (0);
        else
            slaveTimeOut++;
    }
    
    return(pResponse[0]);
    
}

// This function is to read with one byte address and two bytes data
int i2c_readdata_a1d2(int busNum, int DEVADD, int REGADD)
{
    uint16_t slaveTimeOut = 0;
    
    int temp;
    
    uint8_t SourceData[2] = {REGADD, 0x00}; // Only Lower byte is used
    uint8_t pResponse[2] = {0x00, 0x00};  // Only Lower byte is used
    uint8_t ResponseLen = 0x02;

    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

    // Send command
    I2C2_MasterWrite(SourceData,
            0x01,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }

//            do_ms_delay(ReadDelay);

    // Now read bytes
    I2C2_MasterRead(pResponse,
            ResponseLen,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            return (0);
        else
            slaveTimeOut++;
    }
    
    if(DEVADD == I2C_FPGA_W)        
        temp = (pResponse[0]<<8) + pResponse[1]; // Reverse byte order for FPGA
    else
        temp = (pResponse[1]<<8) + pResponse[0];
    return(temp);
    
}

// This function is to read with two byte address and one byte data
int i2c_readdata_a2d1(int busNum, int DEVADD, int REGADD)
{
    uint16_t slaveTimeOut = 0;
    
    uint8_t SourceData[2] = {(REGADD>>8), (REGADD&0xFF)}; // Separate bytes out of INT
    uint8_t pResponse[2] = {0x00, 0x00};  // Only Lower byte is used
    uint8_t ResponseLen = 0x01;

    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

        // Send command
        I2C2_MasterWrite(SourceData,
                0x01,
                DEVADD,
                &status2);

        // wait for the message to be sent or status has changed.
        while (status2 == I2C2_MESSAGE_PENDING) {
            // add some delay here
            do_ms_delay(1);

            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }

//            do_ms_delay(ReadDelay);

        // Now read bytes
        I2C2_MasterRead(pResponse,
                ResponseLen,
                DEVADD,
                &status2);

        // wait for the message to be sent or status has changed.
        while (status2 == I2C2_MESSAGE_PENDING) {
            // add some delay here
            do_ms_delay(1);

            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
                return (0);
            else
                slaveTimeOut++;
    }

    return(pResponse[0]);
}

// This function is to read with two byte address and two bytes data
int i2c_readdata_a2d2(int busNum, int DEVADD, int REGADD)
{
    uint16_t slaveTimeOut = 0;
    
    int temp;
    
    uint8_t SourceData[2] = {(REGADD>>8), (REGADD&0xFF)}; // Separate bytes out of INT
    uint8_t pResponse[2] = {0x00, 0x00};  // Only Lower byte is used
    uint8_t ResponseLen = 0x02;

    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

    // Send command
    I2C2_MasterWrite(SourceData,
            0x02,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }

//            do_ms_delay(ReadDelay);

    // Now read bytes
    I2C2_MasterRead(pResponse,
            ResponseLen,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            return (0);
        else
            slaveTimeOut++;
    }
    
    temp = (pResponse[0]<<8) + pResponse[1];
    return(temp);
}

// This function is to read with two byte address and two bytes data
void i2c_readdata_a2dn(int busNum, int DEVADD, int REGADD, uint8_t *databuffer, uint16_t readlength)
{
    uint16_t slaveTimeOut = 0;
    
    
    uint8_t SourceData[2] = {(REGADD>>8), (REGADD&0xFF)}; // Separate bytes out of INT

    I2C2_MESSAGE_STATUS status2 = I2C2_MESSAGE_PENDING;

    // Send command
    I2C2_MasterWrite(SourceData,
            0x02,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            break;
        else
            slaveTimeOut++;
    }

//            do_ms_delay(ReadDelay);

    // Now read bytes
    I2C2_MasterRead(databuffer,
            readlength,
            DEVADD,
            &status2);

    // wait for the message to be sent or status has changed.
    while (status2 == I2C2_MESSAGE_PENDING) {
        // add some delay here
        do_ms_delay(1);

        // timeout checking
        // check for max retry and skip this byte
        if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
            return;
        else
            slaveTimeOut++;
    }
    
}