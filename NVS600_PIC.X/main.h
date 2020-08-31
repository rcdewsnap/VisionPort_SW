/* 
 * File:   main.h
 * Author: vivek
 *
 * Created on March 20, 2019, 9:29 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

//#define DESER_TEST_MODE 1 // Uncomment this to use without camera heads, only DS90UB954 test patterns on both A and B outputs
#define MIPI_TEST_UART_SW_ENABLE  1 // Uncomment this to enable MIPI side test pattern switch   

#ifdef DESER_TEST_MODE
    #define PIC_MAJ_VERSION 0xAB // XX.yy  Bogus Number for Deser Test
    #define PIC_MIN_VERSION 0xBA // xx.YY
#else
    #define PIC_MAJ_VERSION 0x00 // XX.yy  This is the real version number
    #define PIC_MIN_VERSION 0x20 // xx.YY  This is the real version number   
#endif
    

#define PIC_VERSION ((PIC_MAJ_VERSION<<8) + PIC_MIN_VERSION)
    
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mcc_generated_files/mcc.h"
#include "helper_functions.h"
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/spi2.h"

    
//#define I2C_QUIET_MODE 1  // Makes the debug LEDs not flash so there is less i2c traffic
//#define UART_QUIET_MODE 1 // Uncomment this to disable UART messages
#define UART_VERBOSITY 2 // Lower numbers mean less messages. 0=None; 3=Everything
    
#define MIPI_LANES  4 // Set 2 or 4 MIPI lanes here

// These are 7 bit device addresses. R/Wn bit will be appended.
// First list I2C1 devices
#define I2C_FPGA_W 0x7B   //A=1, D=2  // CHECKED
#define I2C_FPGA_R 0x7C
#define I2C_DESER954A_W 0x3D   //A=1, D=1  // CHECKED
#define I2C_DESER954A_R 0x3D
#define I2C_DESER954B_W 0x30   //A=1, D=1  // CHECKED
#define I2C_DESER954B_R 0x30
#define I2C_SER953A_W 0x0C    //A=1, D=1   // CHECKED
#define I2C_SER953A_R 0x0C
#define I2C_SER953B_W 0x0D    //A=1, D=1   // UPDATE THIS
#define I2C_SER953B_R 0x0D
#define I2C_TCA6416_A0_W 0x20 // A=1, D=2  // CHECKED
#define I2C_TCA6416_A0_R I2C_TCA6416_A0_W
#define I2C_TCA6416_A1_W 0x21 // A=1, D=2  // CHECKED
#define I2C_TCA6416_A1_R I2C_TCA6416_A1_W
#define I2C_MCP4726A0_DAC_W 0x60 // A=1, D=2  // CHECKED
#define I2C_MCP4726A0_DAC_R I2C_MCP4726_DAC_W
#define I2C_MCP4726A1_DAC_W 0x61 // A=1, D=2  // CHECKED
#define I2C_MCP4726A1_DAC_R I2C_MCP4726_DAC_W
#define I2C_MCP9800_W 0x48 // A=1, D=1   // CHECKED
#define I2C_MCP9800_R I2C_MCP9800_W
#define I2C_ATMAC402_MEM_W 0x50 // A=1, D=1   // CHECKED
#define I2C_ATMAC402_MEM_R I2C_ATMAC402_MEM_W
#define I2C_ATMAC402_EUI_W 0x58 // A=1, D=1   // CHECKED
#define I2C_ATMAC402_EUI_R I2C_ATMAC402_EUI_W
#define I2C_EEPROM_W I2C_ATMAC402_MEM_W //A=2, D=1   
#define I2C_EEPROM_R I2C_ATMAC402_MEM_R
#define I2C_SIL1136_W 0x3B  // 7 bit addr so R/W are same
#define I2C_SIL1136_R 0x3B
#define I2C_AP1302ID_W 0x3C  // This is the real address of AP1302 on the remote I2C bus.
#define I2C_AP1302ID_W 0x3C   // Could also be 0x7A. Same for both Ch A and B.
#define I2C_AP1302ALIAS_A_W 0x3C  // This is the alias for the AP1302 on Channel A
#define I2C_AP1302ALIAS_A_W 0x3C
#define I2C_AP1302ALIAS_B_W 0x3E  // This is the alias for the AP1302 on Channel B
#define I2C_AP1302ALIAS_B_W 0x3E
#define I2C_TCAPAD_W I2C_TCA6416_A1_W  // This is the ID for the TCA6416 on either channel
#define I2C_TCAPAD_R I2C_TCA6416_A1_R  // This is the ID for the TCA6416
#define I2C_TCAPADALIAS_A_W 0x1D // This is the TCA6416 on the Ctrl Pad of ChA - Aardvark use 0x3A>>1=0x1D
#define I2C_TCAPADALIAS_A_R 0x1D // This is the TCA6416 on the Ctrl Pad of ChA
#define I2C_TCAPADALIAS_B_W 0x1E // This is the TCA6416 on the Ctrl Pad of ChB - Aardvark use 0x3C>>1=0x1E
#define I2C_TCAPADALIAS_B_R 0x1E // This is the TCA6416 on the Ctrl Pad of ChB
    
#define ASCII_RETURN 0x0D // End command
#define ASCII_ZERO 0x30   // ASCII value for zero

#define ASCII_F      0x46 // FPGA Generic I2C Write f 123 45678<enter> writes address 12 with data 45678 when <enter> is pressed
#define ASCII_L      0x4C // Read debug log from AP1302 on ChA
#define ASCII_M      0x4D // MIPI Test Pattern On
#define ASCII_O      0x4F // AP1302 Test Pattern On
#define ASCII_P      0x50 // Serializer Test Pattern On
#define ASCII_S      0x53 // Toggle DVI AB Select MUX
#define ASCII_T      0x54 // HDMI Test Pattern On

#define ASCII_f      0x66 // fpga read function f 123<enter> reads address 123 when <enter> is pressed
#define ASCII_r      0x72 // Reset FPGA
#define ASCII_m      0x6D // MIPI Test Pattern Off
#define ASCII_o      0x6F // AP1302 Test Pattern Off
#define ASCII_p      0x70 // MIPI Test Pattern Off
#define ASCII_t      0x74 // HDMI Test Pattern Off    
#define ASCII_v      0x76 // Report version numbers
    

    
    
#define TCA6416_WALL 0  
#define TCA6416_ISO 1  
    
#define USE_CHA 1   // For deser, ser, sensor, ap1302, led dac, control pod tca6416 
#define USE_CHB 2   // For deser, ser, sensor, ap1302, led dac, control pod tca6416 
#define USE_DVIA 1  // For DVI Output Ch A
#define USE_DVIB 2 // For DVI Output Ch B
#define USE_DVIAB 3  // For DVI Output Ch AB
    
#define UART1_RXBUFFER_SIZE 0x40   // This is the RX buffer size as per MCC
#define UART1_TXBUFFER_SIZE 0x40   // This is the TX buffer size as per MCC
#define UART1_COMMAND_SIZE 0x04    // This is the number of bytes in a read/write command
    
#define SPI_BUFFER_SIZE 0x4      // 64 byte buffer for SPI transactions

#define SLAVE_I2C_GENERIC_RETRY_MAX           10
#define SLAVE_I2C_GENERIC_DEVICE_TIMEOUT      1000   // define slave timeout 
    
#define PCB_TEMP_ON_LIMIT     55    // Alert goes low if temp exceeds this
#define PCB_TEMP_OFF_LIMIT    50    // And stays low till the temp goes below this
#define TEMP_REPORT_THRESH 5000000 
#define LOCK_REPORT_THRESH 10000
        
#define LED_DCOUNT_THRESH  0x0FFFF //0x1FFFF // toggle PIC_LED every this many counts. Indicator LED.
    
#define MY_BUFFER_SIZE 0x300   // Length of text including \r\n
    
#define LED_ZERO 0           // Zero level for LED
#define LED_INITIAL_LEVEL 0  // Start at this level. 0-10 with 0 as OFF.    
#define LED_CURRENT_MAX 2000 // Inversely proportional so 2000 actually gives 0 LED current. 0 gives 1A
   
// EEPROM SAVE/LOAD RELATED
#define EEPROM_PICVERSION 0xA0 // store PIC version number here for verification
#define EEPROM_EMVERSION_OFFSET (EEPROM_PICVERSION+2)
#define EEPROM_BASE_OFFSET 0x00  // Defaults at 0x00. 
#define EEPROM_BASE_STEP 0x20    // User at 0x20, 0x40, 0x60, 0x80
    
void do_ms_delay( long int n );    

//extern uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
//extern uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH] ;

    
#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

