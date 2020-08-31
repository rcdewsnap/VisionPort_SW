/* 
 * File:   helper_functions.h
 * Author: vivek
 *
 * Created on March 20, 2019, 9:29 PM
 */

#ifndef HELPER_FUNCTIONS_H
#define	 HELPER_FUNCTIONS_H

#ifdef	__cplusplus
extern "C" {
#endif    
    

#include "main.h"
#include "i2c_wrappers.h"

#define MCLK_IS_36MHz // Uncomment to setup OV and MIPI Bridge for 36MHz MCLK. Comment for 24MHz
#define DS954_STP_CABLE // Uncomment to use STP cable, comment for COAX

#define SPI_READ_BUF_MAX_SIZE (SPI_READ_PKT_SIZE+4)  // For SPI Buffer Read transactions, +4 for the command
#define SPI_READ_PKT_SIZE 0x2000  // Number of bytes to read at a time
#define SPI_AP1302_TOTAL_SIZE 0x119A8 // 0x119A8 (200 good) // 0xFA9C(TP) // 0x119A8 (100KA) // 0x109F8(Base) // 0x109F4(Modified)  // Size of the file as reported in 
                // Totalphase Flash Center (max addr+1 after loading bin file)
#define PLL_INIT_SIZE 1004 //1004 (200 good) // 884(TP) // 100KA=1004 // 1024(Others) // From AP1302 XML file bootdata section
#define AP1302_CRC   0xcb0d // 0x0ED2(200 good) // 0x60FF(TP) // 0x0ED2 (100KA) // 0x2A29(Base) // 0xEA04(Modified)  // From AP1302 XML file bootdata section CRC check against SIPS_CRC
#define AP1302_CHECKSUM   0xcf0c // 0xCECA(200 Good) // 0x4D44(TP) // 0xCECA (100KA) // 0xBFDB(Base) // 0x7C70(Modified)  // From AP1302 XML file bootdata section CHECKSUM check against BOOTDATA_CHECKSUM
#define AP1302_I2C_START 0x8000  // AP1302 Boot procedure starts at this I2C address
#define AP1302_I2C_STOP 0x9FFF   // and cycles at this I2C address
#define AP1302_I2C_CYCLE (AP1302_I2C_STOP-AP1302_I2C_START+1)    
//#define READ_AP1302_CRC 1 // Uncomment to check AP1302 CRC of firmware load. 
// Reading CRC can break checksum read. Use checksum instead

// AP1302 Register addresses for various functions we use    
#define AP1302_ZOOM_ADDR 0x1010 // Zoom register address. 0x0100 is 1x zoom, 0x0200 is 2x zoom...
#define AP1302_BRIGHTNESS_ADDR 0x700A // This is actually gamma. Range from 0x1800 (1.5) to 0x2800 (2.5)
#define AP1302_PANX_ADDR 0x118C // Pan X. Range from 0x0000 (left) to 0x0100 (right)    
#define AP1302_PANY_ADDR 0x118E // Pan Y. Range from 0x0000 (top) to 0x0100 (bottom)  
#define AP1302_PAN_SPEED_ADDR 0x1012 // Pan speed setting. Imm=0x8000; Slow=0x0040; Fast=0x0200. 
    
// Max, Min, Default values for various parameters
#define ZOOM_MIN 0x0100 // Min is 1x
#define ZOOM_MAX 0x0500  //0x0800 // Max is 8x //RCD 6/1/20
#define ZOOM_DEFAULT 0x0100 // Default is 1x
#define ZOOM_STEP 0x0080 // Zoom step 0x0100 is 1x, 0x0080 is 0.5x
    
#define BRIGHTNESS_MIN 0x1800 // Min is 0x1800 which is gamma of 1.5    
#define BRIGHTNESS_MAX 0x2800 // Min is 0x2800 which is gamma of 2.5
#define BRIGHTNESS_DEFAULT 0x2000 // Min is 0x1800 which is gamma of 2.0
#define BRIGHTNESS_STEP 0x0200    // Brightness step value.

#define PANX_MIN 0x0000       // Min is 0x0000 which is left
#define PANX_MAX 0x0100       // Min is 0x0100 which is right
#define PANX_DEFAULT 0x0080   // Default in the middle
#define PANX_STEP 0x0020     // Step size
#define PANY_MIN 0x0000       // Min is 0x0000 which is top
#define PANY_MAX 0x0100       // Min is 0x0100 which is right
#define PANY_DEFAULT 0x0080   // Default in the middle
#define PANY_STEP 0x0020     // Step size
#define PAN_SPEED 0x0100      // Default pan speed. Imm=0x8000; Slow=0x0040; Fast=0x0200.
    
#define CAPTURE_DELAY 200   // in ms
    
#define MY_BUFFER_SIZE 0x300   // Length of text including \r\n   
    
#define NO_BUTTON 0x0000
#define BUTTON1 0x0001
#define BUTTON2 0x0002
#define BUTTON3 0x0004
#define BUTTON4 0x0008
#define BUTTON5 0x0010
#define BUTTON6 0x0020
#define BUTTON7 0x0040
#define BUTTON8 0x0080
#define BUTTON9 0x0100
#define BUTTON10 0x0200
#define BUTTON11 0x0400
#define SECRET_BUTTON 0x0401    
    
    

void do_ms_delay( long int n );    
void do_us_delay( long int n );

void set_mcp9800_temp_limit(unsigned int temp_on_limit, unsigned int temp_off_limit);
unsigned int read_mcp9800_temp(void);

void setup_mcp4726_dac(unsigned int dac_id);                   
void update_mcp4726_dac(unsigned int dac_id, unsigned int dacval);

void TCA6416_Initialize(uint8_t addr_low_bit);    // Initializes the TCA6416A. Call this first and only.
void TCA6416_configure(uint8_t addr_low_bit);         // Setup the IO Expander
unsigned int TCA6416_read_P0(uint8_t addr_low_bit);   // read the IO Expander Port0
unsigned int TCA6416_read_P1(uint8_t addr_low_bit);   // read the IO Expander Port1
unsigned int TCA6416_read_DIP8(uint8_t addr_low_bit); // Use this function to read DIP switch. It flips bits to match part markings.
void TCA6416_write_P0(uint8_t addr_low_bit, int value); // Writes to lower byte of IO
void TCA6416_write_P1(uint8_t addr_low_bit, int value); // Writes to higher byte of IO

void TCA6416_Pad_configure(uint8_t use_ch);
unsigned int TCA6416_Pad_Initialize(uint8_t use_ch);
unsigned int TCA6416_Pad_read_P0(uint8_t use_ch);
unsigned int TCA6416_Pad_read_P1(uint8_t use_ch);
void TCA6416_Pad_write_P0(uint8_t use_ch, int value);
void TCA6416_Pad_write_P1(uint8_t use_ch, int value);

void Channel_Initialize(uint8_t use_ch);
void Channel_Release(uint8_t use_ch);

void Ch_RedLed_On(uint8_t use_ch);    // Do RMW to set LED
void Ch_RedLed_Off(uint8_t use_ch);   // Do RMW to set LED
void Ch_RedLed_Toggle(uint8_t use_ch);   // Do RMW to toggle
void Ch_GreenLed_On(uint8_t use_ch);  // Do RMW to set LED
void Ch_GreenLed_Off(uint8_t use_ch); // Do RMW to set LED
void Ch_GreenLed_Toggle(uint8_t use_ch); // Do RMW to toggle

void DbgLedISORedOn(void); // Do RMW to set LED
void DbgLedISORedOff(void); // Do RMW to set LED
void DbgLedISOGreenOn(void); // Do RMW to set LED
void DbgLedISOGreenOff(void); // Do RMW to set LED
void DbgLedWallRedOn(void); // Do RMW to set LED
void DbgLedWallRedOff(void); // Do RMW to set LED
void DbgLedWallGreenOn(void); // Do RMW to set LED
void DbgLedWallGreenOff(void); // Do RMW to set LED

void setup_deserializer954(unsigned int deser_id, unsigned int deser_test_mode);  
void setup_deserializer954_tp(unsigned int deser_id);  
unsigned int setup_ser_953(unsigned int deser_id, unsigned int ser_test_mode); 
unsigned int setup_ser_953_tp(unsigned int deser_id);
void Ch_Deser_On(unsigned int deser_id);
void Ch_Deser_Off(unsigned int deser_id);
unsigned int setup_ap1302(unsigned int use_ch);


void increment_led_level(uint8_t use_ch);
void decrement_led_level(uint8_t use_ch);
void set_led_level(uint8_t use_ch);

void use_dvi_ch(uint8_t use_dvi);  // call this before trying to i2c a channel
uint8_t initialize_dvi(uint8_t use_dvi); // Call this once per channel to turn it on

void uart1_send_buffer(char *Buffer, int BufferLen);
void uart_send_welcome(void);
void uart_report_versions(int loopcount);

void process_input_char( uint8_t *buffer, unsigned int bufLen, unsigned int numBytesRead);

void ProcessButtonInput(uint8_t use_ch);

void button_secret_action(uint8_t use_ch);
void button1_action(uint8_t use_ch);
void button2_action(uint8_t use_ch);
void button3_action(uint8_t use_ch);
void button4_action(uint8_t use_ch);
void button5_action(uint8_t use_ch);
void button6_action(uint8_t use_ch);
void button7_action(uint8_t use_ch);
void button8_action(uint8_t use_ch);
void button9_action(uint8_t use_ch);
void button10_action(uint8_t use_ch);
void button11_action(uint8_t use_ch);

void reset_fpga(void);

uint8_t spi_check_id(void);      // Returns 1 if ID matches expected 0xEF7016    
void spi_wr_en(void);     // Writes 0x06 to FLASH
uint8_t spi_read_status(uint8_t StatusRegNum);  // Reads StatusReg 1, 2 or 3
void spi_read_unique_ID(void); // Read unique ID and report on UART
void spi_read_buffer(long start_address, uint16_t byteCount, uint8_t *dataReceived); // Read buffer
void ap1302_test_pattern_control(unsigned int use_ch, unsigned int tp_enable);
uint8_t ap1302_load_patch(unsigned int use_ch);

void ap1302_read_debug_log(unsigned int use_ch);

void ap1302_init_defaults(unsigned int use_ch);

void init_fpga_regs(void);
void set_fpga_dviab(unsigned int use_ch);
void toggle_fpga_dviab(void);

void close_capture_relay(void);
void open_capture_relay(void);

void turn_on_fpga_out_tp(uint8_t use_ch);
void turn_off_fpga_out_tp(uint8_t use_ch);

extern uint8_t glLedLevelA;
extern uint8_t glLedLevelB;
extern uint8_t glLedEnableA;
extern uint8_t glLedEnableB;

extern uint16_t gl_fpga_version;
extern uint16_t gl_ap1302_version;

extern uint8_t ds954_lock;

extern uint8_t i2c_deser;
extern uint8_t i2c_ser;

extern uint8_t glDbgLedIsoRed;
extern uint8_t glDbgLedIsoGreen;
extern uint8_t glDbgLedWallRed;
extern uint8_t glDbgLedWallGreen;

extern char myBuffer[MY_BUFFER_SIZE];
extern uint8_t myBufferLen;

extern uint8_t glCommandIndex;                  // This is the index of where we are in the current command
extern uint8_t glCommandBuffer[MY_BUFFER_SIZE]; // This is a buffer where we can assemble commands

extern uint8_t success_ap1302a;
extern uint8_t success_ap1302b;

extern int glButtonsA;
extern int glButtonsA_last;
extern int glButtonsB;
extern int glButtonsB_last;

extern int glZoomA;
extern int glBrightnessA;
extern int glPanXA;
extern int glPanYA;
extern int glZoomB;
extern int glBrightnessB;
extern int glPanXB;
extern int glPanYB;

extern int glMuxOutCh;

#ifdef	__cplusplus
}
#endif

#endif	/* HELPER_FUNCTIONS_H */

