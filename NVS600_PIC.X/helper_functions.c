/* 
 * File:   helper_functions.c
 * Author: vivek
 *
 * Created on March 20, 2019, 9:29 PM
 */


#include "helper_functions.h"

uint8_t Zoom_Mx_Cnt = 0; //RCD6/18/20 for sub@ helper.c 2460

void do_ms_delay( long int n )
{
    long int i;
    long int j;
    long int e;
    
    for ( i = 0; i < n; i++)
    {
        for (j = 0; j < 3200; j++)

        {      
         e = i - 1;
        }
    }
}

void do_us_delay( long int n )
{
    long int i;
    long int j;
    long int e;
    for ( i = 0; i < n; i++)
    {
        for (j = 0; j < 3; j++)
        {
            e = i - 1;
        }
    }
}

void set_mcp9800_temp_limit(unsigned int temp_on_limit, unsigned int temp_off_limit)
{
    // MCP9800 reg 3 holds the temp limit
    // Default value is 80C
    // Set desired value in C in bits [14:8]
    // Bit[15] is a sign bit, leave at 0
    // Bit[7:0] can be left at zero
    
    
    if(temp_on_limit > 127)  // Limit it to 7 bits = 127C
        temp_on_limit = 127;
    
    if(temp_off_limit > 127)  // Limit it to 7 bits = 127C
        temp_off_limit = 127;
    
    if(temp_off_limit > temp_on_limit)  // Make sure off is less than on otherwise alert will not fire
        temp_off_limit = temp_on_limit;
    
    // MCP9800 gets data MSByte then LSByte. Ours sends in reverse so we just
    // pretend like we are sending the high byte as the low to make it work out
    
    // Temp Hysteresis address = 0x02
    i2c_sendcommand_a1d2(0, I2C_MCP9800_W, 0x02, temp_off_limit);

    // Temp Limit address = 0x03
    i2c_sendcommand_a1d2(0, I2C_MCP9800_W, 0x03, temp_on_limit);
    
    // Send UART message
    sprintf(myBuffer, "Temp Alert Set to: %d C     \r\n", PCB_TEMP_ON_LIMIT);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);

}

unsigned int read_mcp9800_temp(void)
{  // This function reads the temperature from the MCP9800 temp IC
    // First byte = Temp in C 
    // We could read an additional byte to get 4 fractional bits but not gonna do it
  
    unsigned int pcb_temp = 0;
    
    pcb_temp = i2c_readdata_a1d1(0, I2C_MCP9800_W, 0x00);

    return(pcb_temp);
}

void setup_mcp4726_dac(unsigned int dac_id)
{
    // Setup the mcp4726 for VDD as VREF
    // Write it to the DAC
    // Byte 0: Command bits = 3'b010
    // Byte 0: Ref voltage sel bits = 2'b00
    // Byte 0: Power down bits = 2'b00
    // Byte 0: Gain bit = 1'b0
    // Byte 1: b15:b6
    // Byte 2: b7:b0
    unsigned int config_byte = 0x40; // VDD as VREF
    unsigned int dacvalue = 0x00; // Set DAC value reg to 0
    
    if(dac_id == 1)
    {
        i2c_sendcommand_a1d2(0, I2C_MCP4726A0_DAC_W, config_byte, dacvalue);
        sprintf(myBuffer, "LED Current DAC A Initialized\r\n");
    } else if(dac_id == 2)
    {
        i2c_sendcommand_a1d2(0, I2C_MCP4726A1_DAC_W, config_byte, dacvalue);
        sprintf(myBuffer, "LED Current DAC B Initialized\r\n");
    }
            
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    

}

void update_mcp4726_dac(unsigned int dac_id, unsigned int dacval)
{   // This functions updates the voltage output of the MCP4726 DAC via
    // the I2C bus. The DAC range is 12 bits (0 to 4095). This corresponds
    // to different voltages. When set to have VREF of 3.3V then 4095 = 3.3V out
    //    

    // Check that dacval is within range
    if(dacval > 4095)
    {
        dacval = 4095;        
    }
    
    // In order to reuse the i2c command function we separate the dacval[11:0] into two separate 8 bit words
    // Word 0 = {4'h0, dacval[11:8]}
    // Word 1 = dacval[7:0]
    // Send Word0 as the "Register address"
    // Send Word1 as the first and only databyte
        
    // Write it to the DAC
    if(dac_id == 1)
    {
        i2c_sendcommand_a1d1(0, I2C_MCP4726A0_DAC_W, (dacval>>8), (dacval%256));   // one byte addr, one byte data
    } else if(dac_id == 2)
    {
        i2c_sendcommand_a1d1(0, I2C_MCP4726A1_DAC_W, (dacval>>8), (dacval%256));   // one byte addr, one byte data
    }

}

// read is done via the IO expander on the POD board
void TCA6416_configure(uint8_t addr_low_bit)
{
    uint8_t tca_i2c_addr = I2C_TCA6416_A0_W;

    if(addr_low_bit == 0x00)
    {
        tca_i2c_addr = I2C_TCA6416_A0_W;
    } else
    {
        tca_i2c_addr = I2C_TCA6416_A1_W;
    }
    
    // This function configures the IO Expander on the wall side (addr=0)
    // Pins are:
    // P0 = Output = A_DV_Resetn
    // P1 = Output = B_DV_Resetn
    // P2 = Output = AB_DV_Resetn
    // P3 = Input = Wall_IO_Spare1
    // P4 = Input = A_DV_INT
    // P5 = Input = B_DV_INT
    // P6 = Input = AB_DV_INT
    // P7 = Input = Wall_IO_Spare2
    // P10 = Output = I2CEN1
    // P11 = Output = I2CEN2
    // P12 = Output = I2CEN3
    // P13 = Output = I2CEN4
    // P14 = Output = Capture_Signal
    // P15 = Input = Wall_IO_Sp3
    // P16 = Output = DBG_LED_RED_D21
    // P17 = Output = DBG_LED_GREEN_D22
    
    // and also the IO Expander on the iso side (addr=1)
    // Pins are:
    // P0 = Output = ADS954_PDB
    // P1 = Output = ADS954_BISTEN
    // P2 = Input = ADS954_PASS
    // P3 = Output = BDS954_PDB
    // P4 = Output = BDS954_BISTEN
    // P5 = Input = BDS954_PASS
    // P6 = Output = DBG_LED_RED_D18
    // P7 = Output = DBG_LED_GREEN_D19
    // P10 = Output = CHA_Led_Red
    // P11 = Output = CHA_Led_Green
    // P12 = Output = CHB_Led_Red
    // P13 = Output = CHB_Led_Green
    // P14 = Output = CTRLBUS0
    // P15 = Output = CTRLBUS1
    // P16 = Output = CTRLBUS2
    // P17 = Output = CTRLBUS3
    
    // IO Direction Config Registers
    #define TCA6416_P0_CFGADDR 0x06 // From datasheet IO direction for Port 0
    #define TCA6416_P1_CFGADDR 0x07 // From datasheet IO direction for Port 1
    #define TCA6416_P0_CFG 0xF8 // As described above, 1=input
    #define TCA6416_P1_CFG 0x20 // As described above, 1=input
    #define TCA6416B_P0_CFG 0x24 // As described above, 1=input
    #define TCA6416B_P1_CFG 0x00 // As described above, 1=input

    // Polarity inversion registers
    #define TCA6416_P0_POLADDR 0x04 // From datasheet Polarity Inversion for Port 0
    #define TCA6416_P1_POLADDR 0x05 // From datasheet Polarity Inversion for Port 1
    #define TCA6416_P0_POL 0x00 // None inverted
    #define TCA6416_P1_POL 0x00 // None inverted
    #define TCA6416B_P0_POL 0x00 // None inverted
    #define TCA6416B_P1_POL 0x00 // None inverted

    if(addr_low_bit == 0x0)
    {
        // Setup IO Expander output data values before making them outputs to avoid blips
        i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x02, 0x00);         
        i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x03, 0x00);

        // Write Configuration Registers
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_CFGADDR, TCA6416_P0_CFG);   // one byte addr, one byte data
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_CFGADDR, TCA6416_P1_CFG);   // one byte addr, one byte data

        // We don't invert any polarities we don't really need to write those registers
        // But doing so anyway so that later if we want to invert it's all set up
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_POLADDR, TCA6416_P0_POL);   // one byte addr, one byte data
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_POLADDR, TCA6416_P1_POL);   // one byte addr, one byte data
    } else if(addr_low_bit == 0x1)
    {
        // Setup IO Expander output data values before making them outputs to avoid blips
        i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x02, 0x00);         
        i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x03, 0x00);

        // Write Configuration Registers
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_CFGADDR, TCA6416B_P0_CFG);   // one byte addr, one byte data
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_CFGADDR, TCA6416B_P1_CFG);   // one byte addr, one byte data

        // We don't invert any polarities we don't really need to write those registers
        // But doing so anyway so that later if we want to invert it's all set up
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_POLADDR, TCA6416B_P0_POL);   // one byte addr, one byte data
        i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_POLADDR, TCA6416B_P1_POL);   // one byte addr, one byte data        
    }

    return;
}

// read is done via the IO expander on the POD board
unsigned int TCA6416_read_P0(uint8_t addr_low_bit)
{
    // This function reads the 8 bits of the IO expander P0
    // on the POD board (TCA6416ARTWR with ADDR set low)
    
    unsigned int data8;
    
    uint8_t tca_i2c_addr = I2C_TCA6416_A0_W;

    if(addr_low_bit == 0x00)
    {
        tca_i2c_addr = I2C_TCA6416_A0_W;
    } else
    {
        tca_i2c_addr = I2C_TCA6416_A1_W;
    }
    
    data8 = i2c_readdata_a1d1(2, tca_i2c_addr, 0x00);
    
    return data8;
}

// read is done via the IO expander
unsigned int TCA6416_read_P1(uint8_t addr_low_bit)
{
    // This function reads the 8 bits of the IO expander 
    // specified by the addr_low_bit.
    // 0 = Main board IO extender
    // 1 = Buttons board IO extender
    
    unsigned int data8;
        
    // Setup correct I2C address based on low bit
    uint8_t tca_i2c_addr = I2C_TCA6416_A0_W;
    if(addr_low_bit == 0x00)
    {
        tca_i2c_addr = I2C_TCA6416_A0_W;
    } else
    {
        tca_i2c_addr = I2C_TCA6416_A1_W;
    }
    
    // Call the read function to do the thing
    data8 = i2c_readdata_a1d1(2, tca_i2c_addr, 0x01);
    
    return data8;
}

// write is done via the IO expander on the POD board
void TCA6416_write_P0(uint8_t addr_low_bit, int value)
{
    // This function write the 8 bits of the IO expander P0    
    
    // Setup correct I2C address based on low bit
    uint8_t tca_i2c_addr = I2C_TCA6416_A0_W;
    if(addr_low_bit == 0x00)
    {
        tca_i2c_addr = I2C_TCA6416_A0_W;
    } else
    {
        tca_i2c_addr = I2C_TCA6416_A1_W;
    }

    
    #define TCA6416_P0_WRADDR 0x02
    
    unsigned int data8;

    data8 = value & 0x00FF; // Mask any higher order bits
    
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_WRADDR, data8);
}

// write is done via the IO expander on the POD board
void TCA6416_write_P1(uint8_t addr_low_bit, int value)
{
    // This function write the 8 bits of the IO expander P0    
    
    #define TCA6416_P1_WRADDR 0x03

    // Setup correct I2C address based on low bit
    uint8_t tca_i2c_addr = I2C_TCA6416_A0_W;
    if(addr_low_bit == 0x00)
    {
        tca_i2c_addr = I2C_TCA6416_A0_W;
    } else
    {
        tca_i2c_addr = I2C_TCA6416_A1_W;
    }    
    
    unsigned int data8;

    data8 = value & 0x00FF; // Mask any higher order bits
    
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_WRADDR, data8);
    
}

void TCA6416_Initialize(uint8_t addr_low_bit)
{  // Call this function to setup the IO Extenders as required    

    // Setup I2C IO Extender
    TCA6416_configure(addr_low_bit);        
        
    do_ms_delay(100);    
    
    if(addr_low_bit == 0x0)
    {
        TCA6416_write_P0(addr_low_bit, 0x00); // Red LED on only. DVI chips in reset
        TCA6416_write_P1(addr_low_bit, 0x40); // Red LED on only. DVI chips in reset
        do_ms_delay(300);           // Wait a moment
        TCA6416_write_P1(addr_low_bit, 0x80); // Green LED on only. DVI chips in reset
        sprintf(myBuffer, "ISO Side GPIO Extender Initialized\r\n");
    } else if(addr_low_bit == 0x1)
    {
        TCA6416_write_P0(addr_low_bit, 0x40); // Red LED on.
        TCA6416_write_P1(addr_low_bit, 0x05); // CHA and CHB Red LEDs on
        do_ms_delay(300);           // Wait a moment before enabling second serializer
        TCA6416_write_P0(addr_low_bit, 0x80); // Green LED on
        TCA6416_write_P1(addr_low_bit, 0x0A); // CHA and CHB Green LEDs on
        sprintf(myBuffer, "Wall Side GPIO Extender Initialized\r\n");
    }
        
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    
}

// Configure the IO expander on the control pad
void TCA6416_Pad_configure(uint8_t use_ch)
{
    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }
    
    // This function configures the IO Expander on the wall side (addr=0)
    // Pins are:
    // P0-12 = Input = Button1 -> Button 12
    // P17 = Output = DBG_LED_GREEN
       
    // IO Direction Config Registers
    #define TCA6416_P0_CFGADDR 0x06 // From datasheet IO direction for Port 0
    #define TCA6416_P1_CFGADDR 0x07 // From datasheet IO direction for Port 1
    #define TCA6416PAD_P0_CFG 0xFF // As described above, 1=input
    #define TCA6416PAD_P1_CFG 0x07 // As described above, 1=input


    // Polarity inversion registers
    #define TCA6416_P0_POLADDR 0x04 // From datasheet Polarity Inversion for Port 0
    #define TCA6416_P1_POLADDR 0x05 // From datasheet Polarity Inversion for Port 1
    #define TCA6416PAD_P0_POL 0xFF // All inverted
    #define TCA6416PAD_P1_POL 0x07 // Low 3 bits inverted

    // Setup IO Expander output data values before making them outputs to avoid blips
    i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x02, 0x00);         
    i2c_sendcommand_a1d1(2, tca_i2c_addr, 0x03, 0x00);

    // Write Configuration Registers
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_CFGADDR, TCA6416PAD_P0_CFG);   // one byte addr, one byte data
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_CFGADDR, TCA6416PAD_P1_CFG);   // one byte addr, one byte data

    // We don't invert any polarities we don't really need to write those registers
    // But doing so anyway so that later if we want to invert it's all set up
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_POLADDR, TCA6416PAD_P0_POL);   // one byte addr, one byte data
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_POLADDR, TCA6416PAD_P1_POL);   // one byte addr, one byte data

    return;
}

unsigned int TCA6416_Pad_Initialize(uint8_t use_ch)
{  
    // Call this function to setup the IO Extenders as required    
    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }
    
    // Setup I2C IO Extender
    TCA6416_Pad_configure(use_ch);        
        
    do_ms_delay(100);    

    TCA6416_Pad_write_P0(use_ch, 0x00); // Low byte is zero
    TCA6416_Pad_write_P1(use_ch, 0x80); // Green LED on
    
    if(use_ch == USE_CHA)
    {
        sprintf(myBuffer, "Control Pad A GPIO Extender Initialized\r\n");
    } else if(use_ch == USE_CHB)
    {
        sprintf(myBuffer, "Control Pad B GPIO Extender Initialized\r\n");
    }
        
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);  
    
    return(1);
}

// read is done via the IO expander on the POD board
unsigned int TCA6416_Pad_read_P0(uint8_t use_ch)
{
    // This function reads the 8 bits of the IO expander P0
    // on the control pad

    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    unsigned int data8;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }        
        
    data8 = i2c_readdata_a1d1(2, tca_i2c_addr, 0x00);
    
    return data8;
}

// read is done via the IO expander
unsigned int TCA6416_Pad_read_P1(uint8_t use_ch)
{
    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    unsigned int data8;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }        
        
    // Call the read function to do the thing
    data8 = i2c_readdata_a1d1(2, tca_i2c_addr, 0x01);
    
    return data8;
}

// write is done via the IO expander on the POD board
void TCA6416_Pad_write_P0(uint8_t use_ch, int value)
{
    // This function write the 8 bits of the IO expander P0    
    #define TCA6416_P0_WRADDR 0x02
    
    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    unsigned int data8;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }        
        
    data8 = value & 0x00FF; // Mask any higher order bits
    
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P0_WRADDR, data8);
}

// write is done via the IO expander on the POD board
void TCA6416_Pad_write_P1(uint8_t use_ch, int value)
{
    // This function write the 8 bits of the IO expander P0    
    
    #define TCA6416_P1_WRADDR 0x03

    uint8_t tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    unsigned int data8;

    if(use_ch == USE_CHA)
    {
        tca_i2c_addr = I2C_TCAPADALIAS_A_W;
    } else
    {
        tca_i2c_addr = I2C_TCAPADALIAS_B_W;
    }        
    
    data8 = value & 0x00FF; // Mask any higher order bits
    
    i2c_sendcommand_a1d1(2, tca_i2c_addr, TCA6416_P1_WRADDR, data8);
    
}

void Ch_Deser_On(unsigned int deser_id)    // Do RMW to set 
{
    int temp = 0;
    
    temp = TCA6416_read_P0(TCA6416_ISO);  // First read
    
    // Next modify
    if(deser_id == USE_CHA)
    {        
        temp = temp | 0x01;        
    } else if(deser_id == USE_CHB)
    {
        temp = temp | 0x08;
    }
    
    // Then write
    TCA6416_write_P0(TCA6416_ISO, temp);
    
    // Wait a bit for things to settle
    do_ms_delay(100);
    
    // Configure Channel Deserializer
    // In test mode it runs headless so configure to generate a test pattern
#ifdef DESER_TEST_MODE    
    setup_deserializer954(deser_id, 1);
#else
    setup_deserializer954(deser_id, 0); 
#endif
    
}

void Ch_Deser_Off(unsigned int deser_id)   // Do RMW to set 
{
    int temp = 0;
    
    // Read
    temp = TCA6416_read_P0(TCA6416_ISO);
    
    // Modify
    if(deser_id == USE_CHA)
        temp = temp & 0xFE;
    else if(deser_id == USE_CHB)
        temp = temp & 0xF7;
    
    // Write
    TCA6416_write_P0(TCA6416_ISO, temp);
}

void Channel_Initialize(uint8_t use_ch)
{
    #define RETRIES 2  // AP1302 Boot Retries

    uint8_t retry_count = 0;
    uint8_t ap1302_checksum_pass = 0;

    uint8_t ch_lock = 0;
    
    // Setup front panel lights
    // Eventually gate this on Lock as well?
    Ch_RedLed_Off(use_ch);
    Ch_GreenLed_On(use_ch);

    // Setup Channel LED and plugin detection message
    if(use_ch == USE_CHA)
    {
        glLedLevelA = LED_INITIAL_LEVEL; // Set to default LED level
        sprintf(myBuffer, "Channel A Cable Plugged In\r\n");        
    }
    else if(use_ch == USE_CHB)
    {
        glLedLevelB = LED_INITIAL_LEVEL; // Set to default LED level
        sprintf(myBuffer, "Channel B Cable Plugged In\r\n");
    }
    
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    
        
    // Set actual LED level
    set_led_level(use_ch);

    // Enable Channel Deserializer
    Ch_Deser_On(use_ch);
    do_ms_delay(200);
    
//    // Reset FPGA just for now to test if this makes the DVI show up automatically
//    reset_fpga(); // It worked so leaving it in place for now. Will figure something out
//                  // if this becomes an issue. Could probably happen earlier than this
//                  // One fix would be to enable the deserializer and Deser test pattern
//                  // early on for both channels. After that reset the FPGA. Then use the
//                  // lock signal to just switch the Deserializer from tp to live mode.
//                  // This would be a good diagnostic too, except that the tp is ugly. We
//                  // have a nicer test pattern.
//    // Enable the test pattern on the DVI output
//    turn_on_fpga_out_tp();
    
    // Check lock signal and report on UART
    if(use_ch == USE_CHA)
    {
        ch_lock = ADS954_LOCK_GetValue();

        if(ch_lock == 1)
            sprintf(myBuffer, "Channel A SerDes Locked\r\n");
        else
            sprintf(myBuffer, "** Channel A SerDes Not Locked!! **\r\n");
        
    } else if(use_ch == USE_CHB)
    {
        ch_lock = BDS954_LOCK_GetValue();
        
        if(ch_lock == 1)
            sprintf(myBuffer, "Channel B SerDes Locked\r\n");
        else
            sprintf(myBuffer, "** Channel B SerDes Not Locked!! **\r\n");
    }
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);        
    
    // Enable Channel Serializer
    setup_ser_953(use_ch, 0);
    do_ms_delay(100);
    
    // Enable Channel AP1302, Loop till success
    do{
        ap1302_checksum_pass = setup_ap1302(use_ch);
        do_ms_delay(100);            
        retry_count++;

        if(UART_VERBOSITY > 1)
        {
            sprintf(myBuffer, "\r\nChannel %d AP1302 Boot Attempt %d checksum status %d **\r\n", use_ch, retry_count, ap1302_checksum_pass);
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);  
        }
        
    } while((ap1302_checksum_pass != 1) & (retry_count < RETRIES));

    // Enable Channel Control Pod GPIO Extender
    TCA6416_Pad_Initialize(use_ch);
    do_ms_delay(100);
    
}

void Channel_Release(uint8_t use_ch)
{   
    // Disable Channel Deserializer
    Ch_Deser_Off(use_ch);

    // Setup front panel LEDs
    Ch_RedLed_On(use_ch);
    Ch_GreenLed_Off(use_ch);
    
    // Turn on FPGA test pattern
    turn_on_fpga_out_tp(use_ch); // Check if showing this channel on AB then enable or toggle AB as well

    // Turn Off LED Driver
    if(use_ch == USE_CHA)
    {
        glLedLevelA = LED_ZERO; // Set to default LED level
        sprintf(myBuffer, "Channel A Cable Not Connected\r\n");
    }
    else if(use_ch == USE_CHB)
    {
        glLedLevelB = LED_ZERO; // Set to default LED level
        sprintf(myBuffer, "Channel B Cable Not Connected\r\n");
    }

    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    
    
    // Turn LED driver off
    set_led_level(use_ch);

}

void Ch_RedLed_On(uint8_t use_ch)    // Do RMW to set LED
{
    int temp = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
        temp = temp | 0x01;
    else
        temp = temp | 0x04;
    
    TCA6416_write_P1(TCA6416_ISO, temp);
}

void Ch_RedLed_Off(uint8_t use_ch)   // Do RMW to set LED
{
    int temp = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
        temp = temp & 0xFE;
    else
        temp = temp & 0xFB;
        
    TCA6416_write_P1(TCA6416_ISO, temp);
}

void Ch_RedLed_Toggle(uint8_t use_ch)  // Do RMW to set LED
{
    int temp = 0;
    int state = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
    {
        // Get current state
        state = temp & 0x01;
    }
    else
    {
        state = temp & 0x04;
    }
    
    // if State == 0 then LED is off, turn it on
    if(state == 0) 
        Ch_RedLed_On(use_ch); // LED is off so turn it on
    else 
        Ch_RedLed_Off(use_ch); // otherwise turn it off
    
}

void Ch_GreenLed_On(uint8_t use_ch)  // Do RMW to set LED
{
    int temp = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
        temp = temp | 0x02;
    else
        temp = temp | 0x08;
        
    TCA6416_write_P1(TCA6416_ISO, temp);
}

void Ch_GreenLed_Off(uint8_t use_ch) // Do RMW to set LED
{
    int temp = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
        temp = temp & 0xFD;
    else
        temp = temp & 0xF7;
        
    TCA6416_write_P1(TCA6416_ISO, temp);
}

void Ch_GreenLed_Toggle(uint8_t use_ch)  // Do RMW to set LED
{
    int temp = 0;
    int state = 0;
    
    temp = TCA6416_read_P1(TCA6416_ISO);
    
    if(use_ch == USE_CHA)
    {
        // Get current state
        state = temp & 0x02;
    }
    else
    {
        state = temp & 0x08;
    }
    
    // if State == 0 then LED is off, turn it on
    if(state == 0) 
        Ch_GreenLed_On(use_ch); // LED is off so turn it on
    else 
        Ch_GreenLed_Off(use_ch); // otherwise turn it off
    
}


void DbgLedISORedOn(void) // Do RMW to set LED
{

    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P0(TCA6416_ISO);
    temp = temp | 0x40;
    TCA6416_write_P0(TCA6416_ISO, temp);
#endif
    
    glDbgLedIsoRed = 1;
}

void DbgLedISORedOff(void) // Do RMW to set LED
{

    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P0(TCA6416_ISO);
    temp = temp & 0xBF;
    TCA6416_write_P0(TCA6416_ISO, temp);
#endif
    
    glDbgLedIsoRed = 0;
}

void DbgLedISOGreenOn(void) // Do RMW to set LED
{
    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P0(TCA6416_ISO);
    temp = temp | 0x80;
    TCA6416_write_P0(TCA6416_ISO, temp);
#endif
    
    glDbgLedIsoGreen = 1;
}

void DbgLedISOGreenOff(void) // Do RMW to set LED
{
    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P0(TCA6416_ISO);
    temp = temp & 0x7F;
    TCA6416_write_P0(TCA6416_ISO, temp);
#endif
    
    glDbgLedIsoGreen = 0;
}

void DbgLedWallRedOn(void) // Do RMW to set LED
{
    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P1(TCA6416_WALL);
    temp = temp | 0x40;
    TCA6416_write_P1(TCA6416_WALL, temp);
#endif
    
    glDbgLedWallRed = 1;
}

void DbgLedWallRedOff(void) // Do RMW to set LED
{

#ifndef I2C_QUIET_MODE    
    int temp = 0;
    temp = TCA6416_read_P1(TCA6416_WALL);
    temp = temp & 0xBF;
    TCA6416_write_P1(TCA6416_WALL, temp);
#endif
    
    glDbgLedWallRed = 0;
}

void DbgLedWallGreenOn(void) // Do RMW to set LED
{
    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P1(TCA6416_WALL);
    temp = temp | 0x80;
    TCA6416_write_P1(TCA6416_WALL, temp);
#endif
    
    glDbgLedWallGreen = 1;
}

void DbgLedWallGreenOff(void) // Do RMW to set LED
{
    
#ifndef I2C_QUIET_MODE
    int temp = 0;
    temp = TCA6416_read_P1(TCA6416_WALL);
    temp = temp & 0x7F;
    TCA6416_write_P1(TCA6416_WALL, temp);
#endif
    
    glDbgLedWallGreen = 0;
}


void setup_deserializer954(unsigned int deser_id, unsigned int deser_test_mode)
{

    if(deser_test_mode != 0)
    {
        setup_deserializer954_tp(deser_id);
    }
    else 
    {
        if(deser_id == 1)
        {       
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x0C, 0x81);   // Sets up to use Port 0
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x4C, 0x01);   // Sets up to use Port 0
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x58, 0x5D);  // Used to be 0x5E. Reduced to improve lock // Enable I2C Pass Through
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x5C, 0x18); // Setup Slave Alias for Serializer

            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x5D, (I2C_AP1302ID_W<<1)); // Setup Slave ID for AP1302 on remote bus
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x65, (I2C_AP1302ALIAS_A_W<<1)); // Setup Slave Alias for AP1302 on Ch A

            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x5E, (I2C_TCAPAD_W<<1)); // Setup Slave ID for TCA6416 on Ch A
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x66, (I2C_TCAPADALIAS_A_W<<1)); // Setup Slave Alias for TCA6416 on Ch A.

            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x6D, 0x78);   // Port Config - STP Cable
    //        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x6D, 0x7C);   // Port Config - COAX Cable

            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x0F, 0x0B); // Setup GPIO0, 1 and 3 as inputs to send to serializer
                                                                  // GPIO pin outputs are off by default, leave them alone.
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x6E, 0x10); // Setup GPIO0, 1 and 3 as inputs to send to serializer
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x6F, 0x32); // Setup GPIO0, 1 and 3 as inputs to send to serializer

            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x20, 0x00); // Disable forwarding for Port 0

        if(MIPI_LANES == 4)
        {   // 4 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x33, 0x03); // Enable CSI-2 Port with 4 MIPI lanes  
        }        
        else if(MIPI_LANES == 2)
        {   // 2 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x33, 0x23); // Enable CSI-2 Port with 2 MIPI lanes
        }


            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x20, 0x20); // Enable forwarding for Port 0

            sprintf(myBuffer, "Channel A Deserializer Configured\r\n");
        } else if(deser_id == 2)
        {
            // Do Ch B setup here. Same as above but with different aliases for I2C devices.               
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x0C, 0x81);   // Sets up to use Port 0
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x4C, 0x01);   // Sets up to use Port 0
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x58, 0x5D);  // Used to be 0x5E. Reduced to improve lock // Enable I2C Pass Through
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x5C, 0x1A); // Setup Slave Alias for Serializer

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x5D, (I2C_AP1302ID_W<<1)); // Setup Slave ID for AP1302 on remote bus
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x65, (I2C_AP1302ALIAS_B_W<<1)); // Setup Slave Alias for AP1302 on Ch B on Ch A

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x5E, (I2C_TCAPAD_W<<1)); // Setup Slave ID for TCA6416 on Ch B
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x66, (I2C_TCAPADALIAS_B_W<<1)); // Setup Slave Alias for TCA6416 on Ch B.

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x6D, 0x78);   // Port Config - STP Cable
    //        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x6D, 0x7C);   // Port Config - COAX Cable

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x0F, 0x0B); // Setup GPIO0, 1 and 3 as inputs to send to serializer
                                                                  // GPIO pin outputs are off by default, leave them alone.
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x6E, 0x10); // Setup GPIO0, 1 and 3 as inputs to send to serializer
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x6F, 0x32); // Setup GPIO2 & 3 for pass-through

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x20, 0x00); // Disable forwarding for Port 0

            if(MIPI_LANES == 4)
            {   // 4 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x33, 0x03); // Enable CSI-2 Port with 4 MIPI lanes  
            }        
            else if(MIPI_LANES == 2)
            {   // 2 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x33, 0x23); // Enable CSI-2 Port with 2 MIPI lanes
            }

            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x20, 0x20); // Enable forwarding for Port 0

            sprintf(myBuffer, "Channel B Deserializer Configured\r\n");
        }

        // Send UART message
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);    
    }
    
}


void setup_deserializer954_tp(unsigned int deser_id)
{

    if(deser_id == 1)
    {
        // Port forwarding not needed for Deser PGEN
        
        if(MIPI_LANES == 4)
        {   // 4 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x33, 0x03); // Enable CSI-2 Port with 4 MIPI lanes  
        }        
        else if(MIPI_LANES == 2)
        {   // 2 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x33, 0x23); // Enable CSI-2 Port with 2 MIPI lanes
        }
                
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB0, 0x00);          
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x01); // PGEN CTL
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x01); // Enables the Pattern Generator
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x02);  // PGEN CFG
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x33);  // Set for 8 color bars, 3 bytes/block

        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x03); // PGEN CSI DI
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x1E); // YUV422 8 bit = 0x1E; YUV422 10 bit=0x1F; RAW10=0x2B, RGB888=0x24
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x04); // PGEN LINE SIZE 1 
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x0F); // Set to 3,840 (0x0F00) = 1920*2 (1920 pix/row, 2 bytes/pix)
                                                              // Set to 1280 (0x0500)  = 640*2 (640x 2 bytes/pixel)
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x05); // PGEN LINE SIZE 0
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x00); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x06); // PGEN BAR SIZE1        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x01); // Set for 480 (0x1E0) bytes = 240 pixels. 8 bars = 1920 pixels
                                                              // Set for 160 (0xA0) bytes for 640pix wide frame
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x07); // PGEN BAR SIZE 0
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0xE0); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x08); // PGEN ACT LPF1        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x04); // 1080= 0x438  ; For 720=02D0 ; For 480= 0x1E0
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x09); // PGEN ACT LPF0
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x38); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0A); // PGEN TOT LPF1        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x04); // 1080=0x465 (1125) ; For 720=0x041A (1050) ; For 480=525(0x20D)
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0B); // PGEN TOT LPF0
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x65); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0C); // PGEN LINE PD1       
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x0B); // 1080= 0xB90 (2,960)   ; For 720 = 0x0C67  (Line time / 10ns)
                                                              // 480 = 0x0C67
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0D); // PGEN LINE PD0
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x90); 
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0E); // PGEN VBP
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x24); // For 1080= 0x24 (36) ; For 720 = 0x21
        
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB1, 0x0F); // PGEN VFP
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0xB2, 0x04); // For 1080 = 0x04 ; For 720 = 0x0A
        
        sprintf(myBuffer, "Channel A Deserializer Set for Test Pattern, %d MIPI Lanes\r\n", MIPI_LANES);
        
    } else if(deser_id == 2)
    {
        // Do Ch B setup here. Same as above but with different aliases for I2C devices.
        if(MIPI_LANES == 4)
        {   // 4 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x33, 0x03); // Enable CSI-2 Port with 4 MIPI lanes  
        }        
        else if(MIPI_LANES == 2)
        {   // 2 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x33, 0x23); // Enable CSI-2 Port with 2 MIPI lanes
        }
                
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB0, 0x00);  
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x01); // PGEN CTL
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x01); // Enables the Pattern Generator
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x02);  // PGEN CFG
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x33);  // Set for 8 color bars, 2 bytes/block

        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x03); // PGEN CSI DI
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x1E); // YUV422 8 bit = 0x1E; RAW10=0x2B
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x04); // PGEN LINE SIZE 1 
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x0F); // Set to 3,840 = 1920*2
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x05); // PGEN LINE SIZE 0
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x00); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x06); // PGEN BAR SIZE1        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x01); // Set for 480 bytes = 240 pixels. 8 bars = 1920 pixels
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x07); // PGEN BAR SIZE 0
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0xE0); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x08); // PGEN ACT LPF1        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x04); // //1080= 0x438  ; For 720=02D0
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x09); // PGEN ACT LPF0
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x38); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0A); // PGEN TOT LPF1        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x04); // 1080=0x465 (1125) ; For 720=0x041A (1050)
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0B); // PGEN TOT LPF0
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x65); // See above
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0C); // PGEN LINE PD1       
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x0B); // 1080= 0xB90 (2,960)   ; For 720 = 0x0C67  (Line time / 10ns)
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0D); // PGEN LINE PD0
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x90); 
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0E); // PGEN VBP
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x24); // For 1080= 0x24 (36) ; For 720 = 0x21
        
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB1, 0x0F); // PGEN VFP
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0xB2, 0x04); // For 1080 = 0x04 ; For 720 = 0x0A
   
        sprintf(myBuffer, "Channel B Deserializer Set for Test Pattern, %d MIPI Lanes\r\n", MIPI_LANES);
    }

    
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    
    
}

unsigned int setup_ser_953(unsigned int deser_id, unsigned int ser_test_mode)
{  // This function sets up the DS90UB953 Serializer
      
    unsigned int success = 0;
    uint8_t temp = 0;

    if(ser_test_mode != 0)
    {
        setup_ser_953_tp(deser_id);
    }
    else
    {
        if(deser_id == 1)
        { // Deser #1 for when we have more than one

//            // First reset the serializer and wait a moment
//            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x01, 0x02); // Full Digital Reset
//            do_ms_delay(100); // Wait a moment
            
            if(MIPI_LANES == 4)
            {   // 4 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x02, 0x73); // # MIPI Lanes = 4
            }        
            else if(MIPI_LANES == 2)
            {   // 2 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x02, 0x53); // # MIPI Lanes = 4
            }

            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x03, 0x10); // Mode 
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x0E, 0xB4); // Set GPIO3,1,0 to outputs, Disable those inputs
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x0D, 0xF0); // All GPIOs are controlled remotely
            
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB0, 0x00);          
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x01); // PGEN CTL
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x00); // Disables the Pattern Generator

            temp = i2c_readdata_a1d1(0, I2C_SER953A_W, 0x00); // Read Ser addr for check

            if((temp>>2) == I2C_SER953A_W)
            {
                sprintf(myBuffer, "Channel A Serializer Configured\r\n");
                success = 1;
            }
            else
            {
                sprintf(myBuffer, "** Channel A Serializer FAILED !! **\r\n");
                success = 0;
            }

        } else if(deser_id == 2)
        {
            // Do Ch B setup here. Same as above but with different aliases for I2C devices.                
            if(MIPI_LANES == 4)
            {   // 4 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x02, 0x73); // # MIPI Lanes = 4
            }        
            else if(MIPI_LANES == 2)
            {   // 2 MIPI Lanes
                i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x02, 0x53); // # MIPI Lanes = 4
            }

            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x03, 0x10); // Mode 
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x0E, 0xB4); // Set GPIO3,1,0 to outputs, Disable those inputs
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x0D, 0xF0); // All GPIOs are controlled remotely
            
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB0, 0x00);          
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x01); // PGEN CTL
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x00); // Disables the Pattern Generator

            temp = i2c_readdata_a1d1(0, I2C_SER953B_W, 0x00); // Read Ser addr for check

            if((temp>>2) == I2C_SER953A_W) // Yes this is still the ChA value as it is the 
                                           // I2C ID of the Serializer not the alias of the Serializer
                                           // ID is same for Ch A and B as it is the address on the far side
                                           // of the LVDS link. The Alias is different for A and B as that is the
                                           // address on the Console side of the LVDS link. 

            {
                sprintf(myBuffer, "Channel B Serializer Configured\r\n");
                success = 1;
            }
            else
            {
                sprintf(myBuffer, "** Channel B Serializer FAILED !! **\r\n");
                success = 0;
            }

        }

        // Send UART message
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);        
    }
    
    return(success);
}

unsigned int setup_ser_953_tp(unsigned int deser_id)
{  // This function sets up the DS90UB953 Serializer in test pattern mode
      
    unsigned int success = 0;
    uint8_t temp = 0;

    if(deser_id == 1)
    { // Deser #1 for when we have more than one

        if(MIPI_LANES == 4)
        {   // 4 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x02, 0x73); // # MIPI Lanes = 4
        }        
        else if(MIPI_LANES == 2)
        {   // 2 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0x02, 0x53); // # MIPI Lanes = 2
        }
                
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB0, 0x00);          
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x01); // PGEN CTL
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x01); // Enables the Pattern Generator
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x02);  // PGEN CFG
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x33);  // Set for 8 color bars, 3 bytes/block

        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x03); // PGEN CSI DI
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x1E); // YUV422 8 bit = 0x1E; YUV422 10 bit=0x1F; RAW10=0x2B, RGB888=0x24
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x04); // PGEN LINE SIZE 1 
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x0F); // Set to 3,840 (0x0F00) = 1920*2 (1920 pix/row, 2 bytes/pix)
                                                              // Set to 1280 (0x0500)  = 640*2 (640x 2 bytes/pixel)
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x05); // PGEN LINE SIZE 0
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x00); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x06); // PGEN BAR SIZE1        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x01); // Set for 480 (0x1E0) bytes = 240 pixels. 8 bars = 1920 pixels
                                                              // Set for 160 (0xA0) bytes for 640pix wide frame
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x07); // PGEN BAR SIZE 0
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0xE0); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x08); // PGEN ACT LPF1        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x04); // 1080= 0x438  ; For 720=02D0 ; For 480= 0x1E0
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x09); // PGEN ACT LPF0
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x38); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0A); // PGEN TOT LPF1        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x04); // 1080=0x465 (1125) ; For 720=0x041A (1050) ; For 480=525(0x20D)
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0B); // PGEN TOT LPF0
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x65); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0C); // PGEN LINE PD1       
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x0B); // 1080= 0xB90 (2,960)   ; For 720 = 0x0C67  (Line time / 10ns)
                                                              // 480 = 0x0C67
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0D); // PGEN LINE PD0
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x90); 
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0E); // PGEN VBP
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x24); // For 1080= 0x24 (36) ; For 720 = 0x21
        
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB1, 0x0F); // PGEN VFP
        i2c_sendcommand_a1d1(1, I2C_SER953A_W, 0xB2, 0x04); // For 1080 = 0x04 ; For 720 = 0x0A
        
        temp = i2c_readdata_a1d1(0, I2C_SER953A_W, 0x00); // Read Ser addr for check

        if((temp>>2) == I2C_SER953A_W)
        {
            sprintf(myBuffer, "Channel A Serializer Test Pattern Enabled %d MIPI Lanes\r\n", MIPI_LANES);
            success = 1;
        }
        else
        {
            sprintf(myBuffer, "** Channel A Serializer FAILED !! **\r\n");
            success = 0;
        }

    } else if(deser_id == 2)
    {
        // Do Ch B setup here. Same as above but with different aliases for I2C devices.                
        if(MIPI_LANES == 4)
        {   // 4 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x02, 0x73); // # MIPI Lanes = 4
        }        
        else if(MIPI_LANES == 2)
        {   // 2 MIPI Lanes
            i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0x02, 0x53); // # MIPI Lanes = 4
        }
                
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB0, 0x00);          
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x01); // PGEN CTL
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x01); // Enables the Pattern Generator
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x02);  // PGEN CFG
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x33);  // Set for 8 color bars, 3 bytes/block

        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x03); // PGEN CSI DI
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x1E); // YUV422 8 bit = 0x1E; YUV422 10 bit=0x1F; RAW10=0x2B, RGB888=0x24
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x04); // PGEN LINE SIZE 1 
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x0F); // Set to 3,840 (0x0F00) = 1920*2 (1920 pix/row, 2 bytes/pix)
                                                              // Set to 1280 (0x0500)  = 640*2 (640x 2 bytes/pixel)
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x05); // PGEN LINE SIZE 0
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x00); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x06); // PGEN BAR SIZE1        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x01); // Set for 480 (0x1E0) bytes = 240 pixels. 8 bars = 1920 pixels
                                                              // Set for 160 (0xA0) bytes for 640pix wide frame
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x07); // PGEN BAR SIZE 0
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0xE0); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x08); // PGEN ACT LPF1        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x04); // 1080= 0x438  ; For 720=02D0 ; For 480= 0x1E0
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x09); // PGEN ACT LPF0
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x38); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0A); // PGEN TOT LPF1        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x04); // 1080=0x465 (1125) ; For 720=0x041A (1050) ; For 480=525(0x20D)
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0B); // PGEN TOT LPF0
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x65); // See above
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0C); // PGEN LINE PD1       
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x0B); // 1080= 0xB90 (2,960)   ; For 720 = 0x0C67  (Line time / 10ns)
                                                              // 480 = 0x0C67
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0D); // PGEN LINE PD0
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x90); 
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0E); // PGEN VBP
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x24); // For 1080= 0x24 (36) ; For 720 = 0x21
        
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB1, 0x0F); // PGEN VFP
        i2c_sendcommand_a1d1(1, I2C_SER953B_W, 0xB2, 0x04); // For 1080 = 0x04 ; For 720 = 0x0A        

        temp = i2c_readdata_a1d1(0, I2C_SER953B_W, 0x00); // Read Ser addr for check

        if((temp>>2) == I2C_SER953A_W) // Yes this is still the ChA value as it is the 
                                       // I2C ID of the Serializer not the alias of the Serializer
                                       // ID is same for Ch A and B as it is the address on the far side
                                       // of the LVDS link. The Alias is different for A and B as that is the
                                       // address on the Console side of the LVDS link. 

        {
            sprintf(myBuffer, "Channel B Serializer Test Pattern Enabled %d MIPI Lanes\r\n", MIPI_LANES);
            success = 1;
        }
        else
        {
            sprintf(myBuffer, "** Channel B Serializer FAILED !! **\r\n");
            success = 0;
        }
    }

    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);        
    
    return(success);
}

unsigned int setup_ap1302(unsigned int use_ch)
{
    // Bring AP1302 out of reset by raising Serializer IO pin
    // GPIO0 is AP1302_RST, raise this first
    // GPIO1 is AP1302_STBY, raise this second
    // Turn on red LED on the Baseboard as an alive indicator
    
    int temp;
    unsigned int success;
    unsigned int ap1302_checksum = 0;
    
    if(use_ch == USE_CHA)
    {
        //ADS954_GP3_SetHigh(); // Sets PIC output which goes through SerDes
        // GP3 is a funky open-drain pin shared with INTB in Deser. Instead of driving we
        // use the I2C commands to set it internally to the Deser
        i2c_sendcommand_a1d1(1, I2C_DESER954A_W, 0x6F, 0x90);

        // First set low to make sure the thing gets reset
        ADS954_GP0_SetLow(); // This is Ch A AP1302 Reset
        ADS954_GP1_SetHigh(); // This is Ch A AP1302 Standby. Active low
        do_ms_delay(200);      // wait a moment

        // Then bring it out of reset so it is nice and clean
        ADS954_GP0_SetHigh(); // This is Ch A AP1302 Reset
        do_ms_delay(150);      // wait a moment
        ADS954_GP1_SetLow(); // This is Ch A AP1302 Standby
        do_ms_delay(150);      // wait a moment
        
        // Now see if AP1302 is responding to I2C commands
        temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_A_W, 0x0004);  // Reg 0x0004 is the Mnf ID register
        if(temp == 0x0006)  // Expected value is 0x0006
        {
            success_ap1302a = 1;
            sprintf(myBuffer, "Channel A AP1302 Successfully Detected, Loading Patch...\r\n");
            // Send UART message
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);    
            
            // this loads the bootdata to the ap1302
            ap1302_checksum = ap1302_load_patch(use_ch); 
            
            // And initialize the AP1302 default values
            ap1302_init_defaults(use_ch);
            
        }
        else
        {
            success_ap1302a = 0;
            ap1302_checksum = 0;
            sprintf(myBuffer, "** Channel A AP1302 Detection FAILED!! **\r\n");
            // Send UART message
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);    
            
        }
        
        success = success_ap1302a;
        
    } else if(use_ch == USE_CHB)
    {
        //BDS954_GP3_SetHigh(); // Sets PIC output which goes through SerDes. See note in ChA section above.
        i2c_sendcommand_a1d1(1, I2C_DESER954B_W, 0x6F, 0x90);
        BDS954_GP0_SetHigh(); // This is Ch B AP1302 Reset
        do_ms_delay(100);      // wait a moment      // wait a moment
        BDS954_GP1_SetLow(); // This is Ch B AP1302 Standby
        do_ms_delay(100);      // wait a moment
        
        // Now see if AP1302 is responding to I2C commands
        temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_B_W, 0x0004);  // Reg 0x0004 is the Mnf ID register
        if(temp == 0x0006)  // Expected value is 0x0006
        {
            success_ap1302b = 1;
            sprintf(myBuffer, "Channel B AP1302 Successfully Detected, Loading Patch...\r\n");
            // Send UART message
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);    

            // this loads the bootdata to the ap1302
            ap1302_checksum = ap1302_load_patch(use_ch); 
            
            // And initialize the AP1302 default values
            ap1302_init_defaults(use_ch);

        }
        else
        {
            success_ap1302b = 0;
            ap1302_checksum = 0;
            sprintf(myBuffer, "** Channel B AP1302 Detection FAILED!! **\r\n");
            // Send UART message
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);    
        }

        success = success_ap1302b;
    }

    
    return(ap1302_checksum); // Better indicator of success than success
}

void decrement_led_level(uint8_t use_ch)
{
    if(use_ch == USE_CHA)
    {
        if(glLedLevelA > 0)
        {
            glLedLevelA--;

            set_led_level(use_ch);  // Convert level to DAC voltage and set
            
            if(glLedLevelA == 0)
            {                
                // Make sure it's disabled
                CHA_LED_EN_SetLow();
                glLedEnableA = 0; 
            }

        }
    } else if(use_ch == USE_CHB)
    {
        if(glLedLevelB > 0)
        {
            glLedLevelB--;

            set_led_level(use_ch);  // Convert level to DAC voltage and set
            
            if(glLedLevelB == 0)
            {                
                // Make sure it's disabled
                CHB_LED_EN_SetLow();
                glLedEnableB = 0; 
            }
        }        
    }
}

void increment_led_level(uint8_t use_ch)
{
    if(use_ch == USE_CHA)
    {
        // Make sure it's enabled
        CHA_LED_EN_SetHigh();
        glLedEnableA = 1; 
        
        if(glLedLevelA < 5)  // // rcd6/1/20  was 10 
        {       // If it won't go too high increment
            glLedLevelA++;  // Increment LED level
            glLedEnableA = 1; // Make sure it's enabled

            set_led_level(use_ch);  // Convert level to DAC voltage and set
        }
    } else if(use_ch == USE_CHB)
    {
        // Make sure it's enabled
        CHB_LED_EN_SetHigh();
        glLedEnableB = 1;         
        
        if(glLedLevelB < 5)  // rcd6/1/20  was 10
        {            //If it won't go too high increment
            glLedLevelB++;  // Increment LED level

            set_led_level(use_ch);  // Convert level to DAC voltage and set
        }            
    }
}

void set_led_level(uint8_t use_ch)
{
    // LED Current Equation for LTC3112 is:
    // Iout = 1 - (0.625*Vdac) A
    // For example: Vdac of 0V gives 1A
    //              Vdac of 0.8V gives 500mA
    
    unsigned int LedCurrent;
    uint8_t LedLevelLocal = 0;
    
    // LED Current is inversely proportional
    // And not linear.
//    #define LED_LEVEL_0 LED_CURRENT_MAX  // Fully off
//    #define LED_LEVEL_1 0.1 * LED_CURRENT_MAX // Small steps first
//    #define LED_LEVEL_2 0.09 * LED_CURRENT_MAX
//    #define LED_LEVEL_3 0.08 * LED_CURRENT_MAX
//    #define LED_LEVEL_4 0.07 * LED_CURRENT_MAX
//    #define LED_LEVEL_5 0.06 * LED_CURRENT_MAX
//    #define LED_LEVEL_6 0.045 * LED_CURRENT_MAX
//    #define LED_LEVEL_7 0.03 * LED_CURRENT_MAX
//    #define LED_LEVEL_8 0.02 * LED_CURRENT_MAX
//    #define LED_LEVEL_9 0.01 * LED_CURRENT_MAX
//    #define LED_LEVEL_10 0 //

    // This table below allows for lower dimming but at lower
    // dimming the LED driver is causing horizontal line noise
    // This shouldn't be happening because it is supposed to be
    // a steady current driver not a PWM style driver.
    // For now just fixing it by limiting LED current options at
    // the low end to start from what was Level 5.
    #define LED_LEVEL_0 LED_CURRENT_MAX  // Fully off
    #define LED_LEVEL_1 0.95 * LED_CURRENT_MAX 
    #define LED_LEVEL_2 0.9 * LED_CURRENT_MAX
    #define LED_LEVEL_3 0.85 * LED_CURRENT_MAX
    #define LED_LEVEL_4 0.8 * LED_CURRENT_MAX
    #define LED_LEVEL_5 0.75 * LED_CURRENT_MAX
    #define LED_LEVEL_6 0.7 * LED_CURRENT_MAX
    #define LED_LEVEL_7 0.65 * LED_CURRENT_MAX
    #define LED_LEVEL_8 0.6 * LED_CURRENT_MAX
    #define LED_LEVEL_9 0.55 * LED_CURRENT_MAX
    #define LED_LEVEL_10 0.5 * LED_CURRENT_MAX  // Max drive
        
    // Assign desired channel's global value to local variable
    if(use_ch == USE_CHA)
    {
        LedLevelLocal = glLedLevelA;
        sprintf(myBuffer, "Channel A LED Level set to %d\r\n", LedLevelLocal);
    } else if(use_ch == USE_CHB)
    {
        LedLevelLocal = glLedLevelB;    
        sprintf(myBuffer, "Channel B LED Level set to %d\r\n", LedLevelLocal);
    }
    
    switch(LedLevelLocal)
    {
        case 0:
        {
            LedCurrent = LED_LEVEL_0;  // Set voltage to 4095
            CHA_LED_EN_SetLow(); // Turn off LED Driver
            break;
        }
        case 1:
        {
            LedCurrent = LED_LEVEL_0;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            do_ms_delay(10);
            LedCurrent = LED_LEVEL_1;
            break;
        }
        case 2:
        {
            LedCurrent = LED_LEVEL_2;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 3:
        {
            LedCurrent = LED_LEVEL_3;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 4:
        {
            LedCurrent = LED_LEVEL_4;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 5:
        {
            LedCurrent = LED_LEVEL_5;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 6:
        {
            LedCurrent = LED_LEVEL_6;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 7:
        {
            LedCurrent = LED_LEVEL_7;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }        
        case 8:
        {
            LedCurrent = LED_LEVEL_8;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 9:
        {
            LedCurrent = LED_LEVEL_9;
            CHA_LED_EN_SetHigh();  // Turn on LED Driver enable pin
            break;
        }
        case 10:
        {
            LedCurrent = LED_LEVEL_10;
            CHA_LED_EN_SetHigh();
            break;
        }                                
        default :
        {
            LedCurrent = LED_LEVEL_0;
            CHA_LED_EN_SetLow();  // Turn OFF LED Driver enable pin
            break;
        }
    }

    // Set LED to desired current level since it has changed
    update_mcp4726_dac(use_ch, LedCurrent); // Update Dac level for appropriate channel      
    
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);    
    
}

void use_dvi_ch(uint8_t use_dvi)
{
    // This function sets up I2C communication for the specified DVI Channel
    uint8_t temp;

    if(use_dvi == USE_DVIA)
    { // Dedicated Channel A Output
        // RMW en A 
        temp = TCA6416_read_P1(TCA6416_WALL);
        temp = temp | 0x01; // Set LSB
        temp = temp & 0xF9; // Clear other bits so only one is enabled
        TCA6416_write_P1(TCA6416_WALL, temp);
        
    } else if(use_dvi == USE_DVIB)
    { // Switchable DVI Output
        // RMW en AB 
        temp = TCA6416_read_P1(TCA6416_WALL);
        temp = temp | 0x02; // Set LSB+1
        temp = temp & 0xFA; // Clear other bits so only one is enabled
        TCA6416_write_P1(TCA6416_WALL, temp);        
    } else if(use_dvi == USE_DVIAB)
    { // Dedicated Channel B Output
        // RMW en B 
        temp = TCA6416_read_P1(TCA6416_WALL);
        temp = temp | 0x04; // Set LSB+2
        temp = temp & 0xFC; // Clear other bits so only one is enabled
        TCA6416_write_P1(TCA6416_WALL, temp);
    }
}

uint8_t initialize_dvi(uint8_t use_dvi)
{
    int temp;
    uint8_t rev_1B, rev_1C, rev_1D;   
    uint8_t success = 0; // Return 1 if the SIL1136 rev is properly read
    
    // Set I2C to proper channel
        use_dvi_ch(use_dvi); 
        do_ms_delay(100);
    
    // RMW I2C GPIO Extender to bring channel out of reset
        temp = TCA6416_read_P0(TCA6416_WALL);

        if(use_dvi == USE_DVIA)
            temp = temp | 0x01; // Set LSB
        else if(use_dvi == USE_DVIB)
            temp = temp | 0x02; // Set next
        else if(use_dvi == USE_DVIAB)
            temp = temp | 0x04; // Set next    

    // Write to I2C GPIO Extender
        TCA6416_write_P0(TCA6416_WALL, temp);
    
    // Let the chip come out of reset
        do_ms_delay(100);
        
    // Write necessary I2C commands to initialize the DVI chip
        
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0xC7, 0x00); // Enable TPI as per Progr Ref page 13
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0xC7, 0x00); // Enable TPI as per Progr Ref page 13
        do_ms_delay(10);
        
        // Read device revision - see page 10 of SIL1136 Progr Ref
        rev_1B = i2c_readdata_a1d1(0, I2C_SIL1136_W, 0x1B); // Device ID. Exp 0xB4
        rev_1C = i2c_readdata_a1d1(0, I2C_SIL1136_W, 0x1C); // Device Prodn Rev ID Exp 0x20
        rev_1D = i2c_readdata_a1d1(0, I2C_SIL1136_W, 0x1D); // TPI Rev Level Exp 0x30
        do_ms_delay(10);
        
        if(rev_1B == 0xB4)
            success = 1; // Set flag to return from function

        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x1E, 0x00); // Power up xmitter - pg 38        

        //i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x08, 0x60); // Config Input bus - pg 12        
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x08, 0xA0); // Modified to use half speed PCLK and Negative Edge

        // These commands lifted from Lattice Ref Design
        //i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x00, 0x02); // Pixel Clock  setup
        //i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x01, 0x3A);
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x00, 0x01); // Pixel Clock  setup - 74.25MHz
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x01, 0x1D);


        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x02, 0x70); // Frame rate * 100
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x03, 0x17);

        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x04, 0x98); // Pixels
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x05, 0x08);

        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x06, 0x65); // Lines
        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x07, 0x04);

        i2c_sendcommand_a1d1(0, I2C_SIL1136_W, 0x1A, 0x00);  // Default case from Lattice ref code


    // Report on UART
        if(use_dvi == USE_DVIA)
            sprintf(myBuffer, "DVI Output Ch A Initialized \r\n");
        else if(use_dvi == USE_DVIB)
            sprintf(myBuffer, "DVI Output Ch B Initialized \r\n");
        else if(use_dvi == USE_DVIAB)
            sprintf(myBuffer, "DVI Output Ch AB Initialized\r\n");

        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);    

        return(success);
        
}

void uart1_send_buffer(char *Buffer, int BufferLen)
{
#ifndef UART_QUIET_MODE
    unsigned int   numBytes;    
    UART1_TRANSFER_STATUS status ;

    numBytes = 0;

    while( numBytes < BufferLen)
    {
        status = UART1_TransferStatusGet ( ) ;
        if (status & UART1_TRANSFER_STATUS_TX_EMPTY)
        {
            numBytes += UART1_WriteBuffer ( Buffer + numBytes, BufferLen - numBytes )  ;
            if(numBytes < BufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

            
    }
    

#endif
}

void uart_send_welcome(void)
{
    if(UART_VERBOSITY > 0)
    {
    // Send UART Welcome message
    sprintf(myBuffer, "\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Generate and send *** line
    sprintf(myBuffer, "******************************\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);

    // Send first line
    sprintf(myBuffer, "Welcome to New View Surgical  \r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Generate and send *** line
    sprintf(myBuffer, "******************************\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);

    sprintf(myBuffer, "\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void uart_report_versions(int loopcount)
{
    if(UART_VERBOSITY > 0)
    {

    sprintf(myBuffer, "\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Generate and send next line
    sprintf(myBuffer, "CPU Version: %02X.%02X          \r\n", PIC_MAJ_VERSION, PIC_MIN_VERSION);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Generate and send next line
    sprintf(myBuffer, "FPGA Version: %02X.%02X : L:%dms\r\n", gl_fpga_version>>8, (gl_fpga_version&0xFF), (10*loopcount));
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Generate and send next line
    sprintf(myBuffer, "ISP Version: %02X.%02X          \r\n", gl_ap1302_version>>8, (gl_ap1302_version&0xFF));
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    sprintf(myBuffer, "\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void process_input_char( uint8_t *buffer, unsigned int bufLen, unsigned int numBytesRead)
{
    // This function deals with incoming data from the UART
//uint8_t glCommandIndex = 0;         // This is the index of where we are in the current command
//uint8_t glCommandBuffer[MY_BUFFER_SIZE];     // This is a 4 byte buffer where we can assemble commands
//uint8_t glCommandReady = 0;     // This tells us if we have assembled a full 4 byte command in the buffer, ready to execute

    unsigned int faddr;
    unsigned int fdata;
    int temp;
    
    if(numBytesRead == 1)
    {
        switch(buffer[0])  // This is the incoming UART character buffer
        {
            case ASCII_RETURN :
            {
                if(glCommandIndex == 0) // Handles case of repetitive returns
                    glCommandBuffer[0] = ASCII_RETURN;
                
                // This means command is complete. Execute it.
                switch(glCommandBuffer[0])
                {
                   
                    case ASCII_S :
                    {
                        sprintf(myBuffer, ">> UART: Toggle Select for Video MUX \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        toggle_fpga_dviab();
                        break;                        
                    }

#ifdef MIPI_TEST_UART_SW_ENABLE                    
                    case ASCII_M :
                    {
                        sprintf(myBuffer, ">> UART: Turn On MIPI Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0C); // Get the register value
                        temp = temp | 0x03; // Set two low bits to enable test pattern on A and B
                        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0C, temp); 
                        break;                        
                    }

                    case ASCII_m :
                    {
                        sprintf(myBuffer, ">> UART: Turn off MIPI Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0C); // Get the register value
                        temp = temp & 0xFC; // Clear two low bits to disable test pattern on A and B
                        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0C, temp); 
                        break;                
                    }
#endif
                    
                    case ASCII_P :
                    {
                        sprintf(myBuffer, ">> UART: Turn On Serializer Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        setup_ser_953(USE_CHA, 1); // Enable Serializer Test Patter on Channel A
                        break;                        
                    }
                    
                    case ASCII_p :
                    {
                        sprintf(myBuffer, ">> UART: Turn off Serializer Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        setup_ser_953(USE_CHA, 0); // DISable Serializer Test Patter on Channel A
                        break;                
                    }
                    
                    case ASCII_O :
                    {
                        sprintf(myBuffer, ">> UART: Turn On AP1302 Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        ap1302_test_pattern_control(USE_CHA, 1); 
                        break;                        
                    }
                    
                    case ASCII_o :
                    {
                        sprintf(myBuffer, ">> UART: Turn off AP1302 Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        ap1302_test_pattern_control(USE_CHA, 0); 
                        break;                
                    }

                    case ASCII_T :
                    {
                        sprintf(myBuffer, ">> UART: Turn On HDMI Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, 0x0007); 
                        break;                        
                    }
                    
                    case ASCII_t :
                    {
                        sprintf(myBuffer, ">> UART: Turn off HDMI Test Pattern \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, 0x0000); 
                        break;                
                    }

                    case ASCII_r :
                    {
                        sprintf(myBuffer, ">> UART: Reboot FPGA \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        reset_fpga();         // reset FPGA

                        break;                
                    }

                    case ASCII_v :
                    {
                        sprintf(myBuffer, ">> UART: Report Version Numbers \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        uart_report_versions(0); // Set loopcount to 0 to indicate just report, no FPGA load

                        break;                
                    }
                    
                    case ASCII_L :
                    {
                        ap1302_read_debug_log(USE_CHA);
                        break;                
                    }
                    
                    case ASCII_F :
                    {
                        // This is the generic FPGA write function
                        faddr = ((((uint8_t)glCommandBuffer[2])-ASCII_ZERO)*100) + ((((uint8_t)glCommandBuffer[3])-ASCII_ZERO)*10) + (((uint8_t)glCommandBuffer[4])-ASCII_ZERO);
                        fdata = ((((uint8_t)glCommandBuffer[6])-ASCII_ZERO)*10000) + ((((uint8_t)glCommandBuffer[7])-ASCII_ZERO)*1000) + ((((uint8_t)glCommandBuffer[8])-ASCII_ZERO)*100) + ((((uint8_t)glCommandBuffer[9])-ASCII_ZERO)*10) + (((uint8_t)glCommandBuffer[10])-ASCII_ZERO);
                        sprintf(myBuffer, ">> UART: FPGA I2C Write Addr 0x%02X Data 0x%02X%02X \r\n", faddr, (fdata>>8), (fdata&0xFF));
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        i2c_sendcommand_a1d2(1, I2C_FPGA_W, (faddr&0xFF), (fdata&0xFFFF)); // Write to the FPGA. Limit to 8, 16 bits respectively

                        break;                
                    }

                    case ASCII_f :
                    {
                        // This is the generic FPGA read function
                        faddr = ((((uint8_t)glCommandBuffer[2])-ASCII_ZERO)*100) + ((((uint8_t)glCommandBuffer[3])-ASCII_ZERO)*10) + (((uint8_t)glCommandBuffer[4])-ASCII_ZERO);                        

                        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, faddr);
                        
                        sprintf(myBuffer, ">> UART: FPGA I2C Read Addr 0x%02X returned Data 0x%04X \r\n", faddr, temp);
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        break;                
                    }

                    
                    default :
                    {
                        sprintf(myBuffer, "\r\n**************************\r\n\r\n>> UART Command Information \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

#ifdef MIPI_TEST_UART_SW_ENABLE                        
                        sprintf(myBuffer, ">> MIPI Test Pattern On: M <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> MIPI Test Pattern Off: m <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
#endif
                        
                        sprintf(myBuffer, ">> Toggle DVI AB Select: S <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> HDMI Test Pattern On: T <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> HDMI Test Pattern Off: t <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> Reboot FPGA: r <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> FPGA I2C Read: f rrr <enter> (rrr=dec addr) \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> FPGA I2C Write: F rrr ddddd <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        sprintf(myBuffer, ">> AP1302 TP On: O <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> AP1302 TP OFF: o <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> Serializer TP On: P <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> Serializer TP OFF: p <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);


                        sprintf(myBuffer, ">> Report Version Numbers: v <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, ">> Read ChA AP1302 Log: L <enter> \r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);

                        sprintf(myBuffer, "\r\n**************************\r\n\r\n");
                        myBufferLen = (strlen(myBuffer));
                        uart1_send_buffer(myBuffer, myBufferLen);
                        
                        break;                        
                    }

                }
                
                
                glCommandIndex = 0;     // Clear command index since return was pressed
                
                break;                                
            };

            case ASCII_F :
            {
                sprintf(myBuffer, ">> UART: FPGA write format is <f rrr ddddd> all in decimal \r\n");
                myBufferLen = (strlen(myBuffer));
                uart1_send_buffer(myBuffer, myBufferLen);
                
                glCommandBuffer[glCommandIndex] = buffer[0]; // Queue the character
                glCommandIndex++;                            // Increment the index
                
                break;
                
            };
            
            case ASCII_f :
            {
                sprintf(myBuffer, ">> UART: FPGA read format is <f rrr> all in decimal \r\n");
                myBufferLen = (strlen(myBuffer));
                uart1_send_buffer(myBuffer, myBufferLen);
                
                glCommandBuffer[glCommandIndex] = buffer[0]; // Queue the character
                glCommandIndex++;                            // Increment the index
                
                break;
                
            };

            
            default :
            {
                sprintf(myBuffer, ">> UART: Non-Return Char Detected, Queued in Command buffer \r\n");
                myBufferLen = (strlen(myBuffer));
                uart1_send_buffer(myBuffer, myBufferLen);
                
                glCommandBuffer[glCommandIndex] = buffer[0]; // Queue the character
                glCommandIndex++;                            // Increment the index
                
                break;
            };
        };
    }
    
}

void reset_fpga(void)
{
    // This resets the FPGA but doesn't make it reconfigure
    // Existing SPI/BIT file is maintained
    
    FPGA_RESETN_SetLow();  // reset fpga
    do_ms_delay(100);      
    FPGA_RESETN_SetHigh(); // release from reset
}

void ProcessButtonInput(uint8_t use_ch)
{
    // this function deal with user buttons
    // for either channel
            
    // glButtonsA holds current state of Channel A buttons
    // glButtonsA_last holds the previous state of Channel A buttons
    // Based on which channel is being used pull the data into local
    // variables for processing
    int ButtonsTemp = 0;
    int ButtonsTemp_last = 0;
    
    if(use_ch == USE_CHA)
    {
        ButtonsTemp = glButtonsA; // Latch A side values
        ButtonsTemp_last = glButtonsA_last;
        glButtonsA_last = glButtonsA; // Update for next time
    } else if(use_ch == USE_CHB)
    {
        ButtonsTemp = glButtonsB; // Latch B side values
        ButtonsTemp_last = glButtonsB_last;
        glButtonsB_last = glButtonsB; // Update for next time
    }
    
    // Interpret the command the same regardless of which channel it comes from
    // then send the channel id to the function that acts on the input, e.g. brightness_inc()       
    
    if(ButtonsTemp != ButtonsTemp_last)
    { // If something changed act on it
        switch(ButtonsTemp)
        {
            case NO_BUTTON: // Button(s) just released
            {
                // Do whatever we need to do when buttons are released
                // Maybe nothing, maybe reset a flag for the button-press-counter?
                
                // Send UART message
                sprintf(myBuffer, "Channel %d Button(s) released     \r\n", use_ch);
                myBufferLen = (strlen(myBuffer));
                uart1_send_buffer(myBuffer, myBufferLen);

                
                break;
            }
            
            case SECRET_BUTTON: // Button pressed
            {
                button_secret_action(use_ch); // Act on button
                break;
            }            
            
            case BUTTON1: // Button pressed
            {
                button1_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON2: // Button pressed
            {
                button2_action(use_ch); // Act on button
                break;
            }
                        
            case BUTTON3: // Button pressed
            {
                button3_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON4: // Button pressed
            {
                button4_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON5: // Button pressed
            {
                button5_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON6: // Button pressed
            {
                button6_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON7: // Button pressed
            {
                button7_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON8: // Button pressed
            {
                button8_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON9: // Button pressed
            {
                button9_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON10: // Button pressed
            {
                button10_action(use_ch); // Act on button
                break;
            }
            
            case BUTTON11: // Button pressed
            {
                button11_action(use_ch); // Act on button
                break;
            }
                        
            default : // More than one button pressed
            {
                // This is specifically more than one button
                // It is not no buttons, that is handled separately
                // in it's own case.
                // For now do nothing just UART report

                // Send UART message
                sprintf(myBuffer, "Channel %d Invalid Button Combination     \r\n", use_ch);
                myBufferLen = (strlen(myBuffer));
                uart1_send_buffer(myBuffer, myBufferLen);

                break;
            }
        }
    }
    
    
    
}

void button_secret_action(uint8_t use_ch)
{   // this is the secret combination
    // do something cool/useful here

    // Send UART message
    sprintf(myBuffer, "Channel %d Secret Button Pressed     \r\n", use_ch);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);

}

void button1_action(uint8_t use_ch)
{   // This is Zoom Out
    
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    int temp_zoom = glZoomA;
    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_zoom = glZoomB;
    }

    // Zoom out if we aren't already out all the way
    if(temp_zoom > (ZOOM_MIN+ZOOM_STEP))
    {   // More than enough so go a step out
        temp_zoom = temp_zoom - ZOOM_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, temp_zoom);
    } else if(temp_zoom > ZOOM_MIN)
    {   // Just enough so just go to min
        temp_zoom = ZOOM_MIN;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, temp_zoom);
    }
    
    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glZoomA = temp_zoom;
    } else if(use_ch == USE_CHB)
    {
        glZoomB = temp_zoom;
    }            
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 1 Zoom Out set to %04X\r\n", use_ch, temp_zoom);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
    
}

void button2_action(uint8_t use_ch)
{  // This is Zoom In
    int temp_zoom = glZoomA;
        
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_zoom = glZoomB; 
    }

    // Zoom IN if we aren't already IN all the way //RCD changed "out" to "IN"
    if(temp_zoom < (ZOOM_MAX-ZOOM_STEP))
    {   // More than enough so go a step in
        temp_zoom = temp_zoom + ZOOM_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, temp_zoom);
    } else if(temp_zoom < ZOOM_MAX)
    {   // Just enough so just go to max
        temp_zoom = ZOOM_MAX;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, temp_zoom);
    }else if(temp_zoom == ZOOM_MAX) //RCD 6/1/20  Zoom Wrap to min
    {   // Already at ZOOM_MAX
        Zoom_Mx_Cnt++;//RCD 6/18/2020 Extra button push for zoom wrap
        if (Zoom_Mx_Cnt > 1)
        {
        temp_zoom = ZOOM_MIN; //RCD 6/1/20 reset to beginning zoom min 
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, temp_zoom);
        Zoom_Mx_Cnt = 0;
        }
    }


    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glZoomA = temp_zoom;
    } else if(use_ch == USE_CHB)
    {
        glZoomB = temp_zoom;
    }
    
    if(UART_VERBOSITY > 1)
    {
    // Send UART message
    sprintf(myBuffer, "Channel %d Button 1 Zoom In set to %04X\r\n", use_ch, temp_zoom);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    }

}

void button3_action(uint8_t use_ch)
{  // This is Up
        
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    int temp_pany = glPanYA;
    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_pany = glPanYB; 
    }

    // Pan Up if we aren't already out all the way
    if(temp_pany > (PANY_MIN+PANY_STEP))
    {   // More than enough so go a step in
        temp_pany = temp_pany - PANY_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANY_ADDR, temp_pany);
    } else if(temp_pany > PANY_MIN)
    {   // Just enough so just go to min
        temp_pany = PANY_MIN;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANY_ADDR, temp_pany);
    }

    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glPanYA = temp_pany;
    } else if(use_ch == USE_CHB)
    {
        glPanYB = temp_pany;
    }
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 1 Pan Up set to %04X\r\n", use_ch, temp_pany);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void button4_action(uint8_t use_ch)
{  // This is Left
        
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    int temp_panx = glPanXA;
    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_panx = glPanXB; 
    }

    // Pan down if we aren't already out all the way
    if(temp_panx > (PANX_MIN+PANX_STEP))
    {   // More than enough so go a step in
        temp_panx = temp_panx - PANX_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANX_ADDR, temp_panx);
    } else if(temp_panx > PANX_MIN)
    {   // Just enough so just go to min
        temp_panx = PANX_MIN;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANX_ADDR, temp_panx);
    }

    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glPanXA = temp_panx;
    } else if(use_ch == USE_CHB)
    {
        glPanXB = temp_panx;
    }
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 1 Pan Left set to %04X\r\n", use_ch, temp_panx);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void button5_action(uint8_t use_ch)
{  // This is Right
        
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    int temp_panx = glPanXA;
    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_panx = glPanXB; 
    }

    // Pan Right if we aren't already out all the way
    if(temp_panx < (PANX_MAX-PANX_STEP))
    {   // More than enough so go a step in
        temp_panx = temp_panx + PANX_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANX_ADDR, temp_panx);
    } else if(temp_panx < PANX_MAX)
    {   // Just enough so just go to max
        temp_panx = PANX_MAX;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANX_ADDR, temp_panx);
    }

    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glPanXA = temp_panx;
    } else if(use_ch == USE_CHB)
    {
        glPanXB = temp_panx;
    }
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 1 Pan Right set to %04X\r\n", use_ch, temp_panx);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void button6_action(uint8_t use_ch)
{  // This is Down
        
    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    int temp_pany = glPanYA;
    
    if(use_ch == USE_CHB) // Change to B if needed
    {
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        temp_pany = glPanYB; 
    }
    
    // Pan Down if we aren't already out all the way
    if(temp_pany < (PANY_MAX-PANY_STEP))
    {   // More than enough so go a step in
        temp_pany = temp_pany + PANY_STEP;        
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANY_ADDR, temp_pany);
    } else if(temp_pany < PANY_MAX)
    {   // Just enough so just go to max
        temp_pany = PANY_MAX;
        i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANY_ADDR, temp_pany);
    }


    // Update channel's global value
    if(use_ch == USE_CHA)
    {
        glPanYA = temp_pany;
    } else if(use_ch == USE_CHB)
    {
        glPanYB = temp_pany;
    }
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 1 Pan Down set to %04X\r\n", use_ch, temp_pany);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
}


void button7_action(uint8_t use_ch)
{   
    uint8_t level;
    
    // This is Reduce LED
    decrement_led_level(use_ch);
    
    if(use_ch == USE_CHA)
    {
        level = glLedLevelA;
    } else {
        level = glLedLevelB;
    }
    
     // Send UART message
    sprintf(myBuffer, "Channel %d Button 7 Reduce LED Pressed. Set to %d.\r\n", use_ch, level);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
   
}

void button8_action(uint8_t use_ch)
{   // This is LED On/Off Toggle
    if(use_ch == USE_CHA)
    {
        if(glLedEnableA == 1)
        {
            // LED is on, turn it off
            glLedEnableA = 0;
            CHA_LED_EN_SetLow();
        } else {
            // LED is off, turn it on
            glLedEnableA = 1;

            if(glLedLevelA == 0) // If the level is zero set it to 1
                increment_led_level(use_ch);
            
            CHA_LED_EN_SetHigh();            
        }
        sprintf(myBuffer, "Channel %d Button 8 Toggle LED On/Off Pressed. Enable is %d and level is %d.\r\n", use_ch, glLedEnableA, glLedLevelA);
    } else {
        // Channel B
        if(glLedEnableB == 1)
        {
            // LED is on, turn it off
            glLedEnableB = 0;
            CHB_LED_EN_SetLow();            
        } else {
            // LED is off, turn it on
            glLedEnableB = 1;
            
            if(glLedLevelB == 0) // If the level is zero set it to 1
                increment_led_level(use_ch);
            
            CHB_LED_EN_SetHigh();            
        }
        sprintf(myBuffer, "Channel %d Button 8 Toggle LED On/Off Pressed. Enable is %d and level is %d.\r\n", use_ch, glLedEnableB, glLedLevelB);
    }
    
    // Send UART message
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
   
}

void button9_action(uint8_t use_ch)
{   
    uint8_t level = 0;
    
    // This is Increase LED
    increment_led_level(use_ch);    
    
    if(use_ch == USE_CHA)
    {
        level = glLedLevelA;
    } else {
        level = glLedLevelB;
    }

    // Send UART message
    sprintf(myBuffer, "Channel %d Button 9 Increase LED Pressed. Set to %d.\r\n", use_ch, level);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
   
}

void button10_action(uint8_t use_ch)
{   // This is Toggle View
    
    toggle_fpga_dviab();    // Just call it, nothing else needed
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 10 Toggle Screen Pressed     \r\n", use_ch);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
}

void button11_action(uint8_t use_ch)
{   // This is Capture
    
    // Set the DVIAB output to channel pressed
    set_fpga_dviab(use_ch);

    // Close capture relay
    close_capture_relay();  // same thing regardless of channel
    
    // delay
    do_ms_delay(CAPTURE_DELAY);
    
    // Open relay
    open_capture_relay();
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message
        sprintf(myBuffer, "Channel %d Button 11 Capture Pressed     \r\n", use_ch);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
   
}



uint8_t spi_check_id(void) // Returns 1 if ID matches expected 0xEF7016
{
#define MAX 4
    
    uint8_t check_success = 0;
    uint8_t SpiWrDat[MAX];
    uint8_t SpiRdDat[MAX];
    uint8_t temp = 0x00;
    int spi_count = 0;
    SpiWrDat[0] = 0x9F;
    SpiWrDat[1] = 0x00;
    SpiWrDat[2] = 0x00;
    SpiWrDat[3] = 0x00;
        
    
    // Put a pulse on CS to clear the crud
//    SS2OUT_SetHigh();
    do_us_delay(1);    
    SS2OUT_SetLow();

    
    for(spi_count=0;spi_count<MAX;spi_count++)
    {
         temp = SPI2_Exchange8bit( SpiWrDat[spi_count]);
         SpiRdDat[spi_count] = temp;
    }
    
    do_us_delay(1);
    SS2OUT_SetHigh();
    do_us_delay(1);
//    SS2OUT_SetLow(); // Leave SPI Flash enabled
    
    // Check returned values [1:3]
    if((SpiRdDat[1] == 0xEF) & (SpiRdDat[2] == 0x70) & (SpiRdDat[3] == 0x16)) // For Quad mode flash [2] = 0x40
    {
        check_success = 1;
        sprintf(myBuffer, "Flash ID Check Succesful\r\n");
    }
    else
    {
        check_success = 0;
        sprintf(myBuffer, "** Flash ID Check Failed **\r\n");
    }
    
    // Send UART message    
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    return(check_success);

}

void spi_wr_en(void)     // Writes 0x06 to FLASH    
{
    uint8_t temp = 0x00;
    
    // Put a pulse on CS to clear the crud
//    SS2OUT_SetHigh();
    do_us_delay(1);    
    SS2OUT_SetLow();
        
    temp = SPI2_Exchange8bit((uint8_t)0x06);
    
    // CS Pulse high to close transaction
    do_us_delay(1);
    SS2OUT_SetHigh();
    do_us_delay(1);
//    SS2OUT_SetLow(); // Leave SPI Flash enabled    
    
    // Send UART message    
    sprintf(myBuffer, "Flash Write Enable Set\r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
}

uint8_t spi_read_status(uint8_t StatusRegNum)  // Reads StatusReg 1, 2 or 3
{
    uint8_t status = 0x00;
    uint8_t regadd = 0x05; // Default to status reg1
    
    // Put a pulse on CS to clear the crud
//    SS2OUT_SetHigh();
    do_us_delay(1);    
    SS2OUT_SetLow();

    // Setup appropriate read address
    // RegNum1 is default so skip it
    // Will return RegNum1 for RegNum > 3 (invalid))
    if(StatusRegNum == 2)
    {
        regadd = 0x35; // Status reg 2 read address
    } else if(StatusRegNum == 3)
    {
        regadd = 0x15; // Status reg 3 read address
    }

    // Exchange two bytes, first is junk
    status = SPI2_Exchange8bit(regadd);        // This is the command to read status
    status = SPI2_Exchange8bit((uint8_t)0x00); // This is the real status so overwrite previous
    
    // CS Pulse high to close transaction
    do_us_delay(1);
    SS2OUT_SetHigh();
    do_us_delay(1);
//    SS2OUT_SetLow(); // Leave SPI Flash enabled
    
    // Send UART message    
    sprintf(myBuffer, "Flash Status Reg %d has value 0x%02x\r\n", StatusRegNum, status);
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
        
    return(status);
}

void spi_read_unique_ID(void)  // Updates Unique ID in Global Variable
{
    #define UNIQUE_ID_BSIZE 13
    
    uint8_t   myWrBuffer[UNIQUE_ID_BSIZE];
    uint8_t   myRdBuffer[UNIQUE_ID_BSIZE];
    
    myWrBuffer[0] = 0x4B; // Command to read unique ID
    myWrBuffer[1] = 0x00; // Dummy byte
    myWrBuffer[2] = 0x00; // Dummy byte
    myWrBuffer[3] = 0x00; // Dummy byte
    myWrBuffer[4] = 0x00; // Dummy byte
    myWrBuffer[5] = 0x00; // Dummy byte
    myWrBuffer[6] = 0x00; // Dummy byte
    myWrBuffer[7] = 0x00; // Dummy byte
    myWrBuffer[8] = 0x00; // Dummy byte
    myWrBuffer[9] = 0x00; // Dummy byte
    myWrBuffer[10] = 0x00; // Dummy byte
    myWrBuffer[11] = 0x00; // Dummy byte
    myWrBuffer[12] = 0x00; // Dummy byte    
    
    unsigned int    total = 0;    

    // Put a pulse on CS to clear the crud
//    SS2OUT_SetHigh();
    do_us_delay(1);    
    SS2OUT_SetLow();
            
    do
    {
        total  = SPI2_Exchange8bitBuffer( &myWrBuffer[total], UNIQUE_ID_BSIZE - total, &myRdBuffer[total]);

        do_us_delay(1);

    } while( total < UNIQUE_ID_BSIZE );
    
    // CS Pulse high to close transaction
    do_us_delay(1);
    SS2OUT_SetHigh();
    do_us_delay(1);
//    SS2OUT_SetLow(); // Leave SPI Flash enabled
    
    // Send UART message    
    sprintf(myBuffer, "Flash Unique ID is 0x%02X%02X%02X%02X%02X%02X%02X%02X\r\n", (unsigned int)*(myRdBuffer+5),(unsigned int)*(myRdBuffer+6),(unsigned int)*(myRdBuffer+7),(unsigned int)*(myRdBuffer+8),(unsigned int)*(myRdBuffer+9),(unsigned int)*(myRdBuffer+10),(unsigned int)*(myRdBuffer+11),(unsigned int)*(myRdBuffer+12));
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
        
}

void spi_read_buffer(long start_address, uint16_t byteCount, uint8_t *dataReceived)  // Reads Arbitrary Length Buffer
{

    uint8_t dataTransmitted[SPI_READ_BUF_MAX_SIZE];
    unsigned int    total = 0;    
            
    dataTransmitted[0] = 0x03; // Command to read unique ID
    dataTransmitted[1] = (start_address>>16) & 0x00FF; // Address [23:16], should be no higher bits but mask anyway
    dataTransmitted[2] = (start_address>>8) & 0x00FF; // Address [15:8], mask upper bits
    dataTransmitted[3] = start_address & 0x00FF; // Address [7:0], mask upper bits
    dataTransmitted[4] = 0x00; // This is first data read byte so not added to bytecount
    // OK to not initialize further as they get ignored anyway
    
    // Add 4 to ByteCount to account for above 4 byte command
    byteCount = byteCount + 4;    
    
    // Put a pulse on CS to clear the crud
//    SS2OUT_SetHigh();
    do_us_delay(1);    
    SS2OUT_SetLow();
      
    // Loop till the required number of bytes are read
    do
    {
        total  = SPI2_Exchange8bitBuffer( &dataTransmitted[total], byteCount - total, &dataReceived[total]);

        do_us_delay(1);

    } while( total < byteCount );
    
    // CS Pulse high to close transaction
    do_us_delay(1);
    SS2OUT_SetHigh();
    do_us_delay(1);
//    SS2OUT_SetLow(); // Leave SPI Flash enabled
    
//    // Send UART message    
//    sprintf(myBuffer, "Flash Addr 0x%06X Read %d Bytes\r\n", (unsigned int)start_address, total);
//    myBufferLen = (strlen(myBuffer));
//    uart1_send_buffer(myBuffer, myBufferLen);
        
}

void ap1302_test_pattern_control(unsigned int use_ch, unsigned int tp_enable)
{
    // This function enables the AP1302 test pattern
    // Helps for debugging the image sensor->AP1302 interface
    
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    if(use_ch == USE_CHB) // Change to B if needed
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
    
    if(tp_enable == 1)
    {
        // Set AP1302 test pattern to enabled
        i2c_sendcommand_a2d2(1, ap1302_addr, 0x600C, 0x0390);
    } else {
        //i2c_sendcommand_a2d2(1, ap1302_addr, 0x600C, 0x0211); 
        // Doesn't work. OnSemi confirmed this is the case. No way to turn off test pattern
        // via I2C. You have to reinitialize the chip!
        setup_ap1302(use_ch);
    }

}

uint8_t ap1302_load_patch(unsigned int use_ch)
{   // this function loads the AP1302 patch using SPI and I2C
    // returns 1 if the crc matched expected
    
    #define TIMEOUT_MAX 1000
    #define TIMEOUT_MAX_CHECKSUM 40
    
        
    int temp = 0x0;
    int timeout = 0;   
    uint8_t checksum_check = 0;

    uint8_t dataReceived[(SPI_READ_BUF_MAX_SIZE+4)]; // Temporary Storage for data
// [0] = Dummy read byte, useless                    // Added 4 bytes up front for
// [1] = Dummy read byte, useless                    // dummy read bytes for SPI
// [2] = Dummy read byte, then I2C addr MSB          // and then we put the i2c 
// [3] = Dummy read byte, then I2C addr LSB          // register address in [2] and [3]
// [4] = Real Boot data first byte                   // and start I2C xfer at [2]
    
    long int start_address = 0;    // SPI Start address
    int ap1302_addr_ptr = 0;  // Pointer for I2C address of AP1302 load register
    
    long int num_bytes_xferred = 0;  // this many total bytes read from SPI
            
    uint16_t xfer_length = 0;    
    uint16_t i2c_xfer_length = 0;    
    
    uint8_t first_boot_packet = 1;   // Flag for first packet
    
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    if(use_ch == USE_CHB) // Change to B if needed
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
        
//From AP1302 Datasheet
//Boot data content is loaded to basic address 0x8000 onwards. Once the host loads the
//address 0x9FFF, it goes back to address 0x8000 and continues loading from there. Host
//cycle around 0x9FFF->0x8000 until all content is loaded.

//Boot up procedure using bootdata is as follows:
//1. After power up sequence, host should poll for model_id basic register until value
//0x0265 is read.
   
    temp = i2c_readdata_a2d2(1, ap1302_addr, 0x0004);  // Reg 0x0004 is the Mnf ID register, should be 0x06
    do_ms_delay(1);
    do{
        temp = i2c_readdata_a2d2(1, ap1302_addr, 0x0000);  // Reg 0x0000 is the Chip Version ID register, should be 0x0265
        do_ms_delay(1); // wait a moment
        timeout++;      // inc timeout counter
    } while((temp != 0x0265) | (timeout == TIMEOUT_MAX));
    
    // Before starting boot process set Bootdata stage to 0xFFFF
//    i2c_sendcommand_a2d2(1, ap1302_addr, 0x6002, 0xFFFF); 
    
//2. Host loads the number of bytes as specified in ?pll_init_size? attribute and then set
//bootdata_stage basic register to 0x002. This will apply basic_init_hp settings and
//enable PLL.

    
    // Setup the FLASH Address for SPI read
    start_address = num_bytes_xferred; // FLASH addressing starts at 0 so the start address
                            // is just the number of bytes written to this point

    // Length of the read is the PLL_INIT_SIZE since we do packets
    // of 0x1FFF bytes and PLL INIT is only 0x3FF bytes
    xfer_length = PLL_INIT_SIZE; 

    // Read in SPI data starting at index [0]. First 4 bytes are dummy reads        
    spi_read_buffer(start_address, xfer_length, &dataReceived[0]);        

    // Setup I2C read address in first two bytes
    // AP1302 has a base + offset. Base is 0x8000
    // The offset cycles at some point and you start over at the base address
    ap1302_addr_ptr = AP1302_I2C_START; // For PLL_INIT section the offset is 0x0
    dataReceived[2] = (ap1302_addr_ptr>>8) & 0x00FF; // High Byte of address
    dataReceived[3] = ap1302_addr_ptr & 0x00FF; // Low byte of address

    // Finally write the buffer over I2C now that it is setup with address and data
    // We send (xfer_length+2) to account for the extra two bytes of regaddr we just
    // prefixed to the data. We start at [2] since [0] and [1] are dummy reads from SPI
    i2c_xfer_length = xfer_length + 2;
    i2c_sendcommand_dN(1, ap1302_addr, i2c_xfer_length, &dataReceived[2]);                

    // And then update number of bytes read to keep track
    num_bytes_xferred = num_bytes_xferred + xfer_length; 
       
    if(UART_VERBOSITY>2)
    {
        // Send UART message    
        sprintf(myBuffer, "AP1302 PLL: Flash Read Addr 0x%06X Read 0x%04X Bytes\r\n", (unsigned int)start_address, xfer_length);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);   

        // Send UART message    
        sprintf(myBuffer, "AP1302 PLL: I2C Write Address 0x%04X Data Length 0x%04x bytes\r\n", ap1302_addr_ptr, i2c_xfer_length);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
    
    do_ms_delay(1);
    
    // Now that PLL_INIT_SIZE bytes are loaded set Bootdata stage to 2
    i2c_sendcommand_a2d2(1, ap1302_addr, 0x6002, 0x0002); 
    
//3. Host should wait 1ms for PLL to lock.
    do_ms_delay(10); // wait 10ms instead
    
//4. Continue load the rest of bootdata content (starting at pll_init_size and observing the
//wrapping point of 0x9FFF -> 0x8000) until all bootdata is loaded.
    
    // Set LEDs for toggling out of phase
    Ch_RedLed_On(use_ch);
    Ch_GreenLed_Off(use_ch);
    
    do
    {
        // Toggle front panel indicator LEDs every time around
        Ch_GreenLed_Toggle(use_ch);
        Ch_RedLed_Toggle(use_ch);
        
        // Setup the FLASH Address for SPI read
        start_address = num_bytes_xferred; // FLASH addressing starts at 0 so the start address
                                // is just the number of bytes written to this point
        
        // Calculate length of the read, full or partial packet
        if(first_boot_packet == 1)
        {   // First packet starts from (0x8000 + PLL_INIT_SIZE) and goes till 0x9FFF
            xfer_length = (SPI_READ_PKT_SIZE - PLL_INIT_SIZE); // First packet gets us to 0x9FFF
            first_boot_packet = 0;
        }
        else
        {   // Subsequent packets are 2K bytes each till the last which may be partial
            if((num_bytes_xferred + SPI_READ_PKT_SIZE) < (SPI_AP1302_TOTAL_SIZE))
                xfer_length = SPI_READ_PKT_SIZE;  // Full packet
            else
                xfer_length = SPI_AP1302_TOTAL_SIZE-num_bytes_xferred; // Last packet is partial
        }
                
        // Read in SPI data starting at index [0]. First 4 bytes are dummy reads        
        spi_read_buffer(start_address, xfer_length, &dataReceived[0]);        
        
        // Setup I2C read address in first two bytes
        // AP1302 has a base + offset
        // The offset cycles at some point and you start over at the base address
        ap1302_addr_ptr = AP1302_I2C_START + (num_bytes_xferred%AP1302_I2C_CYCLE);
        dataReceived[2] = (ap1302_addr_ptr>>8) & 0x00FF; // High Byte of address
        dataReceived[3] = ap1302_addr_ptr & 0x00FF; // Low byte of address
                
        // Finally write the buffer over I2C now that it is setup with address and data
        // We send (xfer_length+3) to account for the extra two bytes of regaddr we just
        // prefixed to the data, plus an extra one since our address is 0 based. 
        // We start at [2] since [0] and [1] are dummy reads from SPI.
        i2c_xfer_length = xfer_length + 2;
        i2c_sendcommand_dN(1, ap1302_addr, i2c_xfer_length, &dataReceived[2]); 
        
        // And then update number of bytes read to keep track
        num_bytes_xferred = num_bytes_xferred + xfer_length; 
        
        if(UART_VERBOSITY>2)
        {
            // Send UART message    
            sprintf(myBuffer, "AP1302 Boot: SPI Read Addr 0x%06X Read 0x%04X Bytes\r\n", (unsigned int)start_address, xfer_length);
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);

            // Send UART message    
            sprintf(myBuffer, "AP1302 Boot: I2C Write Address 0x%04X Data Length 0x%04X bytes\r\n", ap1302_addr_ptr, i2c_xfer_length);
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);
        }
        
    } while(num_bytes_xferred < (SPI_AP1302_TOTAL_SIZE)); // Stop when SPI_AP1302_TOTAL_SIZE bytes have been sent
            
    do_ms_delay(20);  // Wait a moment
    
        //5. Host can optionally check the sips_crc basic register and compare the value against
//attribute ?crc?. If the read value does not match the value from the attribute, there was
//an error encountered during bootdata load procedure. Host should reset the AP1302
//and repeat the boot up procedure.

#ifdef READ_AP1302_CRC    
    uint8_t crc_check = 0;

    // Read CRC value
    temp = i2c_readdata_a2d2(1, ap1302_addr, 0xF052);  // SIPS_CRC register
    if(temp == AP1302_CRC)
    {
        // CRC match
        crc_check = 1;
        
        // Send UART message    
        sprintf(myBuffer, "Flash -> AP1302 CRC Matched 0x%04X\r\n", temp);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    } else
    {
        // CRC didn't match
        crc_check = 0;
        
        // Send UART message    
        sprintf(myBuffer, " ** Flash -> AP1302 CRC Error ** Expected 0x%04X Got 0x%04X\r\n", AP1302_CRC, temp);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
    
#endif
    
    
//6. Host write the 0xFFFF to bootdata_stage basic register. This indicates to AP1302 that
//the whole bootdata content was loaded.
    // Set Bootdata stage to 0xFFFF
    i2c_sendcommand_a2d2(1, ap1302_addr, 0x6002, 0xFFFF); 
    
    temp = 0x0000; // Clear it
    timeout = (TIMEOUT_MAX_CHECKSUM); // Set it.
    // Now read 0x6002 till it is 0xFFFF
    while((temp != 0xFFFF) & (timeout>0))
    {
        // Toggle front panel indicator LEDs every time around
        Ch_GreenLed_Toggle(use_ch);
        Ch_RedLed_Toggle(use_ch);

        // Delay and then read
        do_ms_delay(100);        
        temp = i2c_readdata_a2d2(1, ap1302_addr, 0x6002);  // SIPS_CRC register        
        
        // Decrement timeout counter
        timeout--;

    }

    if(timeout == 0)
    {
        // Timed out so print fail message
        sprintf(myBuffer, "** Flash -> AP1302 Bootstage Check Timed out **\r\n");
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);

        sprintf(myBuffer, "** Last read bootstage (0x6002) value is 0x%4X **\r\n", temp);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
        
    } else {
        // Boot data stage correct so print pass message        
        sprintf(myBuffer, "Flash -> AP1302 Bootstage Check Passed, Looped %d times\r\n", ((TIMEOUT_MAX>>2)-timeout));
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }
        
    
    // Bootdata stage checksum
    temp = i2c_readdata_a2d2(1, ap1302_addr, 0x6134);  // Bootdata Checksum
    if(temp == AP1302_CHECKSUM)
    {
        // CRC match
        checksum_check = 1;
        
        // Turn on green LED, Red off
        Ch_GreenLed_On(use_ch);
        Ch_RedLed_Off(use_ch);
        
        // Turn off FPGA Test pattern
        turn_off_fpga_out_tp(use_ch); // If displaying this on AB then tp that or toggle to other?
        
        // Send UART message    
        sprintf(myBuffer, "Flash -> AP1302 CHECKSUM Matched 0x%04X\r\n", temp);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    } else
    {
        // CRC didn't match
        checksum_check = 0;
        
        // Turn off green LED, Turn on Red
        Ch_GreenLed_Off(use_ch);
        Ch_RedLed_On(use_ch);
        
        // Send UART message    
        sprintf(myBuffer, " ** Flash -> AP1302 CHECKSUM Error ** Expected 0x%04X Got 0x%04X\r\n", AP1302_CHECKSUM, temp);
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }    

    do_ms_delay(1);
    do_ms_delay(1);  // For breakpoint
    
    // If checksum check passed then set correct output format, otherwise
    // read debug log
    if(checksum_check == 1)
    {
//        i2c_sendcommand_a2d2(1, ap1302_addr, 0x1184, 0x0001);     // Atomic
//        i2c_sendcommand_a2d2(1, ap1302_addr, 0x2012, 0x0030);     // Set to YUV422 for Preview mode
//        i2c_sendcommand_a2d2(1, ap1302_addr, 0x4012, 0x0030);     // Set to YUV422 for Video mode
//        i2c_sendcommand_a2d2(1, ap1302_addr, 0x1184, 0x000B);     // Atomic
//        
//        if(MIPI_LANES == 4)
//        {
//            i2c_sendcommand_a2d2(1, ap1302_addr, 0x2030, 0x0024);     // Set to MIPI 4 lanes, continous clock
//            i2c_sendcommand_a2d2(1, ap1302_addr, 0x2030, 0x0024);     // Set to MIPI 4 lanes, continous clock
//        } else if(MIPI_LANES == 2)
//        {
//            i2c_sendcommand_a2d2(1, ap1302_addr, 0x2030, 0x0022);     // Set to MIPI 2 lanes, continous clock
//            i2c_sendcommand_a2d2(1, ap1302_addr, 0x2030, 0x0022);     // Set to MIPI 2 lanes, continous clock            
//        }
        
        if(UART_VERBOSITY > 2)
        {
            // Send UART message    
            if(MIPI_LANES == 4)
                sprintf(myBuffer, "Proc Output format set to YUV422, MIPI 4 lanes, continous clock\r\n");
            else if (MIPI_LANES == 2)
                sprintf(myBuffer, "Proc Output format set to YUV422, MIPI 2 lanes, continous clock\r\n");
            else
                sprintf(myBuffer, "** Proc Output MIPI Lane Setting Error ** \r\n");
                
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);
        }

    } else 
    {
        // Read the debug log
        ap1302_read_debug_log(use_ch); 
    }
    return(checksum_check);
}

void ap1302_read_debug_log(unsigned int use_ch)
{
    #define LOG_LENGTH 0x200
    #define LOG_ADDR 0xA2C
    
    uint8_t databuffer[LOG_LENGTH]; // Debug log goes from 0x0A2C to 0x0C2C
    
    // Setup correct ap1302 i2c address based on whichever channel we are addressing
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    if(use_ch == USE_CHB) // Change to B if needed
        ap1302_addr = I2C_AP1302ALIAS_B_W;    
    
    // Send UART message    
    sprintf(myBuffer, "AP1302 Reading Debug Log: \r\n");
    myBufferLen = (strlen(myBuffer));
    uart1_send_buffer(myBuffer, myBufferLen);
    
    // Read debug log over i2c
    i2c_readdata_a2dn(1, ap1302_addr, LOG_ADDR, databuffer, LOG_LENGTH);
            
    // Now display log on UART
//    do_ms_delay(10);  // For breakpoint
    
    uart1_send_buffer(databuffer, LOG_LENGTH);
    
    return;
}

void ap1302_init_defaults(unsigned int use_ch)
{
    // This function sets up default values for the AP1302 processor
    // which also controls the defaults for the image sensor

    // Setup ap1302 address for channel we are using
    int ap1302_addr = I2C_AP1302ALIAS_A_W; // Default to Ch A    
    if(use_ch == USE_CHB) // Change to B if needed
        ap1302_addr = I2C_AP1302ALIAS_B_W;    

    i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_ZOOM_ADDR, ZOOM_DEFAULT);
    i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_BRIGHTNESS_ADDR, BRIGHTNESS_DEFAULT);
    i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANX_ADDR, PANX_DEFAULT);
    i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PANY_ADDR, PANY_DEFAULT);
    i2c_sendcommand_a2d2(1, ap1302_addr, AP1302_PAN_SPEED_ADDR, PAN_SPEED);

}

void init_fpga_regs(void)
{
    // This function sets FPGA defaults to desired values
    // Fill in as needed    
    
    set_fpga_dviab(USE_CHA); // Set DVIAB to Channel A
    
}

void set_fpga_dviab(unsigned int use_ch)
{   // Set the AB DVI output to the desired channel
    if(use_ch == USE_CHA)
    {
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0C, 0x0004);  // Set DVIAB out to Ch A
        sprintf(myBuffer, "DVIAB Output Set to Channel A\r\n");
    } else {
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0C, 0x0000);  // Set DVIAB out to Ch B
        sprintf(myBuffer, "DVIAB Output Set to Channel B\r\n");
    }
    
    if(UART_VERBOSITY > 1)
    {
        // Send UART message    
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
    }

}

void toggle_fpga_dviab(void)
{   
    unsigned int temp;
    
    // Read, then invert the view
    temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0C); // Get the register value
    temp = temp & 0x0004; // Mask out [2]
    
    if(temp == 0x0)
    { // Meaning it is set to ch B so change to A
        set_fpga_dviab(USE_CHA);        
    } else
    {
        set_fpga_dviab(USE_CHB);
    }    
    
}

void close_capture_relay(void)
{
    // This function closes the capture relay
    unsigned int temp;
    
    // RMW I2C GPIO Extender to bring channel out of reset
    temp = TCA6416_read_P1(TCA6416_WALL);
    
    // Set [5] to close relay
    temp = temp | 0x10;

    // Write to I2C GPIO Extender
    TCA6416_write_P1(TCA6416_WALL, temp);
    
}

void open_capture_relay(void)
{
    // This function opens the capture relay
    unsigned int temp;
    
    // RMW I2C GPIO Extender to bring channel out of reset
    temp = TCA6416_read_P1(TCA6416_WALL);
    
    // Clear [5] to open relay
    temp = temp & 0xEF;

    // Write to I2C GPIO Extender
    TCA6416_write_P1(TCA6416_WALL, temp);
    
}

void turn_on_fpga_out_tp(uint8_t use_ch)
{   // All tp's ON. This is generated on the output side near the pins to DVI
    
    int temp;
    
    if(use_ch == USE_DVIA)
    {
        // Also check if DVIAB is showing USE_CHA and then switch that as well
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp | 0x01; // Set bit[0] to enable tp on DVIA
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    } else if(use_ch == USE_DVIB)
    {
        // Also check if DVIAB is showing USE_CHA and then switch that as well
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp | 0x02; // Set bit[1] to enable tp on DVIB
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    } else if(use_ch == USE_DVIAB)
    {
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp | 0x04; // Set bit[2] to enable tp on DVIAB
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    }
    
   
}

void turn_off_fpga_out_tp(uint8_t use_ch)
{   // All tp's OFF. This is generated on the output side near the pins to DVI
        int temp;
    
    if(use_ch == USE_DVIA)
    {
        // Also check if DVIAB is showing USE_CHA and then switch that as well
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp & 0xFE; // Clear bit[0] to disable tp on DVIA
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    } else if(use_ch == USE_DVIB)
    {
        // Also check if DVIAB is showing USE_CHA and then switch that as well
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp & 0xFD; // Clear bit[1] to disable tp on DVIB
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    } else if(use_ch == USE_DVIAB)
    {
//        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x01); // Get the register value
        temp = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x0D); // Get the register value
        temp = temp & 0xFB; // Clear bit[2] to disable tp on DVIAB
//        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x01, temp);
        i2c_sendcommand_a1d2(1, I2C_FPGA_W, 0x0D, temp);
    }
    
}