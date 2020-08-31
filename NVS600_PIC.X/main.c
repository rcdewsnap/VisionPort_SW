/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system intialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75.1
        Device            :  PIC24EP256MC204
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB 	          :  MPLAB X v5.05
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/spi2.h"
#include "helper_functions.h"

/* GLOBAL VARIABLES*/
uint8_t glLedLevelA = 0;
uint8_t glLedLevelB = 0;
uint8_t glLedEnableA = 0;
uint8_t glLedEnableB = 0;
uint8_t TempAlert = 0; // High if temp alert has been triggered by I2C temp chip
int glPCBTemp = 0;  
uint32_t TempCheckThresh = 0; // Make it check the first time, then periodically
uint32_t led_count = 0; // Stores LED toggle delay count
uint8_t CHA_Detect = 0; // Stores current value of CHA Detect pin
uint8_t last_CHA_Detect = 1; // Stores previous value of CHA Detect pin
uint8_t CHB_Detect = 0; // Stores current value of CHB Detect pin
uint8_t last_CHB_Detect = 1; // Stores previous value of CHB Detect pin
uint8_t CHA_Lock = 0;
uint8_t CHB_Lock = 0;

uint16_t gl_fpga_version = 0xFFFF;   // Global to store FPGA version
uint16_t gl_ap1302_version = 0; // Global to store AP1302 version

uint8_t glDbgLedIsoRed = 0; // Off
uint8_t glDbgLedIsoGreen = 0; // Off
uint8_t glDbgLedWallRed = 0; // Off
uint8_t glDbgLedWallGreen = 0; // Off

uint8_t glCommandIndex = 0;         // This is the index of where we are in the current command
uint8_t glCommandBuffer[MY_BUFFER_SIZE];     // This is a 4 byte buffer where we can assemble commands
uint8_t glCommandReady = 0;     // This tells us if we have assembled a full 4 byte command in the buffer, ready to execute

uint8_t glSerialNumber[6]; // For EUI-48 from AT24MAC EEP addr 0x9A to 0x9F

char myBuffer[MY_BUFFER_SIZE] = "Welcome to New View Surgical  \r\n";
uint8_t myBufferLen;

uint8_t ReadBuffer[MY_BUFFER_SIZE] = "BLAH BLAH BLAH BLAH BLAH BLAH  \r\n";
unsigned int ReadBufferLen;

uint8_t success_dvia = 0; // 1 if succesfully initialized
uint8_t success_dvib = 0;
uint8_t success_dviab = 0;

uint8_t success_ap1302a = 0;
uint8_t success_ap1302b = 0;

uint8_t ChA_Low8 = 0;
uint8_t ChA_High8 = 0;
uint8_t ChB_Low8 = 0;   
uint8_t ChB_High8 = 0;
uint8_t ChA_I2C_Int;
uint8_t ChB_I2C_Int;

// Holds button state for each channel
int glButtonsA = 0;
int glButtonsA_last = 0xFFFF;
int glButtonsB = 0;
int glButtonsB_last = 0xFFFF;

// Global Variables for imaging parameters for each channel
int glZoomA = ZOOM_MIN;       // Zoom Step Default to Min Zoom
int glPanXA = PANX_DEFAULT; // Start at center pan value, range from 0x0000 (left) to 0x0100 (right)
int glPanYA = PANY_DEFAULT; // Start at center pan value, range from 0x0000 (top) to 0x0100 (bottom)
int glZoomB = ZOOM_MIN;       // Zoom Step Default to Min Zoom
int glPanXB = PANX_DEFAULT; // Start at center pan value, range from 0x0000 (left) to 0x0100 (right)
int glPanYB = PANY_DEFAULT; // Start at center pan value, range from 0x0000 (top) to 0x0100 (bottom)

int glMuxOutCh = USE_CHA;   // Start muxed output at Channel A

/*
                         Main application
 */
int main(void)
{

    // Local variables
    uint8_t use_ch = 0;
    char myBufferHB[MY_BUFFER_SIZE];
    uint8_t myBufferLenHB;
    uint8_t ChA_First_Time = 1;  // First time flag
    uint8_t ChB_First_Time = 1;  // First time flag
    
    uint8_t spi_status1 = 0xAB;
    uint8_t spi_status2 = 0xCD;
    uint8_t spi_status3 = 0xEF;
    uint8_t spi_id_success = 0xFF;\

    int temp = 0;
    int rcd_tempA = 0;
    int rcd_tempB = 0;
    int loopcount = 0;
    
    unsigned int numBytesRead = 0;
    
    // Disable software watchdog timer
    //WDT_WatchdogtimerSoftwareDisable();
    WATCHDOG_TimerSoftwareDisable();
        
    // initialize the device
    SYSTEM_Initialize();    
    
    // Setup GPIO to Initial conditions
    PIC_LED1_SetHigh();    // Turn on the green LED
    I2CIO_RST_W_SetHigh(); // Bring wall side I2C GPIO Extender out of reset
    I2CIO_RSTP_SetHigh();  // Bring isolated I2C GPIO extender out of reset
    SS2OUT_SetHigh();      // Set the SPI Flash to Enabled (Active low)
    CHA_LED_EN_SetLow();   // Turn off LED for sure
    CHB_LED_EN_SetLow();   // Turn off LED for sure
    glLedEnableA = 0;      // Keep track of LED state
    glLedEnableB = 0;      // Keep track of LED state
    CHA_PWREN_SetLow();    // Make sure power is low for reboot situations
    CHB_PWREN_SetLow();    // Make sure power is low for reboot situations
    
    // Bring FPGA out of reset and let it boot so we can read version number
    FPGA_RESETN_SetHigh();
    
    // Send welcome message
    uart_send_welcome();
    
    // Read FPGA version, loop till valid !=0xFFFF)
    loopcount = 0; // Count loops
    do
    {
        do_ms_delay(10); // 10ms delay for FPGA to boot
        gl_fpga_version = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x00);  // Reg 0x00 in FPGA is the version register
        loopcount++;
    } while((gl_fpga_version == 0) && (loopcount < 100));
    
    if(loopcount == 100)
    {
        gl_fpga_version = 0xFAFA; // Indicates FPGA didn't load
    } else {    
        do_ms_delay(10);
        gl_fpga_version = i2c_readdata_a1d2(1, I2C_FPGA_W, 0x00);  // Do one extra read to make sure we didn't get a partial in the loop
        init_fpga_regs();
    }
    
    // Read AP1302 Version
    gl_ap1302_version = AP1302_CHECKSUM; // Use AP1302 Checksum as the version number
    
    // Initialize UART Receive Buffer Length   
    ReadBufferLen = sizeof(ReadBuffer)/sizeof(uint8_t);    
    
    // Send Welcome message over UART    
    uart_report_versions(loopcount);
    
    // Put FPGA back in reset as toggling IO to the DVI chips before they are ready may screw them up
    FPGA_RESETN_SetLow();
            
    // Initialize string for heartbeat message
    sprintf(myBufferHB, "System is Alive!\r\n");
    myBufferLenHB = (strlen(myBufferHB));    
    
    // Initialize I2C IO Extenders
    TCA6416_Initialize(TCA6416_ISO);  
    TCA6416_Initialize(TCA6416_WALL);        
    
    // Read from SPI
    spi_id_success = spi_check_id();
    
    spi_status1 = spi_read_status(0x01);
    spi_status2 = spi_read_status(0x02);
    spi_status3 = spi_read_status(0x03);
    
    spi_read_unique_ID();

    uint8_t dataReceived[SPI_READ_BUF_MAX_SIZE];
    long SPI_start_address = 0;    
    uint16_t SPI_byteCount = SPI_READ_PKT_SIZE; // Must be less than SPI_READ_BUF_MAX_SIZE
        
    spi_read_buffer(SPI_start_address, SPI_byteCount, &dataReceived[0]);  // Reads Arbitrary Length Buffer
    
    do_ms_delay(1); // Finish UART xfer
    do_us_delay(1); // For a breakpoint
    
    // Initialize temp measurement and report it over the UART
    set_mcp9800_temp_limit(PCB_TEMP_ON_LIMIT,PCB_TEMP_OFF_LIMIT); // Set limit as desired    
    
    // Setup LED Current Control DACs and report
    setup_mcp4726_dac(USE_CHA);
    setup_mcp4726_dac(USE_CHB);
   
    // Reset FPGA
    reset_fpga();
    
    // Enable test pattern on DVI outputs
    turn_on_fpga_out_tp(USE_DVIA);
    turn_on_fpga_out_tp(USE_DVIB);
    turn_on_fpga_out_tp(USE_DVIAB);
    
    // Initialize DVI Output chips
    success_dvia = initialize_dvi(USE_DVIA);
    do_ms_delay(10);
    success_dvib = initialize_dvi(USE_DVIB);
    do_ms_delay(10);
    success_dviab = initialize_dvi(USE_DVIAB);
    do_ms_delay(10);    
   
            
    // Enable power to Channel A and B
    CHA_PWREN_SetHigh();  // Turn on power to Channel A
    do_ms_delay(1);     // Wait a moment
    if(CHA_PG_GetValue() != 1) // Check pg signal
    {
        // Send error message on UART or blink red LED or whatever
        sprintf(myBuffer, "Ch A Power Test FAIL!!\r\n");
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
        while(1); // for a breakpoint
    } else {
            sprintf(myBuffer, "Ch A Power Good\r\n");
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);
    }
    
    CHB_PWREN_SetHigh();  // Turn on power to Channel B
    do_ms_delay(1);     // Wait a moment
    if(CHB_PG_GetValue() != 1) // Check pg signal
    {
        // Send error message on UART or blink red LED or whatever
        sprintf(myBuffer, "Ch B Power Test FAIL!!\r\n");
        myBufferLen = (strlen(myBuffer));
        uart1_send_buffer(myBuffer, myBufferLen);
        while(1); // for a breakpoint
    } else {
            sprintf(myBuffer, "Ch B Power Good\r\n");
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);
    }
    
    // Setup front panel LEDs to correct status (both red)
    // Setup front panel LEDs
    Ch_RedLed_On(USE_CHA);
    Ch_GreenLed_Off(USE_CHA);
    Ch_RedLed_On(USE_CHB);
    Ch_GreenLed_Off(USE_CHB);

       
    while (1)
    {
        
        // Toggle LED
        if(led_count == 0)  // If led_count expired
        {
            PIC_LED1_Toggle();  // Toggle the LED
            led_count = LED_DCOUNT_THRESH; // Reset to this value, this sets toggle delay
            
            // Flash Debug LED Red
            if(glDbgLedIsoRed == 1)
            {
                DbgLedISORedOff();
                DbgLedISOGreenOn();
                DbgLedWallRedOff();
                DbgLedWallGreenOn();
            }                        
            else
            {
                DbgLedISORedOn();
                DbgLedISOGreenOff();
                DbgLedWallRedOn();
                DbgLedWallGreenOff();
            }
            
            if(UART_VERBOSITY>0)
            {                    
                // Send UART Heartbeat Message
                uart1_send_buffer(myBufferHB, myBufferLenHB);
            }

            // Also report frame count and average if either CH is detected            
            if(UART_VERBOSITY>1)
            {
                if(CHA_Detect == 1) // from last go around so there is a little lag, this is OK.
                {
                    temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_A_W, 0x0002);  // Reg 0x0002 is frame count
                    if(temp - rcd_tempA == 0)
                    {
                       asm("reset");
                    }                    
                    rcd_tempA = temp;
                                        
                    sprintf(myBuffer, "CHA AP1302 Frame count is %d \r\n", temp);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                    temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_A_W, 0x0018);  // Reg 0x0018 is Red Average
                    sprintf(myBuffer, "CHA AP1302 Red Average is 0x%04X \r\n", temp);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                }

                // Could be both so no else
                if(CHB_Detect == 1) // from last go around so there is a little lag, this is OK.
                {
                    temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_B_W, 0x0002);  // Reg 0x0002 is frame count
                    if(temp - rcd_tempB == 0)
                    {
                       asm("reset");
                    }                    
                    rcd_tempB = temp;
                                       
                    sprintf(myBuffer, "CHB AP1302 Frame count is %d \r\n", temp);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                    temp = i2c_readdata_a2d2(1, I2C_AP1302ALIAS_B_W, 0x0018);  // Reg 0x0018 is Red Average
                    sprintf(myBuffer, "CHB AP1302 Red Average is 0x%04X \r\n", temp);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                }
            }
            
        }
        led_count--; // Decrement counter
                
        // Check Temp Alert Pin
        TempAlert = Temp_Alert_GetValue(); // Read temp alert pin
        if(TempAlert == 0)  // It's active low
        {   // If flag is set check temp now
            // Send UART message about temp
            glPCBTemp = read_mcp9800_temp();
            sprintf(myBuffer, "PCB Temperature Alert!! Temp is %d \r\n", glPCBTemp);
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);

        } else if(TempCheckThresh == 0)
        {   // Otherwise check when counter expires, no need to check every time
#ifndef I2C_QUIET_MODE
            glPCBTemp = read_mcp9800_temp();
            sprintf(myBuffer, "PCB Temperature is %d C\r\n", glPCBTemp);
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);

#endif
            TempCheckThresh = TEMP_REPORT_THRESH;
        }
        TempCheckThresh--; // Decrement the count

#ifdef DESER_TEST_MODE
        // In Deser test mode just initialize both channels regardless of head status
        // Do take first time into account as we don't want to do this needlessly
        
        if(ChA_First_Time == 1)
        {
            Channel_Initialize(USE_CHA);  // Init Ch A
            ChA_First_Time = 0;           // Clear First time flag
            
            Channel_Initialize(USE_CHB);  // Init Ch B as well
            ChB_First_Time = 0;           // Clear First time flag
            
            // Now bring FPGA out of reset and let it boot
            FPGA_RESETN_SetHigh();
            
            sprintf(myBuffer, "\r\n\r\n *** THIS IS DESER TEST MODE. NO CAMERA IMAGE *** C\r\n\r\n");
            myBufferLen = (strlen(myBuffer));
            uart1_send_buffer(myBuffer, myBufferLen);

        }
                
#else
        // Check if CHA Detect needs action
        CHA_Detect = CHA_DETECT_GetValue(); // Get current value
        if((last_CHA_Detect != CHA_Detect) | (ChA_First_Time == 1))
        {
            use_ch = USE_CHA;  // Set current channel
            ChA_First_Time = 0;  // Clear first time flag
            
            if(CHA_Detect == 1)
            {
                // Enter here if CHA was just plugged in
                Channel_Initialize(use_ch);
            } else {
                // Enter if CHA was just UNplugged or first time
                Channel_Release(use_ch);                
            }
        }
        last_CHA_Detect = CHA_Detect; // Save last loop's value for next time
        
        // Check if CHB Detect needs action
        CHB_Detect = CHB_DETECT_GetValue(); // Get current value
        if((last_CHB_Detect != CHB_Detect) | (ChB_First_Time == 1))
        {
            use_ch = USE_CHB;  // Set current channel
            ChB_First_Time = 0;  // Clear flag
            
            if(CHB_Detect == 1)
            {
                // Enter here if CHB was just plugged in
                Channel_Initialize(use_ch);
            } else {
                // Enter if CHB was just UNplugged
                Channel_Release(use_ch);                
            }
        }
        last_CHB_Detect = CHB_Detect; // Save last loop's value for next time
        
        // If ChA is detected and locked then check if Ch A Control Pad Interrupt Fired
        CHA_Lock = ADS954_LOCK_GetValue();
        if(CHA_Detect & CHA_Lock)
        {
            ChA_I2C_Int = CHA_POD_I2C_INT_GetValue(); // Get Interrupt pin value
            if(ChA_I2C_Int == 0x00) // Low means it fired
            {
                // Button pressed. Read the thing
                ChA_Low8 = TCA6416_Pad_read_P0(USE_CHA);
                ChA_High8 = TCA6416_Pad_read_P1(USE_CHA);

                // Track changes and do something here
                glButtonsA_last = glButtonsA;           // Save last time's value
                glButtonsA = ((ChA_High8 & 0x07)<<8) + ChA_Low8; // Update new value            

                if(UART_VERBOSITY > 2)
                {
                    // Report to UART
                    sprintf(myBuffer, "Ch A Button Interrupt. Code = %04x \r\n", glButtonsA);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                }

                // Handle Button input
                ProcessButtonInput(USE_CHA);
            }
        }
        
        // Check if Ch B Control Pad Interrupt Fired        
        CHB_Lock = BDS954_LOCK_GetValue();
        if(CHB_Detect & CHB_Lock)
        {
            if(!CHB_POD_I2C_INT_GetValue())
            {
                // Button pressed. Read the thing
                ChB_Low8 = TCA6416_Pad_read_P0(USE_CHB);
                ChB_High8 = TCA6416_Pad_read_P1(USE_CHB);

                // Track changes and do something here
                glButtonsB_last = glButtonsB;           // Save last time's value
                glButtonsB = ((ChB_High8 & 0x07)<<8) + ChB_Low8; // Update new value            

                if(UART_VERBOSITY>2)
                {
                    // Report to UART
                    sprintf(myBuffer, "Ch B Button Interrupt. Code = %04x \r\n", glButtonsB);
                    myBufferLen = (strlen(myBuffer));
                    uart1_send_buffer(myBuffer, myBufferLen);
                }

                // Handle Button input
                ProcessButtonInput(USE_CHB);
                // do_ms_delay(1); // For a breakpoint
            }
        }
#endif
      
        
        // Check if there is anything incoming on the UART
        if(!UART1_ReceiveBufferIsEmpty())
        {
            // Read from receive buffer
            numBytesRead = UART1_ReadBuffer( ReadBuffer, ReadBufferLen);

            // Deal with received character
            process_input_char(ReadBuffer, ReadBufferLen, numBytesRead);
            
            do_ms_delay(1); // For a breakpoint            
            
        }        
        
    }

    return 1;
}
/**
 End of File
*/

