/******************************************************************************
 * @file    clock2uart.c
 * @brief   Measures time between rising edges on PC4 and outputs the timestamp
 * (in microseconds) over UART0 and to a 2004A LCD via I2C.
 * Uses TivaWare for TM4C123 MCUs.
 *
 * This program configures the Tiva C Series microcontroller to:
 * - Use Timer0A to count microseconds since boot (us_counter)
 * - Set up PC4 as a digital input with interrupt on rising edge
 * - Print the current microsecond counter value to UART0 each time a rising
 * edge is detected on PC4
 * - Display the current microsecond counter value on a 2004A LCD via I2C0
 * - UART0 is configured for 115200 baud, 8N1, on PA0/PA1
 * - I2C0 is configured on PB2 (SCL) and PB3 (SDA) for the LCD module
 *
 * All main logic is interrupt-driven for accurate timing and low CPU usage.
 *
 * Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
 *
 * This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"

//*****************************************************************************
// Error routine called by driver library if an error is encountered (debug)
//*****************************************************************************
#ifdef DEBUG
/**
 * @brief DriverLib error handler (only used in debug builds)
 * @param pcFilename Source file name where error occurred
 * @param ui32Line   Line number of error
 */
void __error__(char *pcFilename, uint32_t ui32Line) {
    // User can add breakpoint or error handling here
}
#endif

//*****************************************************************************
// Global Variables
//*****************************************************************************
/**
 * @brief Counts the number of triggers (rising edges) received on PC4.
 */
volatile uint32_t g_TriggerCount = 0;

/**
 * @brief Microsecond counter incremented by Timer0A interrupt.
 *
 * This variable holds the number of microseconds since the timer started.
 * It is incremented every 1us by the Timer0A interrupt handler.
 */
volatile uint64_t us_counter = 0;

/**
 * @brief Delay factor for LCD nibble timing.
 *
 * This global variable controls the delay used after sending each nibble to the LCD.
 * It is used as a divisor for SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor).
 * Increase this value for shorter delays, decrease for longer delays.
 * Typical values: 90000 (short), 30000 (medium), 500 (long, ~2ms at 50MHz).
 */
volatile uint32_t g_LCDNibbleDelayFactor = 5000;

//*****************************************************************************
// LCD Definitions
//*****************************************************************************

// Define I2C address for the LCD module (commonly 0x27 or 0x3F)
#define LCD_I2C_ADDR 0x27

// PCF8574 control bits for the LCD (Most common backpack mapping, P0-P3 control, P4-P7 data):
// PCF8574 Pins:   P7 P6 P5 P4 P3 P2 P1 P0
// LCD Connections:D7 D6 D5 D4 BL EN RW RS
#define PCF8574_RS  0x01    // P0 of PCF8574 connected to LCD RS
#define PCF8574_RW  0x02    // P1 of PCF8574 connected to LCD RW
#define PCF8574_EN  0x04    // P2 of PCF8574 connected to LCD EN
#define PCF8574_BL  0x08    // P3 of PCF8574 connected to LCD Backlight

//*****************************************************************************
// Interrupt Service Routines (ISRs)
//*****************************************************************************

/**
 * @brief Timer0A interrupt handler. Increments the microsecond counter.
 *
 * This ISR is triggered every 1 microsecond (based on a 50 MHz system clock).
 * It clears the interrupt flag and increments the global us_counter.
 */
void Timer0AIntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear interrupt flag
    us_counter++;
}

/**
 * @brief Print a 64-bit unsigned integer as a decimal string over UART0.
 *
 * @param value The 64-bit unsigned integer to print.
 *
 * This function converts the given value to a decimal string and sends it
 * character by character over UART0. No leading zeros are printed.
 */
void UARTPrintUint64(uint64_t value) {
    char buf[16]; // 15 digits + null terminator
    int i = 15;
    buf[i] = '\0';
    i--;
    uint64_t temp = value;
    for (; i >= 0; i--) {
        buf[i] = '0' + (temp % 10);
        temp /= 10;
    }
    for (i = 0; i < 15; i++) {
        UARTCharPut(UART0_BASE, buf[i]);
    }
}


//*****************************************************************************
// New: I2C0 Interrupt Handler (defined in startup_ccs.c)
//*****************************************************************************
void I2C0IntHandler(void)
{
    // Clear the I2C0 interrupt flag.
    // For this blocking master example, this handler isn't strictly necessary,
    // but it's good practice to have it defined if enabled in the vector table.
    // If you were using I2C in slave mode or non-blocking master, you would
    // put your I2C communication logic here.
    HWREG(I2C0_BASE + I2C_O_MICR) = I2C_MICR_IC;
}


//*****************************************************************************
// LCD Functions
//*****************************************************************************


/**
 * @brief Writes a byte to the I2C bus for the LCD.
 * @param byte The byte to send over I2C.
 */
void I2C_Write_Byte(uint8_t byte) {
    // Wait until I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));
    // Set slave address for write
    I2CMasterSlaveAddrSet(I2C0_BASE, LCD_I2C_ADDR, false);
    // Specify data to be sent
    I2CMasterDataPut(I2C0_BASE, byte);
    // Start the transfer
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    // Wait for transfer to complete
    while (I2CMasterBusy(I2C0_BASE));
}

/**
 * @brief Sends a nibble (4 bits) to the LCD, with enable pulse.
 * @param nibble The 4-bit data to send (upper or lower nibble of the LCD command/data).
 * This 'nibble' parameter should already be in the upper 4 bits
 * (e.g., for 0x30, pass 0x30; for 0x20, pass 0x20).
 * @param mode   Control mode (0 for command, 1 for data).
 */
void LCD_SendNibble(uint8_t nibble, uint8_t mode) {
    uint8_t pcf_byte = 0;

    // The 'nibble' parameter already contains the 4 data bits (D7-D4) shifted to their
    // correct positions (bits 7-4). These directly map to PCF8574 pins P7-P4.
    pcf_byte = (nibble & 0xF0);

    // Set control bits (RS, RW, Backlight)
    if (mode == 0) { // Command mode (RS = 0)
        pcf_byte &= ~PCF8574_RS; // Ensure RS is low
    } else {         // Data mode (RS = 1)
        pcf_byte |= PCF8574_RS;  // Ensure RS is high
    }
    pcf_byte &= ~PCF8574_RW; // Ensure RW is low (write mode)
    pcf_byte |= PCF8574_BL; // Ensure Backlight is ON

    // Pulse Enable (EN) to latch the data
    I2C_Write_Byte(pcf_byte | PCF8574_EN); // EN high
    SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor); // Small delay for E pulse width
    I2C_Write_Byte(pcf_byte & ~PCF8574_EN); // EN low
    SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor); // Small delay for E pulse width
}

/**
 * @brief Sends a full byte command to the LCD.
 * @param command The 8-bit command to send.
 * IMPORTANT: Sends UPPER nibble first, then LOWER nibble.
 */
void LCD_SendCommand(uint8_t command) {
    // Send UPPER nibble first (RS=0 for command)
    LCD_SendNibble(command & 0xF0, 0x00);
    // Send LOWER nibble second (RS=0 for command)
    LCD_SendNibble((command << 4) & 0xF0, 0x00);
    SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor); // Wait after command
}

/**
 * @brief Sends a full byte of data to the LCD.
 * @param data The 8-bit data (character) to send.
 * IMPORTANT: Sends UPPER nibble first, then LOWER nibble.
 */
void LCD_SendData(uint8_t data) {
    // Send UPPER nibble first (RS=1 for data)
    LCD_SendNibble(data & 0xF0, 0x01);
    // Send LOWER nibble second (RS=1 for data)
    LCD_SendNibble((data << 4) & 0xF0, 0x01);
    SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor); // Wait after data
}

/**
 * @brief Initializes the 2004A LCD display.
 */
void LCD_Init() {
    SysCtlDelay(SysCtlClockGet() / 10); // Wait for power-up (~300ms)

    // Send reset commands in 8-bit mode (first 3 times)
    // Note: These are sent as 4-bit "nibbles" but act as 8-bit commands for init
    LCD_SendNibble(0x30, 0x00); // Function Set (8-bit interface)
    SysCtlDelay(SysCtlClockGet() / 30000); // Longer delay after each nibble
    LCD_SendNibble(0x30, 0x00); // Function Set (8-bit interface)
    SysCtlDelay(SysCtlClockGet() / 30000);
    LCD_SendNibble(0x30, 0x00); // Function Set (8-bit interface)
    SysCtlDelay(SysCtlClockGet() / 30000);

    LCD_SendNibble(0x20, 0x00); // Set to 4-bit interface
    SysCtlDelay(SysCtlClockGet() / 30000);

    LCD_SendCommand(0x28); // Function Set: 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(0x0C); // Display ON, Cursor OFF, Blink OFF
    LCD_SendCommand(0x06); // Entry Mode Set: Increment cursor, no display shift
    LCD_SendCommand(0x01); // Clear Display
    SysCtlDelay(SysCtlClockGet() / 10); // Wait for clear to finish (longer delay)
}

/**
 * @brief Prints a string to the LCD display.
 * @param str Pointer to the null-terminated string to print.
 */
void LCD_PrintString(char* str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

/**
 * @brief GPIO Port C interrupt handler for PC4 (clock input).
 *
 * This ISR is triggered on a rising edge at PC4. It clears the interrupt flag,
 * prints the current microsecond counter value over UART0, and sends CR+LF.
 * It also displays the timestamp on the LCD.
 */
void GPIOCIntHandler(void) {
    // Increment trigger count
    g_TriggerCount++;
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4); // Clear interrupt flag

    // --- UART Output (existing functionality) ---
    UARTPrintUint64(us_counter);               // Print timestamp
    UARTCharPut(UART0_BASE, '\r');            // Carriage return
    UARTCharPut(UART0_BASE, '\n');            // Line feed

    // --- LCD Output (new functionality) ---
    char timestamp_str[16]; // 15 digits + null terminator
    int i = 15;
    timestamp_str[i] = '\0';
    i--;
    uint64_t temp_counter = us_counter;
    for (; i >= 0; i--) {
        timestamp_str[i] = '0' + (temp_counter % 10);
        temp_counter /= 10;
    }

    // Prepare trigger count string
        char trigger_str[16]; // 15 digits + null terminator
        int trig_i = 15;
        trigger_str[trig_i] = '\0';
        trig_i--;
        uint64_t temp_trig = g_TriggerCount;
        for (; trig_i >= 0; trig_i--) {
            trigger_str[trig_i] = '0' + (temp_trig % 10);
            temp_trig /= 10;
        }

    LCD_SendCommand(0x01); // Clear display
    // IMPORTANT: Longer delay required for clear display command to complete (~2ms)
    SysCtlDelay(SysCtlClockGet() / 500); // 50 MHz / 500 = 100,000 cycles, approx 2ms

    // Return home to ensure cursor is at (0,0) and display is not shifted
    LCD_SendCommand(0x02); // Return home
    SysCtlDelay(SysCtlClockGet() / g_LCDNibbleDelayFactor); // Short delay after home

    // Set cursor to the first line, first column
    LCD_SendCommand(0x80);
    LCD_PrintString("Timestamp:");

    // Set cursor to the second line, first column
    LCD_SendCommand(0xC0);
    LCD_PrintString(timestamp_str);

    // Set cursor to the third line, first column
    LCD_SendCommand(0x94); // 0x94 = 3rd line, 1st col for 20x4 LCD
    LCD_PrintString("Triggers:");

    // Set cursor to the fourth line, first column
    LCD_SendCommand(0xD4); // 0xD4 = 4th line, 1st col for 20x4 LCD
    LCD_PrintString(trigger_str);
}

//*****************************************************************************
// Main Application Entry Point
//*****************************************************************************

/**
 * @brief Main function. Initializes peripherals and enters main loop.
 *
 * - Sets system clock to 50 MHz using PLL
 * - Enables and configures GPIOC (PC4), UART0 (PA0/PA1), Timer0A, and I2C0 (PB2/PB3)
 * - Sets up interrupts for PC4 (rising edge) and Timer0A (1us period)
 * - Initializes the LCD display
 * - All main work is done in ISRs; main loop can optionally sleep
 *
 * @return int (never returns)
 */
int main(void) {
    // Set system clock to 50 MHz (16 MHz crystal, PLL, divide by 4)
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable required peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);   // For PC4 input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);   // For UART0 output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);   // UART0 uses PA0/PA1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  // For Timer0A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);    // New: For I2C0 LCD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   // New: For I2C0 (PB2/PB3)

    // Wait for peripherals to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {} // New
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {} // New

    // Configure UART0 pins (PA0 = RX, PA1 = TX)
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Configure I2C0 pins (PB2 = SCL, PB3 = SDA)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // Set PB2 as I2C SCL
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);    // Set PB3 as I2C SDA

    // Initialize I2C0 Master
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false); // false for 100kbps (standard)

    // Configure PC4 as digital input
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Set up interrupt on rising edge for PC4 (external clock input)
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4); // Disable during config
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);   // Clear any prior interrupt
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_RISING_EDGE); // Rising edge
    GPIOIntRegister(GPIO_PORTC_BASE, GPIOCIntHandler); // Register ISR
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);  // Enable interrupt

    // Configure Timer0A for 1us periodic interrupts
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Periodic mode
    TimerLoadSet(TIMER0_BASE, TIMER_A, 50 - 1);      // 50 MHz clock: 50 cycles = 1us
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler); // Register ISR
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Enable timeout interrupt
    TimerEnable(TIMER0_BASE, TIMER_A);               // Start timer

    // Enable global interrupts
    IntMasterEnable();

    // Initialize the LCD display
    LCD_Init();

    // Display initial message on LCD
    LCD_SendCommand(0x80); // Set cursor to home (first line, first column)
    LCD_PrintString("Clock2UART Ready");
    LCD_SendCommand(0xC0); // Set cursor to second line
    LCD_PrintString("Waiting for clock...");

    // Main loop: all work is interrupt-driven
    while (1) {
        // Optionally, enter sleep mode to save power between interrupts
        // SysCtlSleep();
    }
}

