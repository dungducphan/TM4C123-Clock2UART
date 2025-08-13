
/******************************************************************************
 * @file    clock2uart.c
 * @brief   Measures time between rising edges on PC4 and outputs the timestamp
 *          (in microseconds) over UART0. Uses TivaWare for TM4C123 MCUs.
 *
 * This program configures the Tiva C Series microcontroller to:
 *   - Use Timer0A to count microseconds since boot (us_counter)
 *   - Set up PC4 as a digital input with interrupt on rising edge
 *   - Print the current microsecond counter value to UART0 each time a rising
 *     edge is detected on PC4
 *   - UART0 is configured for 115200 baud, 8N1, on PA0/PA1
 *
 * All main logic is interrupt-driven for accurate timing and low CPU usage.
 *
 * Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
 *
 * This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
 *****************************************************************************/

#include <stdint.h>      // Standard integer types
#include <stdbool.h>     // Standard boolean type
#include "inc/hw_types.h"   // TivaWare hardware types
#include "inc/hw_memmap.h"  // Memory map definitions
#include "driverlib/sysctl.h"    // System control (clock, power)
#include "driverlib/gpio.h"      // GPIO functions
#include "driverlib/uart.h"      // UART functions
#include "driverlib/pin_map.h"   // Pin mapping macros
#include "driverlib/interrupt.h" // Interrupt controller
#include "driverlib/timer.h"     // Timer functions


//*****************************************************************************
// Pin definitions for LaunchPad RGB LED (not used in this project, but kept for reference)
#define RED_LED   GPIO_PIN_1   ///< PF1
#define BLUE_LED  GPIO_PIN_2   ///< PF2
#define GREEN_LED GPIO_PIN_3   ///< PF3

//*****************************************************************************
// Error routine called by driver library if an error is encountered (debug only)
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
 * @brief Microsecond counter incremented by Timer0A interrupt.
 *
 * This variable holds the number of microseconds since the timer started.
 * It is incremented every 1us by the Timer0A interrupt handler.
 */
volatile uint64_t us_counter = 0;

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
    char buf[21]; // Buffer to hold digits (max for 64-bit int)
    int i = 0;
    if (value == 0) {
        UARTCharPut(UART0_BASE, '0');
        return;
    }
    // Extract digits in reverse order
    while (value > 0 && i < 20) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }
    // Print digits in correct order
    while (i > 0) {
        UARTCharPut(UART0_BASE, buf[--i]);
    }
}

/**
 * @brief GPIO Port C interrupt handler for PC4 (clock input).
 *
 * This ISR is triggered on a rising edge at PC4. It clears the interrupt flag,
 * prints the current microsecond counter value over UART0, and sends CR+LF.
 */
void GPIOCIntHandler(void) {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4); // Clear interrupt flag
    UARTPrintUint64(us_counter);               // Print timestamp
    UARTCharPut(UART0_BASE, '\r');            // Carriage return
    UARTCharPut(UART0_BASE, '\n');            // Line feed
}

//*****************************************************************************
// Main Application Entry Point
//*****************************************************************************

/**
 * @brief Main function. Initializes peripherals and enters main loop.
 *
 * - Sets system clock to 50 MHz using PLL
 * - Enables and configures GPIOC (PC4), UART0 (PA0/PA1), and Timer0A
 * - Sets up interrupts for PC4 (rising edge) and Timer0A (1us period)
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

    // Wait for peripherals to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    // Configure UART0 pins (PA0 = RX, PA1 = TX)
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

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

    // Main loop: all work is interrupt-driven
    while (1) {
        // Optionally, enter sleep mode to save power between interrupts
        // SysCtlSleep();
    }
}
