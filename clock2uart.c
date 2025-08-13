//*****************************************************************************
//
// project0.c - Example to demonstrate minimal TivaWare setup
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

//*****************************************************************************
//
// Define pin to LED color mapping.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Project Zero (project0)</h1>
//!
//! This example demonstrates the use of TivaWare to setup the clocks and
//! toggle GPIO pins to make the LED's blink. This is a good place to start
//! understanding your launchpad and the tools that can be used to program it.
//
//*****************************************************************************

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************


volatile uint64_t us_counter = 0;

// Timer0A interrupt handler: increments microsecond counter
void Timer0AIntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    us_counter++;
}

// Helper: print uint64_t as decimal string over UART0
void UARTPrintUint64(uint64_t value) {
    char buf[21]; // Enough for 64-bit int
    int i = 0;
    if (value == 0) {
        UARTCharPut(UART0_BASE, '0');
        return;
    }
    while (value > 0 && i < 20) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }
    // Print in reverse
    while (i > 0) {
        UARTCharPut(UART0_BASE, buf[--i]);
    }
}

// Interrupt handler for PC4 (clock input)
void GPIOCIntHandler(void) {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
    UARTPrintUint64(us_counter);
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
}


int main(void) {
    // Set system clock to 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Enable peripherals: GPIOC for input, UART0 for output, Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // UART0 uses PA0/PA1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    // Configure UART0 pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Configure PC4 as input
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Set up interrupt on rising edge for PC4
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    GPIOIntRegister(GPIO_PORTC_BASE, GPIOCIntHandler);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Configure Timer0A for 1us periodic interrupts
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // 50 MHz clock, so 1us = 50 cycles
    TimerLoadSet(TIMER0_BASE, TIMER_A, 50 - 1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Enable processor interrupts
    IntMasterEnable();

    // Loop forever, all work done in interrupts
    while(1) {
        // Optionally, enter sleep mode to save power
        // SysCtlSleep();
    }
}
