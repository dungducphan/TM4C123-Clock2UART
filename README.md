# Clock2UART: Microsecond Timestamp Logger for TM4C123

## Overview

This project implements a high-precision event timestamp logger for the Texas Instruments TM4C123 microcontroller. It counts microseconds since power-up using a 64-bit counter and outputs the timestamp over UART0 every time a clock pulse is detected on input pin PC4 (Port C, Pin 4).

## Features
- **Microsecond Precision:** Uses a hardware timer to maintain a 64-bit microsecond counter.
- **Event Triggered:** On each rising edge of a clock signal at PC4, the current timestamp is sent over UART0.
- **UART Output:** Timestamps are sent as ASCII decimal numbers, followed by CR+LF (`\r\n`), for easy logging or parsing on a PC.
- **Minimal CPU Usage:** All timing and event handling is interrupt-driven.

## Hardware Requirements
- TI TM4C123 microcontroller (e.g., EK-TM4C123GXL LaunchPad)
- External clock or signal source (0.1 Hz to 1 kHz) connected to PC4
- UART-to-USB adapter or onboard debugger for serial output

## Pinout
- **PC4**: Clock input (rising edge triggers timestamp output)
- **PA0 (U0RX), PA1 (U0TX)**: UART0 (connect PA1 to your serial monitor)

## Usage
1. Build and flash the firmware to your TM4C123 board.
2. Connect your clock signal to PC4.
3. Open a serial terminal (115200 baud, 8N1, no flow control) on your PC.
4. Each time the clock signal rises, you'll see the current microsecond count printed on a new line.

## Code Structure
- **clock2uart.c**: Main application source. Contains all initialization, interrupt handlers, and UART output routines.
- **startup_ccs.c**: Startup and vector table (ensure the GPIOC interrupt points to `GPIOCIntHandler`).
- **clock2uart_ccs.cmd**: Linker script for Code Composer Studio.

## License
MIT License. See the top of `clock2uart.c` for details.

## Author
Dung Duc Phan, August 2025
