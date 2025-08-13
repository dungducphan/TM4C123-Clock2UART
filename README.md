# ⏰🔗 Clock2UART: Microsecond Timestamp Logger for TM4C123 🚀

---

## ✨ Overview

Welcome to **Clock2UART**! This project transforms your TM4C123 microcontroller into a high-precision event logger, outputting microsecond timestamps over UART every time a clock pulse is detected on PC4. Perfect for timing, debugging, and embedded experiments! 😎

---

## ⚡ Features

- 🕒 **Microsecond Precision:** Hardware timer maintains a 64-bit microsecond counter.
- 🎯 **Event Triggered:** Each rising edge on PC4 sends the current timestamp over UART0.
- 📨 **UART Output:** Timestamps are sent as ASCII decimal numbers, followed by CR+LF (`\r\n`) for easy logging or parsing.
- 💤 **Minimal CPU Usage:** All timing and event handling is interrupt-driven for efficiency.

---

## 🛠️ Hardware Requirements

- 🧑‍💻 TI TM4C123 microcontroller (e.g., EK-TM4C123GXL LaunchPad)
- ⏲️ External clock or signal source (0.1 Hz to 1 kHz) connected to PC4
- 🔌 UART-to-USB adapter or onboard debugger for serial output
- 💡 Optional: 2004A LCD module with PCF8574 I2C backpack for real-time display

---

## 🧩 Pinout

| Pin         | Function                        | Notes                        |
|-------------|---------------------------------|------------------------------|
| 🟢 **PC4**  | Clock input (rising edge)       | Triggers timestamp output    |
| 🟡 **PA0**  | UART0 RX                        | Connect to serial monitor    |
| 🟠 **PA1**  | UART0 TX                        | Connect to serial monitor    |
| 🟣 **PB2**  | I2C0 SCL                        | Connect to LCD PCF8574 SCL   |
| 🟡 **PB3**  | I2C0 SDA                        | Connect to LCD PCF8574 SDA   |

---

## 🚦 Usage

1. 🛠️ **Build and flash** the firmware to your TM4C123 board.
2. 🔗 **Connect** your clock signal to PC4.
3. 🖥️ **Open a serial terminal** (115200 baud, 8N1, no flow control) on your PC.
4. ⏱️ **Watch the magic!** Each rising edge on PC4 prints the current microsecond count on a new line.
5. (Optional) **Connect an LCD** as per the hardware setup to view timestamps and trigger counts in real-time.

> 💡 **Tip:** Use any serial terminal (e.g., PuTTY, Tera Term, minicom) to view the output.

---

## 📁 Code Structure

- `clock2uart.c` — Main application source. All initialization, interrupt handlers, and UART output routines.
- `startup_ccs.c` — Startup and vector table (ensure the GPIOC interrupt points to `GPIOCIntHandler`).
- `clock2uart_ccs.cmd` — Linker script for Code Composer Studio.

---

## 📜 License

MIT License. See the top of `clock2uart.c` for details.

---

## 👨‍💻 Author

Dung Duc Phan, August 2025

---

<div align="center">
	<sub>Made with ❤️ for embedded developers and tinkerers everywhere!</sub>
</div>

---

## 🖥️ New Feature: LCD Display

### Overview
This feature adds support for displaying timestamps and trigger counts on a 2004A LCD module with a PCF8574 I2C controller. The LCD provides a real-time view of the microsecond counter and the number of rising edges detected on PC4.

### Hardware Setup
- **LCD Module:** 2004A with PCF8574 I2C backpack.
- **Connections:**
  - PB2 (I2C0 SCL) -> SCL on PCF8574
  - PB3 (I2C0 SDA) -> SDA on PCF8574
  - Ensure the I2C address matches the default `0x27` or update the code if different.

### Usage
1. Connect the LCD module to the TM4C123 board as described above.
2. Build and flash the firmware.
3. Power on the system. The LCD will initialize and display:
   - The current microsecond counter value.
   - The total number of rising edges detected on PC4.

> **Note:** The LCD backlight is enabled by default. Adjust the `PCF8574_BL` definition in the code to disable it if needed.

---
