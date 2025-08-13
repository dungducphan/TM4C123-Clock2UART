
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

---

## 🧩 Pinout

| Pin         | Function                        | Notes                        |
|-------------|---------------------------------|------------------------------|
| 🟢 **PC4**  | Clock input (rising edge)       | Triggers timestamp output    |
| 🟡 **PA0**  | UART0 RX                        | Connect to serial monitor    |
| 🟠 **PA1**  | UART0 TX                        | Connect to serial monitor    |

---

## 🚦 Usage

1. 🛠️ **Build and flash** the firmware to your TM4C123 board.
2. 🔗 **Connect** your clock signal to PC4.
3. 🖥️ **Open a serial terminal** (115200 baud, 8N1, no flow control) on your PC.
4. ⏱️ **Watch the magic!** Each rising edge on PC4 prints the current microsecond count on a new line.

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
