import serial
import threading
import sys
import time

# --- Configuration ---
PORT = 'COM10'
BAUDRATE = 115200
TIMEOUT = 3

# --- Global variable to control the threads ---
running = True

# --- Serial port setup ---
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)  # Give the connection time to establish
    print(f"Connected to {PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"Error: Could not open port {PORT}. {e}")
    sys.exit()


# --- Functions for reading and writing ---
def read_from_port():
    """Continuously reads data from the serial port and prints it."""
    global running
    while running:
        try:
            if ser.in_waiting > 0:
                # Read all available bytes
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"Received from board: {data}")
        except serial.SerialException as e:
            print(f"Reading error: {e}")
            running = False
            break
        time.sleep(0.1)  # Prevents the loop from consuming too much CPU


def write_to_port():
    """Prompts for user input and sends it to the serial port."""
    global running
    print("\nType a character to send to the board. Type 'exit' to quit.")
    while running:
        try:
            # Get input from the user
            user_input = sys.stdin.readline().strip()

            if user_input.lower() == 'exit':
                running = False
                break

            if user_input:
                # Encode the string to bytes and send it
                ser.write(user_input.encode('utf-8'))
                print(f"Sent: {user_input}")
        except serial.SerialException as e:
            print(f"Writing error: {e}")
            running = False
            break
        except Exception as e:
            print(f"Input error: {e}")
            running = False
            break


# --- Main script ---
if __name__ == "__main__":
    try:
        # Create and start the threads
        reader_thread = threading.Thread(target=read_from_port, daemon=True)
        writer_thread = threading.Thread(target=write_to_port, daemon=True)

        reader_thread.start()
        writer_thread.start()

        # Keep the main thread alive until the user decides to exit
        while running:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # Clean up
        running = False
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")