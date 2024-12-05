import serial
import time
import os

# Configure the serial port
uart_port = 'COM4'  # For Linux, replace with your UART port
# uart_port = 'COM3'  # For Windows, replace with your UART port
baud_rate = 115200

# Initialize serial connection
ser = serial.Serial(uart_port, baud_rate, timeout=1)

# Define the instruction message and line sizes
INSTRUCTION_MSG = "Hold blue button for 1 second to toggle tuner modes (Ear or Mic) and tap it to switch strings."
MODE_MSG = "Current mode: Microphone Tuning"  # Default mode
STRING_MSG = "Current string: E2"  # Default string
TUNING_MSG = ""  # Default tuning message

def move_up_and_erase_line(lines=1):
    """Moves the cursor up and erases the previous line(s)."""
    print("\033[A" + "\033[K" * lines, end="")  # Move up 'lines' and erase them

def clear_terminal():
    """Clears the entire terminal."""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_instruction():
    """Prints the instruction message at the top."""
    print(INSTRUCTION_MSG)

def print_mode():
    """Prints the current mode."""
    print(MODE_MSG)

def print_string():
    """Prints the current string."""
    print(STRING_MSG)

def print_tuning():
    """Prints the current tuning message."""
    print(TUNING_MSG)

def read_from_uart():
    """Reads data from UART and displays it."""
    global MODE_MSG, STRING_MSG, TUNING_MSG
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()  # Read line and decode
            if data:
                # If the message is "CLEAR_SCREEN", you can clear the entire screen or erase the last line
                if data == "CLEAR_SCREEN":
                    clear_terminal()  # Clear the entire screen if you want
                else:
                    # Move up previous lines and update the status messages
                    print("\033[A" + "\033[K" , end="")
                    print("\033[A" + "\033[K" , end="")
                    print("\033[A" + "\033[K" , end="")
                    print("\033[A" + "\033[K" , end="")
                    
                    print_instruction()        # Print the static instruction
                    if "Ear Tuning" in MODE_MSG:  # This will check if "Ear Tuning" is anywhere in the string
                        TUNING_MSG = ""  # Clear tuning message if in ear tuning mode
                    if "mode" in data.lower():
                        MODE_MSG = data  # Update mode message
                    elif "string" in data.lower():
                        STRING_MSG = data  # Update string message
                    elif "tuning" in data.lower():
                        if "in tune" in data.lower():
                            TUNING_MSG = "\033[1;32m" + data + "\033[0m"  #make it green
                        elif ">" in data.lower():
                            TUNING_MSG = data + " — string is flat, tune up"  # Update tuning message
                        elif "<" in data.lower():
                            TUNING_MSG = data + " — string is sharp, tune down"
                    print_mode()               # Print current mode
                    print_string()             # Print current string
                    print_tuning()             # Print tuning message

def send_to_uart(message):
    """Sends a message to the UART."""
    ser.write(message.encode('utf-8'))
    ser.flush()  # Ensure the message is sent immediately

def main():
    print("UART Terminal started...")
    time.sleep(1)  # Allow some time for the device to initialize
    
    print("Hold blue button to start the tuner in Ear tuning mode.")
    print("\n")
    # Start reading UART messages
    read_from_uart()

if __name__ == '__main__':
    main()
