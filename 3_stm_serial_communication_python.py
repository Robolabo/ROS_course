import tkinter as tk                # Import the Tkinter library for GUI creation
from tkinter import ttk             # Import ttk for themed widgets in Tkinter
import serial                       # Import the PySerial library for serial communication
import threading                    # Import threading to run tasks in parallel (e.g., sending and reading data)
import time                         # Import time for implementing delays

# Define a class for handling serial communication
class SerialInterface:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # Initialize the serial connection with specified port and baud rate
        # '/dev/ttyACM0' is a typical port for USB serial devices on Linux; change as needed
        # 'baudrate=115200' is a common speed setting for serial communication
        self.ser = serial.Serial(port, baudrate, timeout=1)  # Set a timeout of 1 second
        self.running = True  # Flag to control reading loop
    
    # Method to send data through the serial connection
    def send_data(self, data):
        # Check if the serial port is open before sending data
        if self.ser.is_open:
            # Write data to the serial port, appending a newline character
            # 'encode' converts the string to bytes, as required by PySerial
            self.ser.write((data + '\n').encode('utf-8'))

    # Method to continuously read data from the serial port
    def read_data(self):
        while self.running:  # Run while the flag is True
            # Check if there's any data waiting to be read in the serial buffer
            if self.ser.in_waiting > 0:
                # Read a line of data from the serial port, decode it to a string, and strip any extra whitespace
                data = self.ser.readline().decode('utf-8').strip()
                # Optional: Uncomment the next line to clear the input buffer after reading
                # self.ser.reset_input_buffer()
                if data:  # If data is not empty, print it
                    print(f"Received: {data}")
            # Small sleep to prevent high CPU usage
            time.sleep(0.001)
    
    # Method to stop the reading loop and close the serial connection
    def stop(self):
        self.running = False  # Set flag to False to stop the read loop
        if self.ser.is_open:
            self.ser.close()  # Close the serial port

# Define the main application class for the GUI
class App:
    def __init__(self, root, serial_interface):
        # Initialize the GUI with a title
        self.serial_interface = serial_interface  # Store the SerialInterface object
        self.root = root  # Store the root window
        self.root.title("Serial Communication Interface")  # Set the window title

        # Create input fields for different parameters and position them in a grid layout

        # Control mode label and entry
        self.modo_control_label = tk.Label(root, text="Modo Control:")
        self.modo_control_label.grid(row=0, column=0)  # Position at row 0, column 0
        self.modo_control_entry = tk.Entry(root)  # Create the entry widget
        self.modo_control_entry.grid(row=0, column=1)  # Position at row 0, column 1

        # Setpoint label and entry
        self.setpoint_label = tk.Label(root, text="Setpoint:")
        self.setpoint_label.grid(row=1, column=0)
        self.setpoint_entry = tk.Entry(root)
        self.setpoint_entry.grid(row=1, column=1)

        # KP (proportional gain) label and entry
        self.kp_label = tk.Label(root, text="KP:")
        self.kp_label.grid(row=2, column=0)
        self.kp_entry = tk.Entry(root)
        self.kp_entry.grid(row=2, column=1)

        # PWM (pulse-width modulation) label and entry
        self.pwm_label = tk.Label(root, text="PWM:")
        self.pwm_label.grid(row=3, column=0)
        self.pwm_entry = tk.Entry(root)
        self.pwm_entry.grid(row=3, column=1)

        # Button to start sending data periodically
        self.send_button = ttk.Button(root, text="Start Sending", command=self.start_sending)
        self.send_button.grid(row=4, column=0, columnspan=2)  # Span across two columns

        # Button to stop sending data
        self.stop_button = ttk.Button(root, text="Stop Sending", command=self.stop_sending)
        self.stop_button.grid(row=5, column=0, columnspan=2)  # Span across two columns

        # Flag to control sending loop
        self.sending = False

    # Method to start sending data periodically
    def start_sending(self):
        self.sending = True  # Set flag to True to start sending loop
        # Start a thread to send data periodically
        threading.Thread(target=self.send_data_periodically).start()
        # Start a thread to continuously read data from the serial interface
        threading.Thread(target=self.serial_interface.read_data).start()

    # Method to stop sending data
    def stop_sending(self):
        self.sending = False  # Set flag to False to stop sending loop
        self.serial_interface.stop()  # Stop the serial interface's reading loop

    # Method to send data in a loop with a delay, if sending is active
    def send_data_periodically(self):
        while self.sending:  # Run while sending is active
            # Get the values from the input fields
            modo_control = self.modo_control_entry.get()  # Control mode
            setpoint = self.setpoint_entry.get()          # Setpoint value
            kp = self.kp_entry.get()                      # Proportional gain
            pwm = self.pwm_entry.get()                    # PWM value

            # Format the data as a string to send (in a structured format for easy parsing)
            data = f"{{{modo_control},{setpoint},{kp},{pwm}}}"
            # Send the formatted data string to the serial interface
            self.serial_interface.send_data(data)
            # Delay between messages to prevent flooding the serial port
            time.sleep(0.01)

# Entry point of the program
if __name__ == "__main__":
    # Initialize the serial interface with default parameters
    serial_interface = SerialInterface(port='/dev/ttyACM0', baudrate=115200)
    
    # Create the root window and initialize the GUI app
    root = tk.Tk()
    app = App(root, serial_interface)
    
    # Start the Tkinter event loop (this will keep the GUI open)
    root.mainloop()
    
    # Stop the serial interface when the program exits
    serial_interface.stop()
