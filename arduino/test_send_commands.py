# test_arduino_connection.py
import time
from arduino_connection import ArduinoConnection

def test_send_commands():
    # Initialize the ArduinoConnection
    arduino_connection = ArduinoConnection(baud_rate=115200)
    
    # List of commands to send
    commands = ["move,-300", "move,-200", "move,-100", "move,-300", "move,-100"]
    
    # Give some time for the connection to establish
    time.sleep(2)  # Adjust this based on your setup
    
    for command in commands:
        print(f"Sending command: {command}")
        arduino_connection.send_command(command)
        # Wait a bit between commands to ensure they are processed
        time.sleep(1)
    
    # Stop the Arduino connection after sending all commands
    arduino_connection.stop()
    print("Test completed, connection closed.")

if __name__ == "__main__":
    test_send_commands()