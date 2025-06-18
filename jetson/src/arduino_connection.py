import serial
import serial.tools.list_ports
import threading

import time
class ArduinoConnection(threading.Thread):
    def __init__(self, baud_rate=9600):
        """
        Initializes an instance of the ArduinoConnection class.

        Args:
            baud_rate (int, optional): The baud rate for the serial connection. Defaults to 9600.
        """
        threading.Thread.__init__(self)  # Initialize the base Thread class first
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.command_var = None
        self.running = False
        self.condition = threading.Condition()
        self.connect()  # Attempt to connect to the Arduino

    def connect(self):
        """
        Connects to the Arduino board.

        This method detects the Arduino port, establishes a serial connection,
        and starts a thread for communication with the Arduino.

        Raises:
            ConnectionError: If the Arduino is not found or if there is a failure
                in establishing the serial connection.

        """
        port = self.detect_arduino_port()
        if port:
            try:
                self.serial_conn = serial.Serial(port, self.baud_rate)
                self.running = True
                print(f"Connected to Arduino on {port}")
                self.start()  # Start the thread after a successful connection
            except serial.SerialException as e:
                raise ConnectionError(f"Failed to connect to Arduino: {e}")
        else:
            raise ConnectionError("Arduino not found")
            

    def detect_arduino_port(self):
        """
        Detects the port of the Arduino device connected to the system.

        Returns:
            str: The device port of the Arduino, or None if no Arduino device is found.
        """
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'Arduino' in port.description or 'ttyUSB' in port.device or 'ttyACM' in port.device:
                return port.device
        return None
    
    def send_target_positions(self, position1, position2, speed1, speed2):
        """
        Sends target positions and speeds to the Arduino.

        Args:
            position1 (int): The target position for motor 1.
            position2 (int): The target position for motor 2.
            speed1 (int): The speed for motor 1.
            speed2 (int): The speed for motor 2.
        """
        print(f"Sending target positions: {position1}, {position2}, {speed1}, {speed2}")
        with self.condition:
            self.command_var = f"{position2} {position1} {speed1} {speed2}\n"
            self.condition.notify()
        print(f"Command set: {self.command_var.strip()}")


    def run(self):
        """
        Runs the ArduinoConnection thread.

        This method is responsible for sending commands to the Arduino when the `command_var` is set.
        It waits for the `condition` to be notified, and then checks if there is a valid serial connection.
        If there is a connection, it sends the command to the Arduino. If not, it attempts to reconnect.

        Note:
            This method should be called after starting the thread.

        Raises:
            serial.SerialException: If there is an error while sending the command to the Arduino.

        """
        print("ArduinoConnection thread started")
        while self.running:
            with self.condition:
                self.condition.wait()
                if self.command_var:
                    if self.serial_conn and self.serial_conn.is_open:
                        try:
                            self.serial_conn.write(self.command_var.encode())
                            # print(f"Sent: {self.command_var}")
                        except serial.SerialException as e:
                            print(f"Failed to send target positions: {e}")
                            self.reconnect()
                    else:
                        print("No connection to Arduino. Cannot send target positions.")
                        self.reconnect()
                    self.command_var = None


    def stop(self):
        self.running = False
        with self.condition:
            self.condition.notify()
        self.join() 
        if self.serial_conn:
            self.serial_conn.close()
            print("Connection to Arduino closed")
    
    def reconnect(self):
        print("Attempting to reconnect to Arduino...")
        self.stop()
        time.sleep(2)
        self.connect()


