import serial
import serial.tools.list_ports
import threading
import time
from enum import Enum

class ArduinoState(Enum):
    IDLE = 0
    GET_BALL = 1
    CONTROL = 2
    SET_COLOR = 3

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
        self.command_to_send = None
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
        if not port:
            raise ConnectionError("Arduino not found")
        
        try:
            self.serial_conn = serial.Serial(port, self.baud_rate, timeout=1)
            self.running = True
            print(f"Connected to Arduino on {port}")
            if not self.is_alive():
                self.start()
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to Arduino: {e}")
            

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
    
    def _send_command(self, state: ArduinoState, *args):
        """Internal method to format and queue a command."""
        with self.condition:
            # Lager en kommando-streng fra alle argumentene pluss state
            command_parts = [str(state.value)] + list(map(str, args))
            self.command_to_send = ",".join(command_parts) + "\n"
            self.condition.notify()
    
    def send_idle(self):
        """
        Sends "IDLE" state to the Arduino.
        """
        self._send_command(ArduinoState.IDLE)

    def send_get_ball(self):
        """
        Sends "GET_BALL" state to the Arduino.
        """
        self._send_command(ArduinoState.GET_BALL)

    def send_speed(self, speed1: int, speed2: int):
        """
        Sends target speeds and "CONTROL" state to the Arduino.

        Args:
            speed1 (int): The speed for motor 1.
            speed2 (int): The speed for motor 2.
        """
        self._send_command(ArduinoState.CONTROL, speed1, speed2)

    def send_color(self, r: int, g: int, b: int):
        """
        Sends target rgb value and "SET_COLOR" state to the Arduino.

        Args:
            r (int): The red component of the color.
            g (int): The green component of the color.
            b (int): The blue component of the color.
        """
        self._send_command(ArduinoState.SET_COLOR, r, g, b)

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
                if self.command_to_send is None:
                    self.condition.wait()
                
                if not self.running:
                    break

                if self.command_to_send:
                    if self.serial_conn and self.serial_conn.is_open:
                        try:
                            self.serial_conn.write(self.command_to_send.encode())
                        except serial.SerialException as e:
                            print(f"Failed to send command: {e}. Attempting to reconnect...")
                            self.reconnect()
                    else:
                        print("No connection to Arduino. Attempting to reconnect...")
                        self.reconnect()
                    self.command_to_send = None


    def stop(self):
        if not self.running:
            return
        self.running = False
        with self.condition:
            self.condition.notify()
        if self.is_alive():
            self.join()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Connection to Arduino closed")
    
    def reconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        print("Attempting to reconnect to Arduino...")
        time.sleep(2)
        try:
            self.connect()
        except ConnectionError as e:
            print(e)


