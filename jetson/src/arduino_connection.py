import serial
import serial.tools.list_ports
import threading
import time
from enum import Enum

class ArduinoState(Enum):
    IDLE = 0
    ELEVATOR = 1
    CONTROL = 2
    SET_COLOR = 3

class ArduinoConnection(threading.Thread):
    def __init__(self, baud_rate=115200):
        threading.Thread.__init__(self)
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.command_to_send = None
        self.running = False
        self.condition = threading.Condition()
        self.connect()

    def connect(self):
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
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'Arduino' in port.description or 'ttyUSB' in port.device or 'ttyACM' in port.device:
                return port.device
        return None
    
    def _send_command(self, state: ArduinoState, *args):
        with self.condition:
            # Lager en kommando-streng fra alle argumentene pluss state
            command_parts = list(map(str, args)) + [str(state.value)]
            self.command_to_send = ",".join(command_parts) + "\n"
            self.condition.notify()
    
    def send_idle(self):
        self._send_command(ArduinoState.IDLE)

    def send_elevator(self, direction):
        self._send_command(ArduinoState.ELEVATOR, int(direction))

    def send_speed(self, speed1: int, speed2: int):
        if not hasattr(self, '_last_speed1'):
            self._last_speed1 = None
            self._last_speed2 = None
        
        if (self._last_speed1 != speed1 or self._last_speed2 != speed2):
            self._send_command(ArduinoState.CONTROL, int(speed1), int(speed2))
            self._last_speed1 = speed1
            self._last_speed2 = speed2

    def send_color(self, r: int, g: int, b: int, index: int = -1):
        if not hasattr(self, '_last_color_r'):
            self._last_color_r = None
            self._last_color_g = None
            self._last_color_b = None
            self._last_color_index = None
        
        if (self._last_color_r != r or self._last_color_g != g or 
            self._last_color_b != b or self._last_color_index != index):
            self._send_command(ArduinoState.SET_COLOR, int(r), int(g), int(b), int(index))
            self._last_color_r = r
            self._last_color_g = g
            self._last_color_b = b
            self._last_color_index = index

    def run(self):
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
    
    def reconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        time.sleep(2)
        try:
            self.connect()
        except ConnectionError as e:
            print(e)