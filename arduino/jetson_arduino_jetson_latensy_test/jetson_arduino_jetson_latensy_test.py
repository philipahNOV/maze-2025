import time
import serial
import serial.tools.list_ports

def detect_arduino_port():
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

ser = serial.Serial(detect_arduino_port(), 9600)

start_time = time.time()  # Ta tidsstempel før sending
ser.write(b"test_message")  # Send melding

response = ser.readline()  # Les respons
end_time = time.time()  # Ta tidsstempel ved mottak

total_time = end_time - start_time
print(f"Tidsforsinkelse: {total_time} sekunder")
