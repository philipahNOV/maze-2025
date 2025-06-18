import time
import serial
import serial.tools.list_ports

ser = serial.Serial("/dev/ttyACM0", 9600)

start_time = time.time()  # Ta tidsstempel før sending
ser.write(b"test_message")  # Send melding

response = ser.readline()  # Les respons
end_time = time.time()  # Ta tidsstempel ved mottak

total_time = end_time - start_time
print(f"Tidsforsinkelse: {total_time} sekunder")

