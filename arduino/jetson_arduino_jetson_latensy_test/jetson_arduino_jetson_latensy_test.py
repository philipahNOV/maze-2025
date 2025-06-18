import time
import serial
import serial.tools.list_ports

print(f'Åpner seriel port')
ser = serial.Serial("/dev/ttyACM0", 9600)
print(f'Seriel port er åpnet\n')

print(f'Sender melding')
start_time = time.time()  # Ta tidsstempel før sending
ser.write(b"test_message\n")  # Send melding

print(f'Venter på svar')
response = ser.readline()  # Les respons
end_time = time.time()  # Ta tidsstempel ved mottak
print(f'Svar motat\n')

total_time = end_time - start_time
print(f"Tidsforsinkelse: {total_time} sekunder")

