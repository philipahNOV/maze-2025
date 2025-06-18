import time
import serial
import serial.tools.list_ports

print(f'Åpner seriel port')
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=2)
print(f'Seriel port er åpnet\n')

print(f'Sender melding')
melding = 123456789
meldig_bytes = melding.to_bytes(4, byteorder='big')
start_time = time.time()  # Ta tidsstempel før sending
ser.write(meldig_bytes)  # Send melding

response = ser.read(4)  # Les respons
end_time = time.time()  # Ta tidsstempel ved mottak
print(f'Svar motat\n')

print(f'lukker serielkominikasjonen')
ser.close

total_time = end_time - start_time
print(f"Tidsforsinkelse: {total_time} sekunder")

