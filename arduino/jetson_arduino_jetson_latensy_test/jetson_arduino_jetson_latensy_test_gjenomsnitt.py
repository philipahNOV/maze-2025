import serial
import struct
import time

def main():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    
    tall1 = 123456789
    tall2 = 987654321
    tall3 = 192837465
    tall4 = 567890123

    data = struct.pack('>IIII', tall1, tall2, tall3, tall4)

    n = 1000
    totale_tider = []

    print(f'Starter testen')

    for _ in range(n):
        print(f'Melding {n}')
        start_time = time.time()
        
        ser.write(data)
        
        mottatt = ser.read(16)
        
        end_time = time.time()
        
        rundtur_tid = end_time - start_time
        totale_tider.append(rundtur_tid)

    gjennomsnitt_tid = sum(totale_tider) / n
    
    print(f"Gjennomsnittlig tid for rundreisetid over {n} iterasjoner: {gjennomsnitt_tid:.6f} sekunder")

    ser.close()

if __name__ == "__main__":
    main()
