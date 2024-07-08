import time
import serial

sonic_ser = serial.Serial('/dev/arduinoUltrasound', 57600, timeout=1)

time.sleep(2)
while True:
    print(sonic_ser.readline().decode("utf-8").strip())
