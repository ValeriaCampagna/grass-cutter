import time
import serial

sonic_ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

time.sleep(2)
while True:
    print(sonic_ser.readline().decode("utf-8").strip())
