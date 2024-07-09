import time
import serial

sonic_ser = serial.Serial('/dev/arduinoUltrasound', 57600, timeout=1)

time.sleep(2)
sonic_ser.flushInput()

while True:
    try:
        if sonic_ser.in_waiting > 0:
            print(sonic_ser.readline().decode("utf-8"))
    except KeyboardInterrupt:
        sonic_ser.close()
        print("closing")
        break
