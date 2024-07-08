import serial

sonic_ser = serial.Serial('/dev/arduinoUltrasound', 57600, timeout=1)

while True:
    print(sonic_ser.readline())

