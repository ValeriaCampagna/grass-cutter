import serial
import time

motor_ser = serial.Serial("/dev/arduinoMotors", 115200, timeout=1)
motor_ser.write("150,150\n".encode())
try:
    time.sleep(2)
except KeyboardInterrupt:
    motor_ser.write("0,0\n".encode())

while True:
    try:
        motor_ser.write("-100,255\n".encode())
    except KeyboardInterrupt:
        motor_ser.write("0,0\n".encode())
        break
