import time
import serial

motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)

while True:
    speed = 90
    for i in range(20):
        command = f"{speed},{speed}\n"
        motor_ser.write(command.encode())
        speed = min(150, speed+4)
        time.sleep(0.1)
    time.sleep(4)
    motor_ser.write("0,0\n".encode())
    time.sleep(4)
