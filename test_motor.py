import time
import serial

motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)

try:
    while True:
        speed = 0
        for i in range(20):
            command = f"{speed},{speed}\n"
            motor_ser.write(command.encode())
            speed = min(155, speed+8)
            print("current speed:", speed)
            time.sleep(0.2)
        time.sleep(4)
        motor_ser.write("0,0\n".encode())
        time.sleep(4)
except KeyboardInterrupt:
    motor_ser.write("-10,-10\n".encode())
