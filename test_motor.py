import time
import serial

motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)
angles_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)

steps = 0
try:
    while True:
        speed = 0
        for i in range(20):
            vals = angles_ser.readline().decode("utf-8").strip()
            try:
                _, lenc, renc = vals.split(",")
            except Exception:
                print("Error reading")
                time.sleep(0.1)
            if lenc > 5760:
                print("already moving")
                speed = 90
                break
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
