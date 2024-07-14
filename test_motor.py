import time
import serial

motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)
angles_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)

time.sleep(2)

steps = 0
try:
    while True:
        speed = 0
        for i in range(20):
            command = f"{speed},{speed}\n"
            motor_ser.write(command.encode())
            speed = min(185, speed+10)
            print("current speed:", speed)
            time.sleep(0.2)

        for i in range(100):
            vals = angles_ser.readline().decode("utf-8").strip()
            try:
                _, lenc, renc = vals.split(",")
                print("encoder left", vals)
            except Exception:
                print("Error reading")
                time.sleep(0.1)
                continue
            if int(lenc) > 5760:
                print("already moving setting speed to 90")
                speed = 90
                break
            time.sleep(0.05)
        time.sleep(3)
        motor_ser.write("0,0\n".encode())
        time.sleep(4)
except KeyboardInterrupt:
    motor_ser.write("-10,-10\n".encode())
