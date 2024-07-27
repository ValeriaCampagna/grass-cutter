import time
import serial

motor_ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# angles_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)

time.sleep(2)

steps = 0
try:
    while True:
        speed = 50
        # for i in range(20):
        #     command = f"{speed},{speed}\n"
        #     motor_ser.write(command.encode())
        #     speed = min(255, speed+20)
        #     if speed == 255:
        #         break
        #     print("current speed:", speed)
        #     time.sleep(0.2)
        motor_ser.write(f"50,0,0,50,0,0,0\n".encode())
        # for i in range(100):
        #     angles_ser.flushInput()
        #     vals = angles_ser.readline().decode("utf-8").strip()
        #     try:
        #         _, lenc, renc = vals.split(",")
        #         print("encoder left", vals)
        #     except Exception:
        #         print("Error reading")
        #         time.sleep(0.1)
        #         continue
        #     if int(lenc) > 660:
        #         print("already moving setting speed to 200")
        #         motor_ser.write("190,190\n".encode())
        #         break
        #     time.sleep(0.05)
        # time.sleep(6)
        # motor_ser.write("0,0\n".encode())
        # time.sleep(4)
except KeyboardInterrupt:
    motor_ser.write("0,0,0,0,0,0,0\n".encode())
