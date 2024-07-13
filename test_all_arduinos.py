import time
import serial


def test_serial(port, baudrate, name):
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(3)
    ser.flushInput()
    return ser


# Test each device independently
ultra = test_serial('/dev/arduinoUltrasound', 57600, 'Ultrasound')
sensors = test_serial('/dev/arduinoSensors', 115200, 'Angle')
# test_serial('/dev/arduinoMotors', 115200, 'Motor')

while True:
    line = ultra.readline().decode('utf-8').strip()
    print(f"ultrasound: {line}")
    gline = sensors.readline().decode('utf-8').strip()
    print(f"angles: {gline}")
    time.sleep(0.1)
