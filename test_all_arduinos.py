import time
import serial


def test_serial(port, baudrate, name):
    ser = serial.Serial(port, baudrate, timeout=1)
    if name == "Ultrasound":
        time.sleep(3)
        ser.flushInput()
    else:
        time.sleep(2)  # Give some time for the sensors to initialize
        ser.flushInput()
    return ser


# Test each device independently
ultra = test_serial('/dev/arduinoUltrasound', 57600, 'Ultrasound')
sensors = test_serial('/dev/arduinoSensors', 115200, 'Angle')
sensors.flushInput()
# test_serial('/dev/arduinoMotors', 115200, 'Motor')

for _ in range(10):
    sensors.readline()
    ultra.readline()
    # Add here for motor if needed
    # motor.readline()

while True:
    # Synchronize reads
    ultra.flushInput()
    sensors.flushInput()

    # Read ultrasound data
    ultra_data = ultra.readline().decode('utf-8').strip()
    if ultra_data:
        print(f"ultrasound: {ultra_data}")

    # Read angle data
    angle_data = sensors.readline().decode('utf-8').strip()
    if angle_data:
        print(f"angles: {angle_data}")

    # Add a small delay to prevent overwhelming the serial ports
    time.sleep(0.1)

