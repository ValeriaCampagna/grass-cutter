import time
import serial

def test_serial(port, baudrate, name):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(3)
        ser.flushInput()
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(f"{name}: {line}")
            time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port {name}: {e}")
    finally:
        ser.close()

# Test each device independently
test_serial('/dev/arduinoUltrasound', 57600, 'Ultrasound')
# test_serial('/dev/arduinoSensors', 115200, 'Angle')
# test_serial('/dev/arduinoMotors', 115200, 'Motor')
