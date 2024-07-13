import time
import serial

def test_angle_serial():
    try:
        angle_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)
        #angle_ser.flushInput()
        while True:
            line = angle_ser.readline().decode('utf-8').strip()
            print(f"Angle: {line}")
    except serial.SerialException as e:
        print(f"Error opening or reading from serial port Angle: {e}")
    finally:
        if angle_ser and angle_ser.is_open:
            angle_ser.close()

# Run the test
test_angle_serial()
