import time
import serial

def test_angle_serial():
    try:
        angle_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)
        time.sleep(3)
        angle_ser.flushInput()
        while True:
            if angle_ser.in_waiting > 0:
                line = angle_ser.readline().decode('utf-8').strip()
                print(f"Angle: {line}")
            else:
                print("No data available.")
            time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Error opening or reading from serial port Angle: {e}")
    finally:
        if angle_ser and angle_ser.is_open:
            angle_ser.close()

# Run the test
test_angle_serial()
