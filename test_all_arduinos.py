import time
import serial


def initialize_serial_ports():
    try:
        sonic_ser = serial.Serial('/dev/arduinoUltrasound', 57600, timeout=1)
        time.sleep(3)
        sonic_ser.flushInput()

        angle_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)
        motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)

        return sonic_ser, angle_ser, motor_ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None, None, None


def read_serial(ser, name):
    try:
        if ser and ser.is_open:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(f"{name}: {line}")
        else:
            print(f"Serial port {name} not open.")
    except serial.SerialException as e:
        print(f"Error reading from serial port {name}: {e}")


def main():
    sonic_ser, angle_ser, motor_ser = initialize_serial_ports()
    if not sonic_ser or not angle_ser or not motor_ser:
        return

    while True:
        read_serial(sonic_ser, "Ultrasound")
        read_serial(angle_ser, "Angle")
        read_serial(motor_ser, "Motor")

        time.sleep(0.1)  # Adjust the sleep time as necessary


if __name__ == "__main__":
    main()
