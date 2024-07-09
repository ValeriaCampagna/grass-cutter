import time
import serial
import threading

def initialize_serial(port, baudrate, name):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(3)  # Allow time for the device to initialize
        ser.flushInput()
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {name}: {e}")
        return None

def read_serial(ser, name):
    while True:
        try:
            if ser and ser.is_open:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    print(f"{name}: {line}")
            else:
                print(f"Serial port {name} not open.")
        except serial.SerialException as e:
            print(f"Error reading from serial port {name}: {e}")
        time.sleep(0.1)  # Adjust the sleep time as necessary

def main():
    sonic_ser = initialize_serial('/dev/arduinoUltrasound', 57600, 'Ultrasound')
    angle_ser = initialize_serial('/dev/arduinoSensors', 115200, 'Angle')
    motor_ser = initialize_serial('/dev/arduinoMotors', 115200, 'Motor')

    if not sonic_ser or not angle_ser or not motor_ser:
        return

    # Create and start threads for each serial device
    threading.Thread(target=read_serial, args=(sonic_ser, 'Ultrasound')).start()
    threading.Thread(target=read_serial, args=(angle_ser, 'Angle')).start()
    threading.Thread(target=read_serial, args=(motor_ser, 'Motor')).start()

if __name__ == "__main__":
    main()
