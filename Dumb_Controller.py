import serial
import time

# Define serial ports and baud rates for the Arduinos
MOTOR_SERIAL_PORT = '/dev/arduinoMotors'
SENSOR_SERIAL_PORT = '/dev/arduinoSensors'
BAUD_RATE = 115200

# Initialize serial connections
motor_ser = serial.Serial(MOTOR_SERIAL_PORT, BAUD_RATE, timeout=1)
sensor_ser = serial.Serial(SENSOR_SERIAL_PORT, BAUD_RATE, timeout=1)

time.sleep(2)  # Allow time for the serial connections to initialize


def read_sensor_data():
    """Read data from the sensor Arduino and return it as a tuple (yaw, left_encoder, right_encoder)."""
    sensor_ser.flushInput()
    line = sensor_ser.readline().decode('utf-8').strip()
    if line:
        try:
            yaw, left_encoder, right_encoder = map(float, line.split(','))
            return yaw, left_encoder, right_encoder
        except ValueError:
            print("Error parsing sensor data:", line)
            return None
    return None


def send_motor_command(left_pwm, right_pwm):
    """Send motor command to the motor Arduino."""
    command = f"{left_pwm},{right_pwm}\n"
    motor_ser.write(command.encode())


def main():
    try:
        while True:
            sensor_data = read_sensor_data()
            if sensor_data:
                yaw, left_encoder, right_encoder = sensor_data
                print(f"Yaw: {yaw:.2f}, Left Encoder: {left_encoder}, Right Encoder: {right_encoder}")

            # Here you can add your own logic to determine motor commands
            # For this example, we'll just read user input to control the motors
            left_pwm = input("Enter left motor PWM: ")
            right_pwm = input("Enter right motor PWM: ")
            try:
                left_pwm = int(left_pwm)
                right_pwm = int(right_pwm)
                send_motor_command(left_pwm, right_pwm)
            except ValueError:
                print("Invalid PWM values. Please enter integers.")

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        motor_ser.close()
        sensor_ser.close()


if __name__ == "__main__":
    main()
