import serial
import time
import math

# Define serial ports and baud rates for the Arduinos
MOTOR_SERIAL_PORT = '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0'
SENSOR_SERIAL_PORT = '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0'
BAUD_RATE = 115200

# Initialize serial connections
motor_ser = serial.Serial(MOTOR_SERIAL_PORT, BAUD_RATE, timeout=1)
sensor_ser = serial.Serial(SENSOR_SERIAL_PORT, BAUD_RATE, timeout=1)

time.sleep(2)  # Allow time for the serial connections to initialize

# Wheel and encoder parameters
WHEEL_DIAMETER_CM = 13
COUNTS_PER_REVOLUTION = 5760
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_CM
DISTANCE_PER_TICK = CIRCUMFERENCE / COUNTS_PER_REVOLUTION

# Control parameters
KP = 1.0  # Proportional gain for yaw correction
BRAKE_DISTANCE = 10.0  # Distance in cm where braking starts
MAX_SPEED = 140  # Maximum speed to cap the robot's speed
INITIAL_BOOST = 255  # Initial boost speed to overcome inertia
UTURN_SPEED = 240  # Increased speed for U-turn
NEGATIVE_PWM = -50  # Negative PWM to lock stationary wheel
MIN_YAW_CHANGE = 5  # Minimum yaw change threshold to consider U-turn complete


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


def calculate_ticks_for_distance(distance_cm):
    """Calculate the number of encoder ticks required to travel the given distance."""
    return distance_cm / DISTANCE_PER_TICK


def perform_u_turn():
    """Perform a U-turn with the axis of rotation on the right wheel."""
    print("Performing U-turn...")

    initial_yaw = None
    target_yaw_change = 180  # Target change in yaw for U-turn

    while True:
        sensor_data = read_sensor_data()
        if sensor_data:
            yaw, left_encoder, right_encoder = sensor_data

            if initial_yaw is None:
                initial_yaw = yaw

            yaw_change = yaw - initial_yaw
            if yaw_change < 0:
                yaw_change += 360  # Adjust for wrap-around

            print(f"Yaw change: {yaw_change:.2f} degrees")

            if yaw_change >= target_yaw_change and yaw_change >= MIN_YAW_CHANGE:
                send_motor_command(0, 0)
                print("U-turn completed.")
                break

            # Apply braking logic
            remaining_yaw_change = target_yaw_change - yaw_change
            if remaining_yaw_change <= 45:  # Start braking 45 degrees before completion
                speed_factor = remaining_yaw_change / 45
                left_speed = UTURN_SPEED * speed_factor
                left_speed = max(left_speed, 50)  # Ensure minimum speed for control
            else:
                left_speed = UTURN_SPEED

            # Rotate left wheel forward while right wheel stays stationary with slight negative PWM
            send_motor_command(int(left_speed), NEGATIVE_PWM)

        time.sleep(0.1)


def move_straight_to_start(initial_left_encoder, initial_right_encoder, initial_yaw):
    """Move straight until reaching the starting point."""
    print("Moving straight to starting point...")

    has_started = False
    initial_distance = calculate_distance(initial_left_encoder, initial_right_encoder)

    while True:
        sensor_data = read_sensor_data()
        if sensor_data:
            yaw, left_encoder, right_encoder = sensor_data

            # Calculate distance traveled from the initial encoders
            current_distance = calculate_distance(left_encoder, right_encoder) - initial_distance
            print(f"Distance traveled back: {current_distance:.2f} cm")

            if current_distance <= 0:
                send_motor_command(0, 0)
                print("Reached starting point.")
                break

            # Yaw correction
            yaw_error = yaw - initial_yaw
            correction = KP * yaw_error

            # Determine motor speeds
            if not has_started or current_distance < 20:
                base_speed = INITIAL_BOOST
                has_started = True
            else:
                base_speed = MAX_SPEED

            left_speed = base_speed + correction
            right_speed = base_speed - correction

            # Cap the speeds to max speed
            left_speed = min(left_speed, MAX_SPEED)
            right_speed = min(right_speed, MAX_SPEED)

            # Send motor command
            send_motor_command(int(left_speed), int(right_speed))

        time.sleep(0.1)


def calculate_distance(left_encoder, right_encoder):
    """Calculate the average distance traveled based on encoder readings."""
    average_ticks = (left_encoder + right_encoder) / 2
    return average_ticks * DISTANCE_PER_TICK


def main():
    try:
        # Input the desired distance to travel
        distance_cm = float(input("Enter distance to travel in cm: "))
        adjusted_distance_cm = max(0, distance_cm - 20)  # Subtract 20 cm from the target distance
        target_ticks = calculate_ticks_for_distance(adjusted_distance_cm)
        print(f"Adjusted distance to travel: {adjusted_distance_cm} cm")
        print(f"Target encoder ticks: {target_ticks}")

        # Initialize variables
        initial_left_encoder = initial_right_encoder = None
        initial_yaw = None
        has_started = False
        initial_distance = 0

        while True:
            sensor_data = read_sensor_data()
            if sensor_data:
                yaw, left_encoder, right_encoder = sensor_data

                if initial_left_encoder is None:
                    initial_left_encoder = left_encoder
                    initial_right_encoder = right_encoder
                    initial_yaw = yaw
                    initial_distance = calculate_distance(left_encoder, right_encoder)

                # Calculate distance traveled
                current_distance = calculate_distance(left_encoder, right_encoder) - initial_distance
                print(f"Distance traveled: {current_distance:.2f} cm")

                # Check if target distance is reached or slightly overshot
                if current_distance >= adjusted_distance_cm:
                    print("Target distance reached.")
                    send_motor_command(0, 0)
                    break

                # Yaw correction
                yaw_error = yaw - initial_yaw
                correction = KP * yaw_error

                # Determine motor speeds
                if not has_started or current_distance < 20:
                    base_speed = INITIAL_BOOST
                    has_started = True
                else:
                    base_speed = MAX_SPEED
                    remaining_distance = adjusted_distance_cm - current_distance

                    # Apply braking logic
                    if remaining_distance <= BRAKE_DISTANCE:
                        speed_factor = remaining_distance / BRAKE_DISTANCE
                        base_speed *= speed_factor
                        base_speed = max(base_speed, 50)  # Ensure minimum speed for control

                left_speed = base_speed + correction
                right_speed = base_speed - correction

                # Cap the speeds to max speed
                left_speed = min(left_speed, MAX_SPEED)
                right_speed = min(right_speed, MAX_SPEED)

                # Send motor command
                send_motor_command(int(left_speed), int(right_speed))

            time.sleep(0.1)  # Read data every 100ms

        # Perform U-turn after reaching the target distance
        perform_u_turn()

        # Ensure the robot starts moving again with a boost after U-turn
        move_straight_to_start(initial_left_encoder, initial_right_encoder, initial_yaw)

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        send_motor_command(0, 0)  # Stop the motors
        motor_ser.close()
        sensor_ser.close()


if __name__ == "__main__":
    main()
