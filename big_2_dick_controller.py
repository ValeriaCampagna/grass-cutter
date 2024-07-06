import os
import time
import logging
import serial
import pygame
from pygame.locals import JOYBUTTONDOWN
from simple_pid import PID

logging.basicConfig(filename="logs.txt", filemode="w", level=logging.INFO)


# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    exit()

# Initialize the joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick Name: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")

width = None
height = None
dimensions_file = "saved_work_area_dimensions.txt"
if os.path.isfile(dimensions_file):
    with open(dimensions_file, "r") as f:
        wh = f.readline()
        width, height = (float(i) for i in wh.split(","))
        f.close()


class RobotController:
    def __init__(self):
        # Initialize serial ports (update ports and baud rates as needed)
        self.TURNING_SPEED = 55
        self.LEFT_CRUISE_SPEED = 64
        self.RIGHT_CRUISE_SPEED = 55
        # In Centimeters
        self.WHEEL_RADIUS = 35

        self.motor_ser = serial.Serial('COM3', 115200, timeout=1)
        self.angle_ser = serial.Serial('COM4', 115200, timeout=1)

        self.current_state = init_state
        self.state_history = []
        self.sensor_data: dict = dict()
        self.target_angle = 0
        self.angle_delta = 0
        self.angle_error_margin = 1

        self.distance_per_tick = 0.021
        self.turn_right_next = True

        self.total_ticks = 0
        # TODO: updating this needs to be updated
        self.number_of_turns = -1

        # In centimeters. Just for testing
        self.workspace_height = height if height is not None else 0
        self.workspace_width = width if width is not None else 0 #4 * self.WHEEL_RADIUS

        self.homing_turns = 0
        self.homing = False
        self.mapping = False

        # Desired speed in ticks per second
        self.desired_speed_cm_per_s = 10
        desired_ticks_per_hundred_ms = (self.desired_speed_cm_per_s / self.distance_per_tick) // 10

        # PID controllers
        self.left_pid = PID(1.7, 0, 0, setpoint=desired_ticks_per_hundred_ms)
        self.right_pid = PID(1.7, 0, 0, setpoint=desired_ticks_per_hundred_ms)
        self.left_pid.output_limits = (0, 255)  # Limits to motor power
        self.right_pid.output_limits = (0, 255)  # Limits to motor power

        self.r_ticks_in_current_interval = 0
        self.r_total_ticks_up_to_current_interval = 0

        self.l_ticks_in_current_interval = 0
        self.l_total_ticks_up_to_current_interval = 0

        # Time tracking
        self.last_update_time = time.time()

    def update(self):
        self.update_sensor_readings()
        current_time = time.time()
        if (current_time - self.last_update_time) >= 0.1 and self.sensor_data != {}:  # Update every 100 ms
            l_encoder = self.sensor_data["left_encoder"]
            r_encoder = self.sensor_data["right_encoder"]

            self.l_ticks_in_current_interval = l_encoder - self.l_total_ticks_up_to_current_interval
            self.l_total_ticks_up_to_current_interval = l_encoder

            self.r_ticks_in_current_interval = r_encoder - self.r_total_ticks_up_to_current_interval
            self.r_total_ticks_up_to_current_interval = r_encoder
            logging.info(f"Left ticks {self.l_ticks_in_current_interval} | Right ticks {self.r_ticks_in_current_interval} | Expected {self.left_pid.setpoint}")
            self.last_update_time = current_time
            self.update_pid()

        self.current_state(self)

    def change_state(self, new_state):
        state_name = new_state.__name__
        print("NEW STATE: ", state_name)
        logging.info(f"NEW STATE: {state_name}")
        self.state_history.append(state_name)
        self.current_state = new_state

    def get_angle_deviation(self):
        return abs(abs(self.sensor_data["angle"]) - abs(self.target_angle))

    def update_pid(self):
        # PID control for motor speeds
        self.LEFT_CRUISE_SPEED = l = int(self.left_pid(self.l_ticks_in_current_interval))
        self.RIGHT_CRUISE_SPEED = r = int(self.right_pid(self.r_ticks_in_current_interval))
        logging.info(f"Left Updated Speed: {l} | Right Updated Speed: {r}")

    def forward(self):
        deviation = self.get_angle_deviation()
        logging.info(
            f"Target Angle: {self.target_angle} | Real Angle: {self.sensor_data['angle']} | Deviation: {deviation}")
        self.send_speed(self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
        return deviation

    def imu_forward(self):
        deviation = self.get_angle_deviation()
        logging.info(
            f"Target Angle: {self.target_angle} | Real Angle: {self.sensor_data["angle"]} | Deviation: {deviation}")
        if deviation > self.angle_error_margin:
            # IF angle is positive stop right wheel and increase left wheel speed
            if self.sensor_data["angle"] > self.target_angle:
                # print("Turning Right")
                self.send_speed(self.TURNING_SPEED, 0)
            elif self.sensor_data["angle"] < self.target_angle:
                # print("Turning Left")
                self.send_speed(0, self.TURNING_SPEED)
            return False
        else:
            self.send_speed(self.TURNING_SPEED, self.TURNING_SPEED)
            return True

    def u_turn(self):
        deviation = self.get_angle_deviation()
        if deviation > self.angle_error_margin:
            # IF angle is positive stop right wheel and increase left wheel speed
            if self.sensor_data["angle"] > self.target_angle:
                if self.right_pid.auto_mode:
                    self.right_pid.auto_mode = False
                # print("Turning Right")
                self.send_speed(self.LEFT_CRUISE_SPEED, 0)
            elif self.sensor_data["angle"] < self.target_angle:
                if self.left_pid.auto_mode:
                    self.left_pid.auto_mode = False
                # print("Turning Left")
                self.send_speed(0, self.RIGHT_CRUISE_SPEED)
        return deviation

    def send_speed(self, left_speed, right_speed):
        command = f"{right_speed},{left_speed}\n"
        self.motor_ser.write(command.encode())

    def update_sensor_readings(self):
        data = self.angle_ser.readline().strip().decode('utf-8')
        data_list = data.split(",")
        if len(data_list) != 3:
            print("Error parsing: ", data)
            return {}

        # The encoders are backwards
        angle, right_encoder, left_encoder = data.split(",")

        self.sensor_data = {
            "angle": round(float(angle) - self.angle_delta),
            "left_encoder": float(left_encoder) - self.total_ticks,
            "left_encoder_raw": float(left_encoder),
            "right_encoder": float(right_encoder)
        }

    def reset_encoders(self):
        # Since resetting encoders in the arduino code does not actually reset the encoder
        # values sent due to some communication issue we have to keep track of ticks taken
        # during turning manually like cavemen
        self.total_ticks = self.sensor_data["left_encoder_raw"]
        self.send_speed(0, 0)

        self.l_total_ticks_up_to_current_interval = 0
        self.r_total_ticks_up_to_current_interval = 0
        # Stop for half a second to let the motors slow down
        # time.sleep(0.5)

    def reset_angle(self):
        self.angle_delta = self.sensor_data["angle"]

    def get_tracked_distance(self):
        covered_distance = self.distance_per_tick * self.sensor_data["left_encoder"]
        return covered_distance

    def halt(self):
        print("Exiting Program")
        self.send_speed(0, 0)
        self.motor_ser.close()  # Close motor serial port
        self.angle_ser.close()  # Close angle serial port


def init_state(controller: RobotController):
    if controller.sensor_data == {}:
        return

    controller.send_speed(0, 0)
    print("Adding delay")
    time.sleep(1)
    controller.change_state(map_state)


def map_state(controller: RobotController):
    if controller.mapping:
        controller.forward()

    for event in pygame.event.get():
        if event.type != JOYBUTTONDOWN:
            continue
        button = event.button
        # 0 == X button; pressing this means we start mapping
        if button == 0:
            controller.mapping = True

        # IMPORTANT:
        # 3 == Y button; pressing this means use stored mapping
        if button == 3:
            if not (controller.workspace_width == controller.workspace_height == 0):
                controller.change_state(cruise_state)

        # Once we start turning it MUST mean we reached a corner
        if button == 14:
            # 13 == right d-pad, 14 == left d-pad
            controller.workspace_height = controller.get_tracked_distance()
            controller.reset_encoders()
            controller.target_angle = -90
            controller.change_state(turn_state)

        # Finish the mapping and save the dimensions. 5 == PlaysStation button
        if button == 5:
            controller.workspace_width = controller.get_tracked_distance()
            print(f"Width {controller.workspace_width}, Height {controller.workspace_height}")
            logging.info(f"Width {controller.workspace_width}, Height {controller.workspace_height}")
            controller.reset_encoders()
            # Turn right one last time
            controller.target_angle = -180
            controller.mapping = False
            controller.homing = True
            _save_mapped_dimensions(controller.workspace_width, controller.workspace_height)
            controller.change_state(turn_state)


def _save_mapped_dimensions(m_width, m_height):
    with open(dimensions_file, "w") as f:
        f.write(f"{round(m_width, 2)},{round(m_height, 2)}\n")
        f.close()


def homing_state(controller: RobotController):
    controller.forward()
    if controller.homing_turns == 0:
        if controller.get_tracked_distance() >= controller.workspace_height:
            controller.target_angle = -270
            controller.homing_turns += 1
            controller.reset_encoders()
            controller.change_state(turn_state)
    elif controller.homing_turns == 1:
        if controller.get_tracked_distance() >= controller.workspace_width:
            controller.target_angle = -360
            controller.homing_turns += 1
            controller.reset_encoders()
            controller.homing = False
            controller.change_state(turn_state)


def cruise_state(controller: RobotController):
    logging.info(f"distance: {controller.get_tracked_distance()}")
    if (hist := controller.state_history) and hist[-1] == homing_state.__name__:
        controller.reset_angle()

    # If we reach the intended distance change to turn state
    if (distance := controller.get_tracked_distance()) >= controller.workspace_height:
        # Width/wheel_radius tells us how many turns we need to do to cover the area. If we have done that many turns
        # It means that we have covered the area
        if (controller.workspace_width // controller.WHEEL_RADIUS) <= controller.number_of_turns:
            if distance >= (controller.workspace_height + 45):
                controller.change_state(end_state)
                return
            controller.forward()
            return

        controller.reset_encoders()
        if controller.turn_right_next:
            controller.target_angle = -180
            controller.turn_right_next = False
        else:
            controller.target_angle = 0
            controller.turn_right_next = True
        controller.change_state(turn_state)
    else:
        controller.forward()

    # TODO: if there is an obstacle change to obstacle state

    # TODO: If we reached the end of the track and did the total number of turns we should stop


def turn_state(controller: RobotController):
    # If we were previously turning and now have corrected the direction reset
    # the encoders to ensure we start from 0 Meters again.
    deviation = controller.u_turn()
    if deviation <= controller.angle_error_margin:
        controller.send_speed(-1, -1)
        # The radius between the center point of the wheels is 35 centimeters
        controller.reset_encoders()
        controller.number_of_turns += 1
        controller.change_state(adjust_state)
        time.sleep(0.5)


def adjust_state(controller: RobotController):
    if controller.imu_forward() and controller.get_tracked_distance() >= 20:
        print(controller.get_tracked_distance())
        controller.left_pid.set_auto_mode(True, last_output=controller.TURNING_SPEED)
        controller.right_pid.set_auto_mode(True, last_output=controller.TURNING_SPEED)
        if controller.mapping:
            controller.change_state(map_state)
        elif controller.homing:
            controller.change_state(homing_state)
        else:
            controller.change_state(cruise_state)


# # TODO: Delete this state
# def boost_state(controller: RobotController):
#     if controller.get_tracked_distance() > 15:
#         controller.change_state(cruise_state)
#         return
#     controller.send_speed(85, 80)


def end_state(controller: RobotController):
    controller.halt()
    print("########Done########")
    print("Total width covered:", controller.number_of_turns * 35)
    exit(0)


robot = RobotController()
try:
    while True:
        robot.update()
except KeyboardInterrupt:
    robot.halt()
