import os
import time
import logging
import serial
import pygame
from pygame.locals import JOYBUTTONDOWN, JOYHATMOTION

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
        self.TURNING_SPEED = 140 
        self.LEFT_CRUISE_SPEED = 130
        self.RIGHT_CRUISE_SPEED = 130
        # In Centimeters
        self.WHEEL_RADIUS = 35

        self.motor_ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.angle_ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        self.current_state = init_state
        self.state_history = []
        self.sensor_data: dict = dict()
        self.target_angle = 0
        self.angle_delta = 0
        self.angle_error_margin = 1

        self.distance_per_tick = 0.021
        self.turn_right_next = True

        self.total_ticks = 0
        self.number_of_turns = -1

        # In centimeters. Just for testing
        self.workspace_height = height if height is not None else 0
        self.workspace_width = width if width is not None else 0 #4 * self.WHEEL_RADIUS

        self.homing_turns = 0
        self.homing = False
        self.mapping = False

        self.obstacle_stage = 0
        self.ticks_before_avoiding_obstacle = None
        self.ticks_after_clearing_obstacle = None
        self.ticks_obstacle_length = None

    def update(self):
        self.update_sensor_readings()
        if self.current_state.__name__ != "map_state":
            self._controller_input()
        self.current_state(self)

    def _controller_input(self):
        for event in pygame.event.get():
            # R1: 5, R2: 7
            # L1: 4, L2: 6
            if event.type not in [JOYBUTTONDOWN]:
                continue
            button = event.button
            if button in [5, 7]:
                change = +2 if button == 5 else -2
                print(f"Right Motor Speed: {self.RIGHT_CRUISE_SPEED} -> {self.RIGHT_CRUISE_SPEED + change}")
                self.RIGHT_CRUISE_SPEED += change
            elif button in [4, 6]:
                change = +2 if button == 4 else -2
                print(f"Left Motor Speed: {self.LEFT_CRUISE_SPEED} -> {self.LEFT_CRUISE_SPEED + change}")
                self.LEFT_CRUISE_SPEED += change

    def change_state(self, new_state):
        state_name = new_state.__name__
        print("NEW STATE: ", state_name)
        logging.info(f"NEW STATE: {state_name}")
        self.state_history.append(state_name)
        self.current_state = new_state

    def get_angle_deviation(self):
        return abs(abs(self.sensor_data["angle"]) - abs(self.target_angle))

    def axis_turn(self):
        deviation = self.get_angle_deviation()
        if deviation > self.angle_error_margin:
            if self.sensor_data["angle"] > self.target_angle:
                self.send_speed(self.TURNING_SPEED, -self.TURNING_SPEED)
            elif self.sensor_data["angle"] < self.target_angle:
                self.send_speed(-self.TURNING_SPEED, self.TURNING_SPEED)
        return deviation

    def forward(self):
        deviation = self.get_angle_deviation()
        logging.info(f"Target Angle: {self.target_angle} | Real Angle: {self.sensor_data['angle']} | Deviation: {deviation}")
        if deviation > self.angle_error_margin:
            # IF angle is positive stop right wheel and increase left wheel speed
            if self.sensor_data["angle"] > self.target_angle:
                # print("Turning Right")
                self.send_speed(self.LEFT_CRUISE_SPEED, 0)
            elif self.sensor_data["angle"] < self.target_angle:
                # print("Turning Left")
                self.send_speed(0, self.RIGHT_CRUISE_SPEED)
        else:
            self.send_speed(self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
        return deviation

    def send_speed(self, left_speed, right_speed):
        command = f"{right_speed},{left_speed}\n"
        # print(f"R:{right_speed},L:{left_speed}")
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
            "rear_ultrasound": 0,
            "front_1_ultrasound": 0,
            "front_2_ultrasound": 0,
            "right_encoder": float(right_encoder)
        }

    def reset_encoders(self):
        # Since resetting encoders in the arduino code does not actually reset the encoder
        # values sent due to some communication issue we have to keep track of ticks taken
        # during turning manually like cavemen
        self.total_ticks = self.sensor_data["left_encoder_raw"]
        self.send_speed(0, 0)
        # Stop for half a second to let the motors slow down
        time.sleep(0.5)

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
        if event.type not in [JOYBUTTONDOWN, JOYHATMOTION]:
            continue
        button = event.button if event.type == JOYBUTTONDOWN else event.value
        # 0 == X button; pressing this means we start mapping
        if button == 0:
            controller.mapping = True

        # IMPORTANT:
        # 3 == Y button; pressing this means use stored mapping
        if button == 2 and not controller.mapping:
            if not (controller.workspace_width == controller.workspace_height == 0):
                controller.change_state(cruise_state)

        # Once we start turning it MUST mean we reached a corner
        if button == (1,0):
            # 13 == right d-pad, 14 == left d-pad
            controller.workspace_height = controller.get_tracked_distance()
            controller.reset_encoders()
            controller.target_angle = -90
            controller.change_state(turn_state)

        # Finish the mapping and save the dimensions. 5 == PlaysStation button
        if button == 10:
            # I add 10 Cm to the width because it seems to fall short most times.
            controller.workspace_width = controller.get_tracked_distance() + 10
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
    if controller.angle_delta == 0 and len(controller.state_history) >= 4 and controller.state_history[4] == "homing_state":
        controller.target_angle = 0
        controller.reset_angle()

    # If we reach the intended distance change to turn state
    if (distance := controller.get_tracked_distance()) >= controller.workspace_height:
        # Width/wheel_radius tells us how many turns we need to do to cover the area. If we have done that many turns
        # It means that we have covered the area
        if (controller.workspace_width // controller.WHEEL_RADIUS) <= (controller.number_of_turns - 1):
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
    # Obstacle detection
    # elif controller.sensor_dat3["front_1_ultrasound"] or controller.sensor_data["front_2_ultrasound"]:
    #     controller.change_state(obstacle_state)
    else:
        controller.forward()


def obstacle_state(controller: RobotController):
    if controller.obstacle_stage == 0:
        controller.ticks_before_avoiding_obstacle = controller.sensor_data["left_encoder_raw"]
        # TODO: We must figure out which direction we must turn. Left or right.
        controller.target_angle += - 90
        controller.obstacle_stage += 1
        controller.change_state(axis_turn_state)
        return
    elif controller.obstacle_stage == 1:
        controller.forward()
        # TODO: This needs to check if the rear ultrasound is 0 just after turning.
        #  If it must we must advance until it's value is 1 and then 0
        if controller.sensor_data["rear_ultrasound"] == 0:
            controller.ticks_after_clearing_obstacle = controller.sensor_data["left_encoder"]
            controller.target_angle += 90
            controller.obstacle_stage += 1
            controller.change_state(axis_turn_state)
            return
    elif controller.obstacle_stage == 2:
        # TODO: This needs to check if the rear ultrasound is 0 just after turning.
        #  If it must we must advance until it's value is 1 and then 0
        if controller.sensor_data["rear_ultrasound"] == 0:
            controller.ticks_obstacle_length = controller.sensor_data["left_encoder"]
            controller.target_angle += 90
            controller.obstacle_stage += 1
            controller.change_state(axis_turn_state)
            return
    elif controller.obstacle_stage == 3:
        if controller.sensor_data["left_encoder"] >= controller.ticks_after_clearing_obstacle:
            controller.total_ticks = controller.ticks_before_avoiding_obstacle + controller.ticks_obstacle_length
            controller.target_angle += -90
            controller.obstacle_stage += 1
            controller.change_state(axis_turn_state)
            return
    elif controller.obstacle_stage == 4:
        controller.change_state(cruise_state)
    controller.forward()


def axis_turn_state(controller: RobotController):
    deviation = controller.axis_turn()
    if deviation <= controller.angle_error_margin:
        controller.reset_encoders()
        controller.change_state(obstacle_state)


def turn_state(controller: RobotController):
    # If we were previously turning and now have corrected the direction reset
    # the encoders to ensure we start from 0 Meters again.
    deviation = controller.forward()
    if deviation <= controller.angle_error_margin:
        # The radius between the center point of the wheels is 35 centimeters
        controller.reset_encoders()
        if controller.mapping:
            controller.change_state(map_state)
        elif controller.homing:
            controller.change_state(homing_state)
        else:
            logging.info(controller.state_history)
            controller.number_of_turns += 1
            controller.change_state(boost_state)


def boost_state(controller: RobotController):
    if controller.get_tracked_distance() > 10:
        controller.change_state(cruise_state)
        return
    controller.send_speed(controller.LEFT_CRUISE_SPEED + 15, controller.RIGHT_CRUISE_SPEED + 15)


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
