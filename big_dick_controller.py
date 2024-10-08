import os
import time
import logging
import serial
import pygame
import threading
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


class ObstacleDetectionRoutine:
    def __init__(self, target_angle: int, remaining_turns: int):
        self.controller = None
        self.current_stage = self._stage_1
        self.turning = False
        self.tracked_distance = 0
        self.ticks_before_avoiding_obstacle = 0
        self.ticks_after_clearing_obstacle = 0
        self.ticks_obstacle_length = 0
        self.ultrasound_sequence = []
        self.done = False
        # TODO: This might need to be more than 1 lane since the robot is soo long
        if remaining_turns > 1:
            # if target angle is 0 turn direction is -1 so turn right
            # else turn left
            self.direction = -1 if target_angle == 0 else 1
        else:
            # if less 1 or not turns remain we must turn in the opposite direction
            self.direction = 1 if target_angle != 0 else -1

    def __call__(self, controller: 'RobotController'):
        if self.controller is None:
            self.controller = controller
            self.controller.cutting = False

        if self.done:
            controller.change_state(cruise_state)
            return

        if self.turning:
            print("Axis turn")
            self._axis_turn(controller)
        else:
            print("current stage", self.current_stage.__name__)
            self.current_stage(controller)

    def _axis_turn(self, controller: 'RobotController'):
        deviation = controller.axis_turn()
        if deviation <= controller.angle_error_margin:
            controller.reset_encoders()
            self.turning = False

    def _obstacle_passed(self, ultra_sound_value: int):
        if len(self.ultrasound_sequence) > 0:
            if self.ultrasound_sequence[-1] != ultra_sound_value:
                self.ultrasound_sequence.append(ultra_sound_value)

            if self.ultrasound_sequence in [[0, 1], [0, 1, 0], [1, 0]]:
                self.ultrasound_sequence = []
                print("Obstacle passed")
                return True
            return False
        else:
            self.ultrasound_sequence.append(ultra_sound_value)
            return False

    def _stage_1(self, controller: 'RobotController'):
        self.ticks_before_avoiding_obstacle = controller.sensor_data["left_encoder_raw"]
        controller.target_angle += self.direction * 90
        self.current_stage = self._stage_2
        self.turning = True

    def _stage_2(self, controller: 'RobotController'):
        # decide which ultrasound to use
        ultrasound = controller.sensor_data["left_ultrasound"] \
            if self.direction == -1 else controller.sensor_data["right_ultrasound"]
        print(f"Stage 1 ultrasound: {'left' if self.direction == -1 else 'right'} {ultrasound}")
        if controller.get_tracked_distance() > 20:
            self.ticks_after_clearing_obstacle = controller.sensor_data["left_encoder"]
            controller.target_angle += -self.direction * 90
            self.current_stage = self._stage_3
            self.turning = True
            controller.reset_encoders()
        else:
            controller.forward()

    def _stage_3(self, controller: 'RobotController'):
        ultrasound = controller.sensor_data["left_ultrasound"] \
            if self.direction == -1 else controller.sensor_data["right_ultrasound"]
        if controller.get_tracked_distance() > 20:
            self.ticks_obstacle_length = controller.sensor_data["left_encoder"]
            controller.target_angle += -self.direction * 90
            self.current_stage = self._stage_4
            self.turning = True
        else:
            controller.forward()

    def _stage_4(self, controller: 'RobotController'):
        if controller.get_tracked_distance() > 20:
            controller.total_ticks_left = self.ticks_before_avoiding_obstacle - self.ticks_obstacle_length
            controller.target_angle += self.direction * 90
            self.turning = True
            self.done = True
        else:
            controller.forward()


class RobotController:
    def __init__(self):
        self.TURNING_SPEED = 10
        self.LEFT_CRUISE_SPEED = 8
        self.RIGHT_CRUISE_SPEED = 8

        # In Centimeters
        self.WHEEL_RADIUS = 44
        self.CUTTER_DIAMETER = 15
        self.cutting = 0

        # self.sonic_ser = serial.Serial('/dev/arduinoUltrasound', 115200, timeout=1)
        # time.sleep(3)
        self.angle_ser = serial.Serial('/dev/arduinoSensors', 115200, timeout=1)
        time.sleep(2)
        self.motor_ser = serial.Serial('/dev/arduinoMotors', 115200, timeout=1)

        self.state_history = []
        self.change_state(init_state)

        self.sensor_data: dict = {"front_ultrasound_1": 0, "front_ultrasound_2": 0,
                                  "right_ultrasound": 0, "left_ultrasound": 0}
        self.target_angle = 0
        self.angle_delta = 0
        self.angle_error_margin = 1
        self.turning = False
        self.still_turning = False

        self.distance_per_tick = 0.70
        self.turn_right_next = True

        self.total_ticks_left = 0
        self.total_ticks_right = 0
        self.number_of_turns = 0

        # In centimeters
        self.workspace_height = 0
        self.workspace_width = 0
        self.required_turns = 0

        self.homing_turns = 0
        self.homing = False
        self.mapping = False

        self.boost_increase = 0
        self.cached_speeds = (0, 0)
        self.distance_after_encoder_reset = 0

        # self.stop_event = threading.Event()
        # self.ultrasound_thread = threading.Thread(target=self.read_ultrasound_data, daemon=True)
        # self.ultrasound_thread.start()

    def update(self):
        self.read_angle_data()
        self.current_state(self)
        self._controller_input()

    def _controller_input(self):
        for event in pygame.event.get():
            # R1: 5, R2: 7
            # L1: 4, L2: 6

            if event.type not in [JOYBUTTONDOWN, JOYHATMOTION]:
                continue
            button = event.button if event.type == JOYBUTTONDOWN else event.value
            change_factor = 2
            if button in [5, 7]:
                change = +change_factor if button == 5 else -change_factor
                print(f"Right Motor Speed: {self.RIGHT_CRUISE_SPEED} -> {self.RIGHT_CRUISE_SPEED + change}")
                self.RIGHT_CRUISE_SPEED = min(self.RIGHT_CRUISE_SPEED + 10, self.RIGHT_CRUISE_SPEED + change)
            elif button in [4, 6]:
                change = +change_factor if button == 4 else -change_factor
                print(f"Left Motor Speed: {self.LEFT_CRUISE_SPEED} -> {self.LEFT_CRUISE_SPEED + change}")
                self.LEFT_CRUISE_SPEED = min(self.LEFT_CRUISE_SPEED + 10, self.LEFT_CRUISE_SPEED + change)
            elif button == 10:
                print("#"*10, "EMERGENCY STOP", "#"*10)
                self.change_state(end_state)
                return

            # MANUAL MODE
            if self.current_state.__name__ == "manual_state":
                pygame.event.pump()
                if button == (1, 0):
                    self.send_speed(self.LEFT_CRUISE_SPEED, -self.RIGHT_CRUISE_SPEED)
                elif button == (-1, 0):
                    self.send_speed(-self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
                elif button == (0, 1):
                    # self.forward()
                    self.send_speed(self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
                elif button == (0, -1):
                    self.send_speed(-self.LEFT_CRUISE_SPEED, -self.RIGHT_CRUISE_SPEED)
                elif button == 12:
                    self.cutting = not self.cutting
                    self.send_speed(0, 0)
                else:
                    self.send_speed(0, 0)

    def change_state(self, new_state):
        state_name = new_state.__name__ if hasattr(new_state, "__name__") else type(new_state).__name__
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
                self.send_speed(self.TURNING_SPEED, -self.TURNING_SPEED - 1)
            elif self.sensor_data["angle"] < self.target_angle:
                self.send_speed(-self.TURNING_SPEED, self.TURNING_SPEED + 1)
        return deviation

    def u_turn(self):
        deviation = self.get_angle_deviation()
        logging.info(
            f"U_TURN Target Angle: {self.target_angle} | Real Angle: {self.sensor_data['angle']} | Deviation: {deviation}")
        if deviation > self.angle_error_margin:
            # IF angle is positive stop right wheel and increase left wheel speed
            if self.sensor_data["angle"] > self.target_angle:
                self.send_speed(self.LEFT_CRUISE_SPEED + 2, 0)
            elif self.sensor_data["angle"] < self.target_angle:
                self.send_speed(0, self.RIGHT_CRUISE_SPEED + 2)
        return deviation

    def forward(self):
        deviation = self.get_angle_deviation()
        # logging.info(f"FORWARD: Target Angle: {self.target_angle} | "
        #              f"Real Angle: {self.sensor_data['angle']} | Deviation: {deviation}")
        if deviation > self.angle_error_margin:
            # IF angle is positive stop right wheel and increase left wheel speed
            if self.sensor_data["angle"] > self.target_angle:
                self.send_speed(self.LEFT_CRUISE_SPEED, 0)
            elif self.sensor_data["angle"] < self.target_angle:
                self.send_speed(0, self.RIGHT_CRUISE_SPEED)
        else:
            self.send_speed(self.LEFT_CRUISE_SPEED, self.RIGHT_CRUISE_SPEED)
        return deviation

    def send_speed(self, left_speed, right_speed):
        # v_d,v_i,d_d,d_i,s_d,s_i,cut
        r_speed = abs(min(right_speed, self.RIGHT_CRUISE_SPEED + 10))
        r_dir = 1
        r_stop = 0
        if right_speed <= 0:
            r_dir = 0
            r_stop = 1 if right_speed == 0 else 0

        l_speed = abs(min(left_speed, self.LEFT_CRUISE_SPEED + 10))
        l_dir = 1
        l_stop = 0
        if left_speed <= 0:
            l_dir = 0
            l_stop = 1 if left_speed == 0 else 0

        cutter = 0 #int(self.cutting)
        # right wheel, left_wheel
        command = f"{r_speed},{r_dir},{r_stop},{l_speed},{l_dir},{l_stop},{cutter}\n"
        logging.info(f"Speed Message: {command}",)
        self.motor_ser.write(command.encode())

    def read_angle_data(self):
        self.angle_ser.flushInput()
        angle_data = self.angle_ser.readline().decode('utf-8').strip()
        angle_data_list = angle_data.split(",")
        if len(angle_data_list) != 3:
            print("Error on angle ser parsing: ", angle_data)
            return

        angle, right_encoder, left_encoder = angle_data_list
        # print("Angle: ", angle)
        self.sensor_data["angle"] = round(float(angle) - self.angle_delta)
        self.sensor_data["left_encoder"] = abs(float(left_encoder)) - self.total_ticks_left
        self.sensor_data["left_encoder_raw"] = abs(float(left_encoder))
        self.sensor_data["right_encoder"] = abs(float(right_encoder)) - self.total_ticks_right
        self.sensor_data["right_encoder_raw"] = abs(float(right_encoder))

    def read_ultrasound_data(self):
        while not self.stop_event.is_set():
            self.sonic_ser.flushInput()
            ultrasound_data = self.sonic_ser.readline().decode("utf-8").strip()
            ultrasound_data_list = ultrasound_data.split(",")
            if len(ultrasound_data_list) != 4:
                continue
            right_ultrasound, left_ultrasound, front_ultra_1, front_ultra_2 = ultrasound_data_list
            # print(f"R {right_ultrasound}, L {left_ultrasound}")
            self.sensor_data["left_ultrasound"] = int(left_ultrasound)
            self.sensor_data["right_ultrasound"] = int(right_ultrasound)
            self.sensor_data["front_ultrasound_1"] = int(front_ultra_1)
            self.sensor_data["front_ultrasound_2"] = int(front_ultra_2)

    def reset_encoders(self):
        self.send_speed(0, 0)
        time.sleep(0.5)
        self.read_angle_data()
        self.total_ticks_left = self.sensor_data["left_encoder_raw"]
        self.total_ticks_right = self.sensor_data["right_encoder_raw"]
        logging.info(f"Total Encoder Ticks: L {self.total_ticks_left} | R {self.total_ticks_right}")

    def reset_angle(self):
        self.angle_delta = self.sensor_data["angle"]

    def get_tracked_distance(self):
        covered_distance = round(self.distance_per_tick * self.sensor_data["left_encoder"], 2)
        return covered_distance

    def get_tracked_distance_right(self):
        covered_distance = round(self.distance_per_tick * self.sensor_data["right_encoder"], 2)
        return covered_distance

    def halt(self):
        print("Exiting Program")
        self.cutting = False
        self.send_speed(0, 0)
        #self.stop_event.set()
        # self.ultrasound_thread.join()
        self.motor_ser.close()
        self.angle_ser.close()
        # self.sonic_ser.close()


def init_state(controller: RobotController):
    if not controller.sensor_data:
        return

    controller.send_speed(0, 0)
    print("Adding delay")
    time.sleep(1)
    controller.change_state(map_state)


def manual_state(controller: RobotController):
    print("Angle:", controller.sensor_data["angle"])
    return


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

        if controller.mapping:
            # Once we start turning it MUST mean we reached a corner
            if button == (1, 0):
                # 13 == right d-pad, 14 == left d-pad
                controller.workspace_height = controller.get_tracked_distance()
                controller.reset_encoders()
                controller.target_angle = -90
                controller.change_state(turn_state)

            # Finish the mapping and save the dimensions. (0, 0) is d-pad bellow button
            if button == (0, -1):
                # I add 10 Cm to the width because it seems to fall short most times.
                controller.workspace_width = controller.get_tracked_distance()
                controller.required_turns = controller.workspace_width // controller.CUTTER_DIAMETER
                m = f"Width {controller.workspace_width}, Height {controller.workspace_height}"
                logging.info(m)
                controller.reset_encoders()
                # Turn right one last time
                controller.target_angle = -180
                controller.mapping = False
                controller.homing = True
                _save_mapped_dimensions(controller.workspace_width, controller.workspace_height)
                controller.change_state(turn_state)

        if controller.mapping is False and button == 3:
            controller.change_state(manual_state)
            return

        # IMPORTANT:
        # 2 == Y button; pressing this means use stored mapping
        if button == 2 and not controller.mapping:
            controller.workspace_width, controller.workspace_height = _load_saved_dimensions()
            m = f"Saved dimensions: width = {controller.workspace_width} | height = {controller.workspace_height}"
            print(m)
            logging.info(m)
            if not (controller.workspace_width == controller.workspace_height == 0):
                controller.required_turns = controller.workspace_width // controller.CUTTER_DIAMETER
                controller.change_state(boost_state)
            else:
                print("######### NO AREA DIMENSIONS ARE STORED. YOU MUST MAP THE AREA #########")


dimensions_file = "saved_work_area_dimensions.txt"
def _save_mapped_dimensions(m_width, m_height):
    with open(dimensions_file, "w") as f:
        f.write(f"{round(m_width, 2)},{round(m_height, 2)}\n")


def _load_saved_dimensions() -> (float, float):
    if os.path.isfile(dimensions_file):
        with open(dimensions_file, "r") as f:
            wh = f.readline()
            width, height = (float(i) for i in wh.split(","))
        return width, height
    return 0, 0


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
    controller.cutting = True

    if controller.angle_delta == 0 and "homing_state" in controller.state_history:
        print("Resetting Angle")
        controller.target_angle = 0
        controller.reset_angle()
    # If we reach the intended distance change to turn state
    objective_distance = controller.CUTTER_DIAMETER if controller.still_turning else controller.workspace_height
    if (distance := controller.get_tracked_distance()) >= objective_distance:
        # Width/wheel_radius tells us how many turns we need to do to cover the area. If we have done that many turns
        # It means that we have covered the area
        if controller.required_turns <= controller.number_of_turns:
            if distance >= (controller.workspace_height + 45):
                controller.change_state(end_state)
                return
            controller.forward()
            return

        controller.reset_encoders()
        controller.change_state(turn_state)

    elif 0: #controller.sensor_data["front_ultrasound_1"] or controller.sensor_data["front_ultrasound_2"]:
        turns_left = controller.required_turns - controller.number_of_turns
        controller.reset_encoders()
        controller.change_state(ObstacleDetectionRoutine(controller.target_angle, turns_left))
    else:
        controller.forward()


# # TODO: Delete this
# def boosting_protocol_turning(controller: RobotController, increase: int | float, boost_distance: int, limit: int):
#     if controller.cached_speeds == (0, 0):
#         controller.cached_speeds = (controller.TURNING_SPEED, controller.TURNING_SPEED)
#         controller.TURNING_SPEED = 180
#     tracked_distance = controller.get_tracked_distance_right() if controller.turn_right_next else controller.get_tracked_distance()
#
#     if tracked_distance < boost_distance:
#         controller.TURNING_SPEED = min(limit, controller.TURNING_SPEED + increase)
#         print(f"Turn Current Speed: {controller.TURNING_SPEED}")
#         return False
#     else:
#         cache = controller.cached_speeds
#         controller.TURNING_SPEED = cache[0]
#         controller.cached_speeds = (0, 0)
#         return True
#

def turn_state(controller: RobotController):
    # controller.cutting = True if not (controller.mapping or controller.homing) else False
    if not controller.turning and not (controller.mapping or controller.homing or controller.state_history[-2] == "homing_state"):
        controller.target_angle += -90 if controller.turn_right_next else 90
        print(f"Turning {'Right' if controller.turn_right_next else 'Left'} and target angle {controller.target_angle}")
    controller.turning = True

    deviation = controller.axis_turn()
    # tracked_distance = controller.get_tracked_distance() # controller.get_tracked_distance_right() if controller.turn_right_next else controller.get_tracked_distance()
    if deviation <= controller.angle_error_margin:
        controller.reset_encoders()
        if controller.mapping:
            controller.change_state(map_state)
        elif controller.homing:
            controller.change_state(homing_state)
        else:
            controller.turning = False
            if controller.state_history[-2] == "homing_state":
                controller.change_state(cruise_state)
                return

            if controller.target_angle == -180:
                controller.turn_right_next = False
                controller.still_turning = False
                controller.number_of_turns += 1
            elif controller.target_angle == 0:
                controller.turn_right_next = True
                controller.still_turning = False
                controller.number_of_turns += 1
            else:
                controller.still_turning = True
            controller.change_state(cruise_state)

    # elif int(tracked_distance) < 60:
    #     increase = 0.5
    #     if controller.cached_speeds == (0, 0):
    #         controller.cached_speeds = (controller.TURNING_SPEED, controller.TURNING_SPEED)
    #         controller.TURNING_SPEED = 180
    #
    #     controller.TURNING_SPEED = min(255, controller.TURNING_SPEED + increase)
    #     print(f"Turn Current Speed: {controller.TURNING_SPEED}")
    #
    # elif controller.cached_speeds != (0, 0):
    #     cache = controller.cached_speeds
    #     controller.TURNING_SPEED = cache[0]
    #     controller.cached_speeds = (0, 0)


def adjust_state(controller: RobotController):
    controller.cutting = False
    deviation = controller.get_angle_deviation()
    print(f"Adjust dev: {deviation}, error: {controller.sensor_data['angle']}")
    if deviation >= controller.angle_error_margin:
        print(f"Adjust speed: {controller.TURNING_SPEED}")
        # IF angle is positive stop right wheel and increase left wheel speed
        if controller.sensor_data["angle"] > controller.target_angle:
            controller.send_speed(-controller.TURNING_SPEED - 15, controller.TURNING_SPEED - 15)
        elif controller.sensor_data["angle"] < controller.target_angle:
            controller.send_speed(controller.TURNING_SPEED - 15, -controller.TURNING_SPEED - 15)
    else:
        time.sleep(1)
        controller.reset_encoders()
        controller.change_state(boost_state)


# TODO: delete this
# def boosting_protocol(controller: RobotController, increase: int | float, boost_distance: int, limit: int):
#     if controller.cached_speeds == (0, 0):
#         controller.distance_after_encoder_reset = controller.get_tracked_distance()
#         controller.cached_speeds = (controller.LEFT_CRUISE_SPEED, controller.RIGHT_CRUISE_SPEED)
#     print("Boost distance: ", controller.get_tracked_distance(), controller.sensor_data["left_encoder"])
#     if (t := (controller.get_tracked_distance() - controller.distance_after_encoder_reset)) < boost_distance:
#         controller.LEFT_CRUISE_SPEED = min(limit, controller.LEFT_CRUISE_SPEED + increase)
#         controller.RIGHT_CRUISE_SPEED = min(limit, controller.RIGHT_CRUISE_SPEED + increase)
#         print(f"Boost Current Speeds: L = {controller.LEFT_CRUISE_SPEED} "
#               f"| R = {controller.RIGHT_CRUISE_SPEED}")
#         return False
#     else:
#         cache = controller.cached_speeds
#         controller.LEFT_CRUISE_SPEED = cache[0]
#         controller.RIGHT_CRUISE_SPEED = cache[1]
#         controller.distance_after_encoder_reset = 0
#         controller.cached_speeds = (0, 0)
#         return True


def boost_state(controller: RobotController):
    controller.cutting = True
    if False: #not boosting_protocol(controller, 0.5, 15, 255):
        controller.forward()
    else:
        controller.change_state(cruise_state)


def end_state(controller: RobotController):
    controller.halt()
    print("########Done########")
    print("Total width covered:", controller.number_of_turns * controller.CUTTER_DIAMETER)
    exit(0)


robot = RobotController()


try:
    while True:
        robot.update()
except KeyboardInterrupt:
    robot.halt()
