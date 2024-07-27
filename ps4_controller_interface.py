import pygame
from pygame.locals import JOYBUTTONDOWN, JOYHATMOTION

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


def ps4_controller_interface():
    pass


try:
    while True:
        # Process events
        for event in pygame.event.get():
            if event.type in [JOYBUTTONDOWN]:
                button = event.button
                # if button in [11]:
                #     # controller.forward()
                #     print("Up")
                # elif button in [13, 14]:
                #     print("Left-Right")
                # else:
                print(button)
            elif event.type == JOYHATMOTION:
                print(event.value)
            horizontal = int(round(joystick.get_axis(0)))
            vertical = int(round(joystick.get_axis(1)))
            if not (horizontal == vertical == 0):
                print(f"horizontal: {horizontal} | vertical : {vertical}")
except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()
