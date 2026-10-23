import pygame
from lib.RobotController import Robot
from lib.draw import Drawer
from lib.CommModule import CommModule
from lib.RealWorldObserver import Observer
from lib.UserInputManager import UserInputManager

# ------------------------------ Init
pygame.init()
clock = pygame.time.Clock()
comm = CommModule("configs/comms.yml")
robot = Robot("configs/robot.yml")
drawer = Drawer("configs/graphics.yml", robot, camera_mode="camera")
# [[980, 1234], [880, 120], [1367, 102], [2145, 1001]]
predefined_points = [[980, 1234], [920, 646], [1354, 617], [1561, 1125]]
observer = Observer(predefined_points=predefined_points, screen=drawer.screen)
ui = UserInputManager(debug=True)
FPS = 20
running = True

while running:

    inputs, running = ui.check_for_inputs()  # keyboard + joystick + comm

    if inputs:
        # comm.send(inputs) # Send surrogate robot latest inputs ...
        # ... and apply those to the local robot model
        robot.apply_inputs(inputs)

    # Update direction based on target path and current pose
    robot.run_controller()

    # Update pose based on sensors and/or models
    if (pose := robot.estimate_next_pose(dt=1.0)) is not None:
        robot.set_pose(pose)

    # Render simulation and observer view
    drawer.draw_scene(robot, observer)

    clock.tick(FPS)  # emulate the desired framerate

pygame.quit()
