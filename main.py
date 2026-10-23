import pygame
import yaml
from lib.RobotController import Robot
from lib.draw import Drawer
from lib.CommModule import CommModule
from lib.RealWorldObserver import Observer
from lib.UserInputManager import UserInputManager

FPS = 20

# ------------------------------ Init
with open("configs/comms.yml") as f:
    comms_enabled = yaml.safe_load(f)["network"]["enabled"]
with open("configs/camera.yml") as f:
    camera_cfg = yaml.safe_load(f)["camera"]

pygame.init()
clock = pygame.time.Clock()
comm = CommModule("configs/comms.yml") if comms_enabled else None
robot = Robot("configs/robot.yml")
drawer = Drawer("configs/graphics.yml", robot, camera_mode="camera")
observer = Observer(world_rect=robot.world_rect,
                    snapshot_path=camera_cfg["snapshot"],
                    predefined_points=camera_cfg["calibration_points"],
                    screen=drawer.screen)
ui = UserInputManager(debug=True)
running = True

while running:

    inputs, running = ui.check_for_inputs()  # keyboard + joystick

    if inputs:
        if comm:
            comm.send(inputs)  # Send surrogate robot latest inputs ...
        # ... and apply those to the local robot model
        robot.apply_inputs(inputs)

    # Update direction based on target path and current pose
    robot.run_controller()

    # Update pose based on sensors and/or models
    if (pose := robot.estimate_next_pose(dt=1.0 / FPS)) is not None:
        robot.set_pose(pose)

    # Render simulation and observer view
    drawer.draw_scene(robot, observer)

    clock.tick(FPS)  # emulate the desired framerate

pygame.quit()
