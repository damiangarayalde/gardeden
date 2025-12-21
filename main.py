import pygame
from lib.RobotController import Robot
from lib.draw import Drawer
from lib.CommModule import CommModule
from lib.RealWorldObserver import Observer
from lib.UserInputManager import UserInputManager

# ====== Network settings ======
ESP32_IP = "192.168.0.16"
ESP32_PORT = 4210
LOCAL_IP = "0.0.0.0"   # listen on all interfaces
FPS = 20


# ------------------------------ Init
pygame.init()
clock = pygame.time.Clock()

robot = Robot("configs/robot.yml")
drawer = Drawer("configs/graphics.yml", robot, camera_mode="camera")
# [[980, 1234], [880, 120], [1367, 102], [2145, 1001]]
predefined_points = [[980, 1234], [920, 646], [1354, 617], [1561, 1125]]
observer = Observer(predefined_points=predefined_points, screen=drawer.screen)


comm = CommModule(ESP32_IP, ESP32_PORT, bind_ip=LOCAL_IP)
ui = UserInputManager(comm_module=comm, debug=True)

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
