import pygame
from lib.CartController import Cart
from lib.draw import Drawer
from lib.path import get_job_path
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

cart = Cart("configs/cart.yml")
drawer = Drawer("configs/graphics.yml", cart, camera_mode="camera")


# [[980, 1234], [880, 120], [1367, 102], [2145, 1001]]
predefined_points = [[980, 1234], [920, 646], [1354, 617], [1561, 1125]]

observer = Observer(predefined_points=predefined_points)
observer.run_calibration(drawer.screen)

job_path = get_job_path(
    cart, margin_x=0.0, margin_y=0.0, safety_buffer=cart.specs.turn_radius + 0.2)

cart.set_job_path(job_path)

comm = CommModule(ESP32_IP, ESP32_PORT, bind_ip=LOCAL_IP)
ui = UserInputManager(comm_module=comm, debug=True)

running = True
while running:
    clock.tick(FPS)

    # -------------------------------------------------------------------
    # simulate metrology data with computed estimate
    measured_cart_pose = cart.estimate_next_pose(dt=1.0)

    if measured_cart_pose is not None:

        # update model pose based on metrology
        cart.set_pose(measured_cart_pose)

    # -------------------------------------------------------------------

    inputs, running = ui.check_for_inputs()  # keyboard + joystick + comm

    # -------------------------------------------------------------------

    if inputs:
       # comm.send(inputs)          # Send surrogate robot latest inputs ...
        # ... and apply those to the local cart model
        cart.apply_inputs(inputs)

    # -------------------------------------------------------------------

    cart.run_controller()  # Update direction based on target path and current pose

    # -------------------------------------------------------------------

    # Render simulation and observer view
    drawer.draw_scene(job_path, cart, observer)

pygame.quit()
