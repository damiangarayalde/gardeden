import pygame
from lib.CartController import Cart
from lib.draw import Drawer
from lib.path import get_job_path
from lib.CommModule import CommModule

# ====== Network settings ======
ESP32_IP = "192.168.0.16"
ESP32_PORT = 4210
LOCAL_IP = "0.0.0.0"   # listen on all interfaces
FPS = 60

JS_BTN_OPTIONS = 6
JS_BTN_START = 5
JS_BTN_L1 = 10
JS_BTN_R1 = 9
JS_AXIS_LEFT_HORIZONTAL = 0
# JS_AXIS_LEFT_VERTICAL = 1
# JS_AXIS_LEFT_HORIZONTAL = 2
JS_AXIS_RIGHT_VERTICAL = 3


def keyboard_to_kv(key) -> dict:
    """Map pressed keys to simple key-value dict."""
    mapping = {
        pygame.K_a: {"turn_left": 1},
        pygame.K_d: {"turn_right": 1},
        pygame.K_z: {"disturb_left": 1},
        pygame.K_c: {"disturb_right": 1},
        pygame.K_w: {"faster": 1},
        pygame.K_s: {"slower": 1},
        pygame.K_p: {"toggle_pause": 1},
        pygame.K_t: {"toggle_tracking": 1},
    }
    return mapping.get(key, {})


def joystick_to_kv(joystick) -> dict:
    """
    Map joystick buttons/axes to ControlInput.
    Adjust mapping for Kolke KGJ-182 if needed.
    Return raw joystick state as key-value dict.
    """
    # Axes usually return values between -1.0 and 1.0
    axis_omega = joystick.get_axis(JS_AXIS_LEFT_HORIZONTAL)
    axis_speed = joystick.get_axis(JS_AXIS_RIGHT_VERTICAL)

    # Apply deadzone
    deadzone = 0.2
    if abs(axis_omega) < deadzone:
        axis_omega = 0.0
    if abs(axis_speed) < deadzone:
        axis_speed = 0.0

    return {
        "speed_percent":    int(-axis_speed * 255),   # forward/backward
        "omega":            int(axis_omega * 255),
        "toggle_pause":     1 if joystick.get_button(JS_BTN_START) else 0,
        "toggle_tracking":  1 if joystick.get_button(JS_BTN_OPTIONS) else 0,
        "disturb_left":     1 if joystick.get_button(JS_BTN_L1) else 0,
        "disturb_right":    1 if joystick.get_button(JS_BTN_R1) else 0,
    }


# ------------------------------ Init
pygame.init()
clock = pygame.time.Clock()
pygame.joystick.init()
joystick = None

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")
else:
    print("⚠️ No joystick detected. Keyboard only.")


cart = Cart("configs/cart.yml")
drawer = Drawer("configs/graphics.yml", cart)

job_path = get_job_path(cart)
cart.set_job_path(job_path)

# Comms
comm = CommModule(ESP32_IP, ESP32_PORT, bind_ip=LOCAL_IP)

running = True
while running:
    clock.tick(FPS)

    # -------------------------------------------------------------------
    #  Updates cart pose based on metrology

    # simulate metrology data with computed estimate
    measured_cart_pose = cart.estimate_next_pose(dt=1.0)

    if measured_cart_pose is not None:
        cart.set_pose(measured_cart_pose)  # update pose

    # -------------------------------------------------------------------
    # Check for inputs: local (keyboard or joystick + remote)

    inputs = {}

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            inputs.update(keyboard_to_kv(event.key))

    # Poll joystick separately each frame
    if joystick:
        inputs.update(joystick_to_kv(joystick))

    # Handle incoming data
    # remote_inputs = comm.receive()
    # if remote_inputs:
    #     print("Remote inputs:", remote_inputs)

    # Send inputs if any
    if inputs:
        comm.send(inputs)
        cart.apply_inputs(inputs)

    # -------------------------------------------------------------------
    # Update
    #  //cart.updateMotors();
    cart.run_controller()

    drawer.draw_scene(job_path, cart)

pygame.quit()
