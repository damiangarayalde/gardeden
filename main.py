import pygame
from lib.CartController import Cart
from lib.draw import Drawer
from lib.path import get_job_path
from lib.CommModule import CommModule
from lib.RealWorldObserver import Observer

# ====== Network settings ======
ESP32_IP = "192.168.0.16"
ESP32_PORT = 4210
LOCAL_IP = "0.0.0.0"   # listen on all interfaces
FPS = 20

JS_BTN_OPTIONS = 6
JS_BTN_START = 5
JS_BTN_L1 = 10
JS_BTN_R1 = 9
JS_BTN_SQUARE = 2
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


def joystick_axes_to_kv(joystick) -> dict:
    """Poll joystick axes every frame and return continuous inputs.
    Map joystick buttons/axes to ControlInput.
    Adjust mapping for Kolke KGJ-182 if needed.
    Return raw joystick state as key-value dict.
    """
    axis_omega = joystick.get_axis(JS_AXIS_LEFT_HORIZONTAL)
    axis_speed = joystick.get_axis(JS_AXIS_RIGHT_VERTICAL)

    # Deadzone
    deadzone = 0.2
    if abs(axis_omega) < deadzone:
        axis_omega = 0.0
    if abs(axis_speed) < deadzone:
        axis_speed = 0.0

    return {
        "speed_percent": int(-axis_speed * 255),  # forward/backward
        "omega_percent": int(axis_omega * 255),
    }


def joystick_event_to_kv(event) -> dict:
    """Handle joystick button events and return discrete inputs."""
    if event.type == pygame.JOYBUTTONDOWN:
        if event.button == JS_BTN_START:
            return {"toggle_pause": 1}
        elif event.button == JS_BTN_OPTIONS:
            return {"toggle_tracking": 1}
        elif event.button == JS_BTN_L1:
            return {"disturb_left": 1}
        elif event.button == JS_BTN_R1:
            return {"disturb_right": 1}
        elif event.button == JS_BTN_SQUARE:
            return {"horn_pressed": 1}

    elif event.type == pygame.JOYBUTTONUP:
        # Release events (optional, e.g. reset toggle or send 0)
        if event.button == JS_BTN_L1:
            return {"disturb_left": 0}
        elif event.button == JS_BTN_R1:
            return {"disturb_right": 0}
        elif event.button == JS_BTN_SQUARE:
            return {"horn_released": 1}

    return {}


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
# observer = Observer()
# observer.run_calibration()

job_path = get_job_path(
    cart, margin_x=0.0, margin_y=0.0, safety_buffer=cart.specs.turn_radius + 0.2)

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
        elif joystick and (event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP):
            inputs.update(joystick_event_to_kv(event))

    # Poll joystick separately each frame
    if joystick:
        inputs.update(joystick_axes_to_kv(joystick))

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
    # drawer.draw_augmented_reality(observer, job_path, cart)

pygame.quit()
