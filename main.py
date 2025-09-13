import socket
import json
from sim_physics import Cart, ControlInput, AnalogInput
import pygame
from draw import Drawer
from path import get_job_path


UDP_IP = "0.0.0.0"       # bind to all interfaces
UDP_PORT = 4210           # local port (optional, can be 0 for auto)

ESP32_IP = "192.168.0.16"  # ESP32 IP on WiFi
ESP32_PORT = 4210

FPS = 30


def get_user_input(key) -> ControlInput | None:
    """Map pressed keys to cart control inputs."""
    if key == pygame.K_a:          # spin left
        return ControlInput.TURN_LEFT
    elif key == pygame.K_d:        # spin right
        return ControlInput.TURN_RIGHT
    elif key == pygame.K_z:
        return ControlInput.DISTURB_LEFT
    elif key == pygame.K_c:
        return ControlInput.DISTURB_RIGHT
    elif key == pygame.K_w:
        return ControlInput.FASTER
    elif key == pygame.K_s:
        return ControlInput.SLOWER
    elif key == pygame.K_p:
        return ControlInput.TOGGLE_PAUSE
    elif key == pygame.K_t:
        return ControlInput.TOGGLE_TRACKING
    else:
        return None


def get_joystick_input(joystick) -> ControlInput | None:
    """
    Map joystick buttons/axes to ControlInput.
    Adjust mapping for Kolke KGJ-182 if needed.
    """
    # Axes usually return values between -1.0 and 1.0
    axis_x = joystick.get_axis(0)  # left stick vertical
    axis_y = joystick.get_axis(3)  # right stick vertical
    # axis_x = joystick.get_axis(0)  # left stick horizontal
    # axis_y = joystick.get_axis(1)  # left stick vertical

    # for debugging: print all axes and buttons
    # print("Axes:", [f"{joystick.get_axis(i):.2f}" for i in range(
    #     joystick.get_numaxes())])

    # print("Buttons:", [joystick.get_button(i)
    #   for i in range(joystick.get_numbuttons())])

    # Deadzone
    deadzone = 0.2
    if abs(axis_x) < deadzone:
        axis_x = 0.0
    if abs(axis_y) < deadzone:
        axis_y = 0.0

    # Convert to analog control
    # Y controls speed: up = forward, down = backward
    speed_percent = -axis_y  # invert so up is +speed
    # X controls turning rate
    max_omega = 0.04  # 2.0  # rad/s, tune as needed
    omega = axis_x * max_omega

    if speed_percent != 0.0 or omega != 0.0:
        return AnalogInput(speed_percent, omega)

    # Buttons for discrete commands
    if joystick.get_button(5):  # center start button
        return ControlInput.TOGGLE_PAUSE
    elif joystick.get_button(6):  # options button
        return ControlInput.TOGGLE_TRACKING
    elif joystick.get_button(10):  # L1
        return ControlInput.DISTURB_LEFT
    elif joystick.get_button(9):  # R1
        return ControlInput.DISTURB_RIGHT

    return None


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def get_joystick_raw_input(joystick):
    """
    Map joystick buttons/axes to ControlInput.
    Adjust mapping for Kolke KGJ-182 if needed.
    """
    # Axes usually return values between -1.0 and 1.0
    axis_x = joystick.get_axis(0)  # left stick vertical
    axis_y = joystick.get_axis(3)  # right stick vertical

    if joystick.get_button(5):  # center start button
        return axis_x, axis_y, True
    else:
        return axis_x, axis_y, False


def send_controls(controls: dict):
    """
    Send arbitrary controls as JSON.
    Example: {"x": 120, "y": -200, "btnA": 1, "btnB": 0}
    """
    msg = json.dumps(controls)
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    print(f"Sent: {msg}")


def send_joystick(x_float: float, y_float: float, btn: bool):
    """
    Send joystick values to ESP32.
    x_float, y_float: floats in range [-1.0, 1.0]
    btn: boolean
    """

    # Clamp & scale floats into -255 to +255 ints
    x = max(-1.0, min(1.0, x_float))
    y = max(-1.0, min(1.0, y_float))
    x_int = int(x * 255)
    y_int = int(y * 255)

    # Cast button to 1 or 0
    btn_int = 1 if btn else 0
    msg = {"x": x_int, "y": y_int, "btnA": btn_int, "btnB": 0}
    send_controls(msg)

    print(f"Sent: {msg}")


pygame.init()
clock = pygame.time.Clock()

# --- Initialize joystick(s) ---
pygame.joystick.init()
joystick = None

# -- udp shit
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on port {UDP_PORT}...")
sock.settimeout(0.5)  # 1 second timeout

# -- end udp shit


if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")
else:
    print("⚠️ No joystick detected. Keyboard only.")


cart = Cart("cart.yml")
drawer = Drawer("graphics.yml", cart)

job_path = get_job_path(cart)
cart.set_job_path(job_path)

running = True

while running:
    clock.tick(FPS)

    user_input = None
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            user_input = get_user_input(event.key)

    # Poll joystick separately each frame
    if joystick:
        x, y, btn = get_joystick_raw_input(joystick)
        send_joystick(x, y, btn)
        j_input = get_joystick_input(joystick)
        if j_input is not None:
            user_input = j_input

    # Predict next pose (or replace with observer input in real system)
    measured_cart_pose = cart.estimate_next_pose(dt=1.0)

    # Update cart state (handles manual + tracking)
    cart.run_controller(user_input, measured_cart_pose)

    # Draw scene
    drawer.draw_scene(job_path, cart)

    try:
        data, addr = sock.recvfrom(1024)  # will wait max 1 sec
        msg = data.decode().strip()
        controls = json.loads(msg)  # parse JSON
        print("Received from ESP32:", controls)

        # Extract like the old CSV format
        x = int(controls.get("x", 0))
        y = int(controls.get("y", 0))
        btn = bool(int(controls.get("btn", 0)))

        print(f"Joystick X={x}, Y={y}, Button={btn}")
    except socket.timeout:
        # no packet received in 1 second
        print("No packet received...")
        continue
    except Exception as e:
        print("Invalid packet:", msg, "Error:", e)

pygame.quit()

# desde la app tenemos que pasar info de los controles recibidos unicamente, por ej si se apreto p, t, w, s, a, d, z, c o si se movio el joystick +
# info de la posicion del joystick
# el mappeo de comandos a acciones del cart es una libreria del cart
