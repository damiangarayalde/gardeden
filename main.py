from sim_physics import Cart, ControlInput
import pygame
from draw import Drawer
from path import get_job_path

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
    print("Axes:", [joystick.get_axis(i)
          for i in range(joystick.get_numaxes())])
    # print("Buttons:", [joystick.get_button(i)
    #   for i in range(joystick.get_numbuttons())])

    # Deadzone handling (ignore small noise)
    deadzone = 0.2
    if axis_x < -deadzone:
        return ControlInput.TURN_LEFT
    elif axis_x > deadzone:
        return ControlInput.TURN_RIGHT
    elif axis_y < -deadzone:
        return ControlInput.FASTER
    elif axis_y > deadzone:
        return ControlInput.SLOWER

    # Button mapping (check with print debug)
    if joystick.get_button(5):  # center start button
        return ControlInput.TOGGLE_PAUSE
    elif joystick.get_button(6):  # options button
        print("Toggle Tracking")
        return ControlInput.TOGGLE_TRACKING
    elif joystick.get_button(10):  # L1
        return ControlInput.DISTURB_LEFT
    elif joystick.get_button(9):  # R1
        return ControlInput.DISTURB_RIGHT

    return None


pygame.init()
clock = pygame.time.Clock()

# --- Initialize joystick(s) ---
pygame.joystick.init()
joystick = None


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
        j_input = get_joystick_input(joystick)
        if j_input is not None:
            user_input = j_input

    # Predict next pose (or replace with observer input in real system)
    measured_cart_pose = cart.estimate_next_pose(dt=1.0)

    # Update cart state (handles manual + tracking)
    cart.run_controller(user_input, measured_cart_pose)

    # Draw scene
    drawer.draw_scene(job_path, cart)

pygame.quit()
