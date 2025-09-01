from sim_physics import Cart, ControlInput
import pygame
# import math

from draw import Drawer
from path import get_job_path

FPS = 30


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


pygame.init()
clock = pygame.time.Clock()

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

    # Predict next pose (or replace with observer input in real system)
    measured_cart_pose = cart.estimate_next_pose(dt=1.0)

    # Update cart state (handles manual + tracking)
    cart.run_controller(user_input, measured_cart_pose)

    # Draw scene
    drawer.draw_scene(job_path, cart)

pygame.quit()
