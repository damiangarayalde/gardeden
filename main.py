import pygame
import math

from sim_physics import Thing
from draw import Drawer
from path import get_job_path

FPS = 30


def get_user_inputs(key):

    if key == pygame.K_a:  # spin left (just once per press)
        input = "left"
    elif key == pygame.K_d:  # spin right
        input = "right"
    elif key == pygame.K_z:
        input = "disturb_left"
    elif key == pygame.K_c:
        input = "disturb_right"
    elif key == pygame.K_w:
        input = "move_faster"
    elif key == pygame.K_s:
        input = "move_slower"
    elif key == pygame.K_p:
        input = "toggle_pause"
    elif key == pygame.K_t:
        input = "toggle_tracking"
    return input


pygame.init()
clock = pygame.time.Clock()

cart = Thing("cart.yml")
drawer = Drawer("graphics.yml", cart)

job_path = get_job_path(cart)
cart.set_job_path(job_path)

user_inputs = ""
running = True

while running:

    clock.tick(FPS)  # running for real this goes away

    user_inputs = ""

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            user_inputs = get_user_inputs(event.key)

    measured_cart_pose = cart.estimate_next_pose(dt=1.0)
    # measured_cart_pose = cart.observer.retrieve_cart_pose()     # Use real world metrology with

    cart.run_controller(user_inputs,  measured_cart_pose)

    drawer.draw_scene(
        job_path,
        cart
    )

pygame.quit()
