import pygame
import math

from sim_physics import Thing
from draw import Drawer
from path import get_job_path

FPS = 30


pygame.init()
clock = pygame.time.Clock()

cart = Thing("cart.yml")
drawer = Drawer("graphics.yml", cart)

job_path = get_job_path(cart)
cart.set_job_path(job_path)

running = True

while running:

    clock.tick(FPS)  # running for real this goes away

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    measured_cart_pose = cart.estimate_next_pose(dt=1.0)
    # measured_cart_pose = cart.observer.retrieve_cart_pose()     # Use real world metrology with

    # update cart model pose based on observer data
    cart.set_pose(measured_cart_pose)

    cart.user_ui.check_for_inputs(pygame, cart)

    cart.run_controller()

    drawer.draw_scene(
        job_path,
        cart
    )

pygame.quit()
