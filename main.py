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

    clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    cart.user_controller.update(pygame, cart)
    # future real world metrology with observer.retrieve_cart_pose()
    cart_pose = cart.get_pose()
    cart.set_pose(cart_pose)  # update cart pose based on observer data
    cart.run_controller()
    cart.estimate_next_pose(dt=1.0)

    drawer.draw_scene(
        job_path,
        cart
    )

pygame.quit()


##

# at high level, we need to control  speed of the cart
# we have to ask the observer to tell us the position of the cart
# we have to run the control loop step
