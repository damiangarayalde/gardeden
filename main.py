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

    cart.run_controller_step()

    drawer.draw_scene(
        job_path,
        cart
    )

pygame.quit()
