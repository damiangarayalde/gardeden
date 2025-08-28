import pygame
import math

from sim_physics import Thing
from draw import Drawer
from path import get_job_path

FPS = 30


pygame.init()
pygame.display.set_caption("Cart Simulation")
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

    wheel_angle_rad = cart.closed_loop_controller.pure_pursuit()
    cart.wheel_angle = max(-45, min(45, math.degrees(wheel_angle_rad)))

    cart.user_controller.update(pygame, cart)

    cart.update_pose(dt=1.0)

    drawer.draw_scene(
        job_path,
        cart
    )
    pygame.display.flip()

pygame.quit()
