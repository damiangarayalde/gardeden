import pygame
import math
from sim_physics import Thing
from draw import CartDrawer
from path import get_job_path

WIDTH, HEIGHT = 800, 600
FPS = 60
WORLD_RECT = [0, 0, 9, 4]
CART_RECT = [0, 0, 0.5, 0.4]
BLADE_POS = (0, 0)
BLADE_DIAMETER = 0.3  # meters

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cart Simulation")
clock = pygame.time.Clock()

cart = Thing(wheel_base=0.4, blade_diameter=BLADE_DIAMETER,
             angle_step=20, speed_step=0.002)
cart.set_world_space(WORLD_RECT)
cart.set_initial_pose(0, 0, 0)

drawer = CartDrawer((WIDTH, HEIGHT))
drawer.set_world_space(WORLD_RECT)
drawer.set_cart_size(CART_RECT, BLADE_POS, BLADE_DIAMETER)

job_path = get_job_path(
    WORLD_RECT, blade_ef_diam=BLADE_DIAMETER, overlap=0.2, turn_radius=0.2)

running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    path_xy = [(x, y) for x, y, _ in job_path]
    wheel_angle_rad = cart.closed_loop_controller.pure_pursuit(path_xy)
    cart.wheel_angle = max(-45, min(45, math.degrees(wheel_angle_rad)))

    cart.user_controller.update(pygame, cart)
    cart.update_pose(dt=1.0)

    drawer.draw_scene(
        screen,
        job_path,
        cart
    )
    pygame.display.flip()

pygame.quit()
