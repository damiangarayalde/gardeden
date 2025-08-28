import pygame
import math
from sim_physics import Thing
from draw import CartDrawer
from path import get_job_path
from controller import pure_pursuit_control

# --- Configurations ---
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

cart = Thing(wheel_base=0.4, blade_diameter=BLADE_DIAMETER)
cart.set_world_space(WORLD_RECT)
cart.set_initial_pose(0, 0, 0)

drawer = CartDrawer((WIDTH, HEIGHT))
drawer.set_world_space(WORLD_RECT)
drawer.set_cart_size(CART_RECT, BLADE_POS, BLADE_DIAMETER)

job_path = get_job_path(
    WORLD_RECT, blade_ef_diam=BLADE_DIAMETER, overlap=0.2, turn_radius=0.2)

ANGLE_STEP = 20
SPEED_STEP = 0.002


def handle_cart_controls(keys, cart, angle_step, speed_step):
    """Update cart state based on keyboard input."""
    if keys[pygame.K_a]:
        cart.angle -= angle_step
    if keys[pygame.K_d]:
        cart.angle += angle_step
    if keys[pygame.K_w]:
        cart.speed += speed_step
    if keys[pygame.K_s]:
        cart.speed -= speed_step


running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pose = cart.get_pose()
    path_xy = [(x, y) for x, y, _ in job_path]
    wheel_angle_rad = pure_pursuit_control(
        pose[0], pose[1], math.radians(pose[2]), path_xy, lookahead=0.05, wheelbase=cart.wheel_base)
    cart.wheel_angle = max(-45, min(45, math.degrees(wheel_angle_rad)))

    keys = pygame.key.get_pressed()

    handle_cart_controls(keys, cart, ANGLE_STEP, SPEED_STEP)

    cart.update_pose(dt=1.0)

    drawer.draw_scene(
        screen,
        job_path,
        cart
    )
    pygame.display.flip()

pygame.quit()
