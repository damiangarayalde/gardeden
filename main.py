import pygame
import math
import yaml
from sim_physics import Thing
from draw import CartDrawer
from path import get_job_path

# Load config from YAML
with open("config.yml", "r") as f:
    config = yaml.safe_load(f)

WORLD_RECT = config["world"]["rect"]
WIDTH = config["world"]["width"]
HEIGHT = config["world"]["height"]
FPS = config["world"]["fps"]

CART_RECT = config["cart"]["rect"]
BLADE_POS = tuple(config["cart"]["blade_pos"])
BLADE_DIAMETER = config["cart"]["blade_diameter"]
WHEEL_BASE = config["cart"]["wheel_base"]
ANGLE_STEP = config["cart"]["angle_step"]
SPEED_STEP = config["cart"]["speed_step"]
INITIAL_POSE = config["cart"]["initial_pose"]

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cart Simulation")
clock = pygame.time.Clock()

cart = Thing(
    wheel_base=WHEEL_BASE,
    blade_diameter=BLADE_DIAMETER,
    angle_step=ANGLE_STEP,
    speed_step=SPEED_STEP
)
cart.set_world_space(WORLD_RECT)
cart.set_initial_pose(*INITIAL_POSE)

drawer = CartDrawer((WIDTH, HEIGHT))
drawer.set_world_space(WORLD_RECT)
drawer.set_cart_size(CART_RECT, BLADE_POS, BLADE_DIAMETER)

job_path = get_job_path(
    WORLD_RECT, blade_ef_diam=BLADE_DIAMETER, overlap=0.2, turn_radius=0.2
)

running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    wheel_angle_rad = cart.closed_loop_controller.pure_pursuit(job_path)
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
