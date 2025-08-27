import pygame
import math
from sim_physics import Thing
from draw import CartDrawer
from path import get_job_path
from controller import pure_pursuit_control

# --- Configurations ---
WIDTH, HEIGHT = 800, 600
FPS = 60

# World and cart setup
WORLD_RECT = [0, 0, 9, 4]  # Example world rectangle (local coordinates)
CART_RECT = [0, 0, 0.5, 0.4]
BLADE_POS = (0, 0)
BLADE_DIAMETER = 0.3  # Blade diameter in meters

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cart Simulation")
clock = pygame.time.Clock()

# Initialize simulation objects
cart = Thing(wheel_base=0.4)
cart.set_world_space(WORLD_RECT)
cart.set_initial_pose(0, 0, 0)  # Start near center, facing up

drawer = CartDrawer((WIDTH, HEIGHT))
drawer.set_world_space(WORLD_RECT)
drawer.set_cart_size(CART_RECT, BLADE_POS, BLADE_DIAMETER)

# Get job path (list of (x, y, angle) tuples in world coordinates)
job_path = get_job_path(
    WORLD_RECT, blade_ef_diam=BLADE_DIAMETER, overlap=0.2, turn_radius=0.2)

# Store walked path in real world coordinates
walked_path = []

# Control sensitivity
ANGLE_STEP = 2      # degrees per key press (for perturbation)
SPEED_STEP = 0.002  # units per key press

running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Key states
    keys = pygame.key.get_pressed()
    # Use pure pursuit controller for wheel angle setpoint
    # Find closest point on job_path for control
    cart_pose = cart.get_pose()
    cart_x, cart_y, cart_theta_deg = cart_pose
    cart_theta_rad = math.radians(cart_theta_deg)
    # Use only (x, y) for path
    path_xy = [(x, y) for x, y, _ in job_path]
    wheel_angle_rad = pure_pursuit_control(
        cart_x, cart_y, cart_theta_rad, path_xy, lookahead=0.5, wheelbase=cart.wheel_base)
    cart.wheel_angle = math.degrees(wheel_angle_rad)

    # a/d keys act as perturbance to cart angle pose (not wheel angle setpoint)
    if keys[pygame.K_a]:
        cart.angle -= ANGLE_STEP
    if keys[pygame.K_d]:
        cart.angle += ANGLE_STEP
    if keys[pygame.K_w]:
        cart.speed += SPEED_STEP
    if keys[pygame.K_s]:
        cart.speed -= SPEED_STEP

    # Limit wheel angle to reasonable values
    cart.wheel_angle = max(-45, min(45, cart.wheel_angle))

    # Update cart position and heading
    cart.update_pose(dt=1.0)

    # Record walked path (cart center) in real world coordinates
    walked_path.append(cart.get_pose()[:2])

    # Draw
    screen.fill((240, 240, 240))

    # Draw job path using drawer method
    drawer.draw_job_path(screen, job_path)

    # Draw walked path and blade area using drawer method
    drawer.draw_walked_path(screen, walked_path, blade_diameter=BLADE_DIAMETER)

    # Draw cart
    drawer.draw_cart(screen, cart.get_pose(), cart.wheel_angle)
    pygame.display.flip()

pygame.quit()
