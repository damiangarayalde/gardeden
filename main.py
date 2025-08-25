import pygame
from sim_physics import Thing
from draw import CartDrawer
from path import get_job_path

# --- Configurations ---
WIDTH, HEIGHT = 800, 600
FPS = 60

# World and cart setup
WORLD_RECT = [0, 0, 400, 300]  # Example world rectangle (local coordinates)
CART_RECT = [0, 0, 40, 40]     # Example cart bounding box (local coordinates)
BLADE_POS = (0, 0)             # Blade at cart center
BLADE_DIAMETER = 20            # Blade diameter in cm

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cart Simulation")
clock = pygame.time.Clock()

# Initialize simulation objects
cart = Thing(wheel_base=40)
cart.set_world_space(WORLD_RECT)
cart.set_initial_pose(0, 0, 0)  # Start near center, facing up

drawer = CartDrawer((WIDTH, HEIGHT))
drawer.set_world_space(WORLD_RECT)
drawer.set_cart_size(CART_RECT, BLADE_POS, BLADE_DIAMETER)

# Get job path (list of (x, y, angle) tuples in world coordinates)
job_path = get_job_path(
    WORLD_RECT, blade_ef_diam=BLADE_DIAMETER, overlap=0.2, turn_radius=0.5)

# Store walked path in real world coordinates
walked_path = []

# Control sensitivity
ANGLE_STEP = 2      # degrees per key press
SPEED_STEP = 0.2    # units per key press

running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Key states
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        cart.wheel_angle -= ANGLE_STEP
    if keys[pygame.K_d]:
        cart.wheel_angle += ANGLE_STEP
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
