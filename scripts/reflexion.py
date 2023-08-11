import pygame
import random
import math

# Grid parameters
GRID_SIZE = 10
CELL_SIZE = 50
GRID_WIDTH = GRID_SIZE * CELL_SIZE
GRID_HEIGHT = GRID_SIZE * CELL_SIZE

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Initialize Pygame
pygame.init()

# Create the display surface
screen = pygame.display.set_mode((GRID_WIDTH, GRID_HEIGHT))
pygame.display.set_caption("Robot Beam Wall Interaction Simulation")

# Create the grid (occupancy grid)
grid = [[1 if i == 0 or i == GRID_SIZE - 1 or j == 0 or j == GRID_SIZE - 1 else 0 for j in range(GRID_SIZE)] for i in range(GRID_SIZE)]

# Generate a random robot position
robot_x = random.randint(1, GRID_SIZE - 2)
robot_y = random.randint(1, GRID_SIZE - 2)

# Main loop
running = True
new_beam_generated = False
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)

    # Draw grid cells
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            color = BLACK if grid[i][j] == 1 else WHITE
            pygame.draw.rect(screen, color, pygame.Rect(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw robot
    pygame.draw.circle(screen, GREEN, (robot_x * CELL_SIZE + CELL_SIZE // 2, robot_y * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 3)

    # Simulate and draw beam
    if not new_beam_generated:
        angle = random.uniform(0, math.pi * 2)  # Random beam angle
        beam_x = robot_x * CELL_SIZE + CELL_SIZE // 2
        beam_y = robot_y * CELL_SIZE + CELL_SIZE // 2

        while 0 < beam_x < GRID_WIDTH and 0 < beam_y < GRID_HEIGHT:
            grid_x = int(beam_x // CELL_SIZE)
            grid_y = int(beam_y // CELL_SIZE)

            pygame.draw.circle(screen, RED, (int(beam_x), int(beam_y)), 2)

            if grid[grid_y][grid_x] == 1:  # Hit a wall
                angle = random.uniform(0, math.pi * 2)  # Generate a new beam in a random direction
                new_beam_x = beam_x
                new_beam_y = beam_y
                new_beam_generated = True
                break

            beam_x += math.cos(angle)
            beam_y += math.sin(angle)

    # Generate and draw a new beam
    if new_beam_generated:
        pygame.draw.circle(screen, RED, (int(new_beam_x), int(new_beam_y)), 2)

        new_beam_x += math.cos(angle)
        new_beam_y += math.sin(angle)

        if new_beam_x < 0 or new_beam_x > GRID_WIDTH or new_beam_y < 0 or new_beam_y > GRID_HEIGHT:
            new_beam_generated = False

    # Update the display
    pygame.display.flip()

    pygame.time.delay(50)  # Add a slight delay for visualization

# Quit Pygame
pygame.quit()
