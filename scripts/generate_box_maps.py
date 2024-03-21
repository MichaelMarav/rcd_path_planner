import os
import random
from PIL import Image, ImageDraw
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

output_folder = "/home/michael/github/rcd_path_planner/maps/random_boxes1/"
num_images = 100
image_size = (1000, 1000)
border_size = 10
min_box_size = 10
max_box_size = 100

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0

def generate_image(coverage):
    img = Image.new("L", image_size, color=255)  # White background
    draw = ImageDraw.Draw(img)

    # Add black border
    draw.rectangle([(0, 0), (image_size[0] - 1, border_size - 1)], fill=0)  # Top
    draw.rectangle([(0, 0), (border_size - 1, image_size[1] - 1)], fill=0)  # Left
    draw.rectangle([(0, image_size[1] - border_size), (image_size[0] - 1, image_size[1] - 1)], fill=0)  # Bottom
    draw.rectangle([(image_size[0] - border_size, 0), (image_size[0] - 1, image_size[1] - 1)], fill=0)  # Right

    total_area = (image_size[0] - 2 * border_size) * (image_size[1] - 2 * border_size)
    black_area = int(coverage / 100 * total_area)

    while black_area > 0:
        box_size = random.randint(min_box_size, max_box_size)
        x = random.randint(border_size, image_size[0] - border_size - box_size)
        y = random.randint(border_size, image_size[1] - border_size - box_size)
        draw.rectangle([(x, y), (x + box_size - 1, y + box_size - 1)], fill=0)
        black_area -= box_size * box_size

    return img

def generate_random_positions(image):
    width, height = image.size
    
    # Bottom left corner
    robot_position = Node(height - border_size - 1, border_size)

    # Top right corner
    target_position = Node(border_size, height - border_size - 1)

    grid = [[1 if image.getpixel((x, y)) == 255 else 0 for y in range(height)] for x in range(width)]
    grid = Grid(matrix=grid)

    finder = AStarFinder()

    path, _ = finder.find_path(grid.node(robot_position.x, robot_position.y),
                               grid.node(target_position.x, target_position.y),
                               grid)

    if path:
        return (robot_position.x, robot_position.y), (target_position.x, target_position.y)
    else:
        return generate_random_positions(image)

def save_yaml(filename, robot_position, target_position, coverage):
    with open(filename, 'w') as file:
        file.write("robot_position_x: " + str(int(robot_position[1])) + '\n')
        file.write("robot_position_y: " + str(int(robot_position[0])) + '\n')
        file.write("target_position_x: " + str(int(target_position[1])) + '\n')
        file.write("target_position_y: " + str(int(target_position[0])) + '\n')
        file.write("coverage: " + str(coverage) + '\n')


for i in range(num_images):
    coverage = random.randint(20, 70)
    img = generate_image(coverage)
    robot_position, target_position = generate_random_positions(img)

    img.save(f"{output_folder}/{i}.png")
    yaml_filename = f"{output_folder}/{i}.yaml"
    save_yaml(yaml_filename, robot_position, target_position, coverage)

print("Images generated successfully.")
