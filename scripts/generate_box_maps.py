import os
import random
from PIL import Image, ImageDraw

output_folder = "/home/michael/github/rcd_path_planner/maps/random_boxes/"
num_images = 100
image_size = (1000, 1000)
border_size = 10
min_box_size = 10
max_box_size = 100

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

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

for i in range(num_images):
    coverage = random.randint(20,70)
    img = generate_image(coverage)
    img.save(f"{output_folder}/{i}.png")

print("Images generated successfully.")
