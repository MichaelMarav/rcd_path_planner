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

# Function to generate random positions
def generate_random_positions(image):
    width, height = image.size
    # Access pixel values using getpixel() method
    free_space = [(x, y) for x in range(width) for y in range(height) if image.getpixel((y, x)) == 255]
    
   
    robot_position = random.choice(free_space)
    free_space.remove(robot_position)
    target_position = random.choice(free_space)
    return robot_position, target_position

# Function to save YAML file
def save_yaml(filename, robot_position, target_position,coverage):
    with open(filename,'w') as file:
            file.write("robot_position_x: " +  str((int)(robot_position[1]) ) + '\n')
            file.write("robot_position_y: " +  str((int)(robot_position[0])) + '\n')
            file.write("target_position_x: " + str((int)(target_position[1])) + '\n')
            file.write("target_position_y: " + str((int)(target_position[0])) + '\n')
            file.write("coverage: "+str(coverage) + '\n')         
            
            
            
for i in range(num_images):
    coverage = random.randint(20,70)
    img = generate_image(coverage)
    robot_position, target_position = generate_random_positions(img)

    img.save(f"{output_folder}/{i}.png")
    yaml_filename = f"{output_folder}/{i}.yaml"
    save_yaml(yaml_filename, robot_position, target_position,coverage)


    

print("Images generated successfully.")
