import os
import random
import cv2
import yaml

# Function to add border and scale image
def preprocess_image(image):
    bordered_image = cv2.copyMakeBorder(image, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
    scaled_image = cv2.resize(bordered_image, (1000, 1000))
    return scaled_image

# Function to generate random positions
def generate_random_positions(image):
    free_space = [(x, y) for x in range(image.shape[0]) for y in range(image.shape[1]) if image[x, y] == 255]
    robot_position = random.choice(free_space)
    free_space.remove(robot_position)
    target_position = random.choice(free_space)
    return robot_position, target_position

# Function to save YAML file
def save_yaml(filename, robot_position, target_position):
    with open(filename,'w') as file:
            file.write("robot_position_x: " +  str((int)(robot_position[1]) ) + '\n')
            file.write("robot_position_y: " +  str((int)(robot_position[0])) + '\n')
            file.write("target_position_x: " + str((int)(target_position[1])) + '\n')
            file.write("target_position_y: " + str((int)(target_position[0])) + '\n')
            # file.write("workspace_dimension_x: " + str(self.workspace_size[0]) + '\n')
            # file.write("workspace_dimension_y: " + str(self.workspace_size[1]) + '\n')
            file.write("grid_resolution: "+str(1) + '\n')         

# Main function
def main():
    # Loop through each image
    for i in range(100):
        filename ="/home/michael/github/rcd_path_planner/maps/mazes/" + f"{i}.png"
        image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"Could not read {filename}")
            continue

        # Preprocess image
        processed_image = preprocess_image(image)

        # Save scaled image
        scaled_filename = os.path.splitext(filename)[0] + "_scaled.png"
        cv2.imwrite(scaled_filename, processed_image)

        # Generate random positions
        robot_position, target_position = generate_random_positions(processed_image)

        # Save YAML file
        yaml_filename = os.path.splitext(filename)[0] + ".yaml"
        save_yaml(yaml_filename, robot_position, target_position)

if __name__ == "__main__":
    main()
