
# Imported libraries for every submodule
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
matplotlib.rcParams['font.size'] = 16  # Choose your desired font size

import matplotlib.pyplot as plt
import math
from scipy.signal import convolve2d
import datetime
import yaml
import os
from PIL import Image
import subprocess


class InteractiveGridGenerator:
    # Initialization of parameters
    grid_params_path = "../config/interactive_grid_params.yaml"
    workspace_size     = [None,None]
    grid_resolution    = None
    drawing_brush_size = None
    robot_size = None
    occupancy_grid_filename = None
    grid = np.zeros(())
    robot_pos   = None
    target_pos  = None
    grid_size   = None
    # Private Params 
    __im = 0           # Image object
    __drawing         = False
    __init_robot_pos  = False
    __init_target_pos = False
    
    
    
    def __init__(self):
        self.load_interactive_plot_params()
        self.grid_size = [int(self.workspace_size[0]/self.grid_resolution) , int(self.workspace_size[1]/self.grid_resolution)]
        wall_size = int(math.ceil((self.robot_size)/self.grid_resolution))
        self.user_input_mode()
        # Initialize Occupancy grid
        self.grid = np.zeros(self.grid_size)

        # Add walls
        self.add_walls(wall_size)
    
        # Draw the occupancy grid 
        self.draw_grid()
        
        while self.robot_pos == None or self.target_pos == None:
            self.print_red("You need to enter both robot and target positions in order to continue. Point the cursor in the occupancy grid and press any key. One for the robot and one more for the target")
            plt.close('all')
            self.__init_target_pos = False
            self.__init_robot_pos  = False
            self.robot_pos   = None
            self.target_pos = None
            
            self.draw_grid()
            
            
        # Inflate the occupancy grid
        self.inflate_occupancy_grid()
        
        inverted_array = 100 - self.grid.T[::-1]
        # Save the 2D array as png
        normalized_array = (inverted_array / 100) * 255
        
        os.makedirs("../maps/" + self.occupancy_grid_filename, exist_ok=True)

        # Configure map's filename
        file_path = "../maps/"+ self.occupancy_grid_filename +"/"+ self.occupancy_grid_filename+ ".png"


        # Convert the array to uint8 for PIL compatibility
        image_data = np.uint8(normalized_array)
        # Create an image from the array data
        image = Image.fromarray(image_data, mode='L')  # 'L' mode for grayscale
        # Save the image as PNG
        image.save(file_path)
        self.print_green("Map saved as " + self.occupancy_grid_filename + ".png" + " at maps/ folder")
            
        command2 = "cd ../maps/" + self.occupancy_grid_filename + "&& mogrify -format ppm *.png"

        # Execute the shell command
        subprocess.run(command2, shell=True) 
                    
        # TODO: add the txt + yaml params inside a file in the folder.
        with open(f"/home/michael/github/rcd_path_planner/maps/" + self.occupancy_grid_filename + "/" + self.occupancy_grid_filename + ".yaml",'w') as file:
            file.write("robot_position_x: " +  str((int)(self.robot_pos[0]) ) + '\n')
            file.write("robot_position_y: " +  str((int)(self.grid_size[1] - self.robot_pos[1]) ) + '\n')
            file.write("target_position_x: " + str((int)(self.target_pos[0])) + '\n')
            file.write("target_position_y: " + str((int)(self.grid_size[1] - self.target_pos[1])) + '\n')
            file.write("grid_resolution: "+str(self.grid_resolution) + '\n')         
        print("Exiting..")
        
    def print_green(self,text):
        print("\033[32m" + text)
        print("\033[0m")
        
        
        
    def print_red(self,text):
        print("\033[31m" + text)
        print("\033[0m")
        

    def user_input_mode(self):
        
        print(" \n\n -- \033[31mR\033[0may \033[31mC\033[0masting and \033[31mD\033[0miffusion Path Planning Algorithm (RCD) --\n\n")            
        print("You can set the grid configuration parameters in config/grid_params.yaml")
        print("Settings: \n * An interactive window will appear with the boarders predifined as walls._")
        print(" * Press left click anywhere inside the window to start drawing the occupied space") 
        print(" * Press any key while pointing with cursor somewhere in the grid and the robot will be placed at that position")
        print(" * After the robot (Blue point) is placed you can press any key and the target will be placed at the place that the cursor is")
        print(" * After you place the target the GUI exits and the occupancy grid will be saved at: \033[32m maps/ \033[0m")

        input("\nPress Enter to continue to continue")
        return

    # Loads config Parameters from the .yaml file 
    def load_interactive_plot_params(self):
        # Read data from the YAML file
        with open(self.grid_params_path, "r") as file:
            config_options = yaml.load(file, Loader=yaml.FullLoader)
        
        self.workspace_size     = config_options["workspace_size"]
        self.grid_resolution    = config_options["grid_resolution"]
        self.drawing_brush_size = config_options["drawing_brush_size"]
        self.drawing_brush_size = int(self.drawing_brush_size/self.grid_resolution)
        self.robot_size         = config_options["robot_size"]
        self.occupancy_grid_filename= str(config_options["occupancy_grid_filename"])

        
    # Adds obstacles by drawing at the plot
    def draw_obstacles(self, grid, position, drawing_brush_size):
        cell_position = (round(position[0]),round(position[1]))
        half_brush = drawing_brush_size // 2
        for i in range(-half_brush, half_brush + 1):
            for j in range(-half_brush, half_brush + 1):
                x = int(cell_position[0] + i)
                y = int(cell_position[1] + j)
                if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                    grid[x, y] = 100


        
        
    # Adds Walls to the occupancy grid boundaries
    def add_walls(self,wall_size):
        self.grid[0:wall_size, :] = 100
        self.grid[-wall_size:, :] = 100
        self.grid[:, 0:wall_size] = 100
        self.grid[:, -wall_size:] = 100        
        return
    

    def on_press(self,event):
        self.__drawing = True
        if (event.xdata is not None and event.ydata is not None):
            self.draw_obstacles(self.grid, (event.xdata, event.ydata), self.drawing_brush_size)
            self.__im.set_data(self.grid.T)
            plt.draw()


    def on_release(self,event):
        self.__drawing = False

    def on_motion(self,event):
        if (self.__drawing and event.xdata is not None and event.ydata is not None):
            self.draw_obstacles(self.grid, (event.xdata, event.ydata), self.drawing_brush_size)
            self.__im.set_data(self.grid.T)
            plt.draw()


    def set_goal_robot(self,event):

        if event.xdata is not None and event.ydata is not None:
           
            if not self.__init_target_pos and self.__init_robot_pos:
                self.target_pos = [round(event.xdata),round(event.ydata)] 
                self.__init_target_pos = True
                plt.scatter([self.target_pos[0]], [self.target_pos[1]], color='green', marker='o', s=80, label='Target', zorder=2)
            
            if not self.__init_robot_pos:
                self.robot_pos = [round(event.xdata),round(event.ydata)] 
                self.__init_robot_pos = True
                plt.scatter([self.robot_pos[0]], [self.robot_pos[1]], color='blue', marker='o', s=80, label='Robot', zorder=2)

            plt.draw()
            
            
            
    def draw_grid(self):
        plt.figure(figsize=(self.workspace_size[0], self.workspace_size[1]))

        self.__im = plt.imshow(self.grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
        plt.gca().invert_yaxis()
        plt.axis('off')  # Turn on axis for grid lines

        # # Connect the mouse events
        press   = plt.connect('button_press_event', self.on_press)
        release = plt.connect('button_release_event', self.on_release)
        motion  = plt.connect('motion_notify_event', self.on_motion)
        goal    = plt.connect('key_press_event', self.set_goal_robot)

        plt.show(block = False)

        print("Draw and place goal and robot")
        input("Press Enter when setup is completed...")


        plt.disconnect(press)
        plt.disconnect(release)
        plt.disconnect(motion)
        plt.disconnect(goal)
        return
    

    # Uses the convolve2d to inflate the occupancy grid by robot_size//2
    def inflate_occupancy_grid(self):
        inflated_grid = np.copy(self.grid)

        # Create a binary mask of occupied cells
        occupied_mask = (self.grid == 100).astype(np.int32)

        # Create a circular kernel for inflation
        kernel_size = int(self.robot_size / self.grid_resolution)
        kernel = np.zeros((kernel_size * 2 + 1, kernel_size * 2 + 1), dtype=np.int32)
        y, x = np.ogrid[-kernel_size:kernel_size + 1, -kernel_size:kernel_size + 1]
        mask = x**2 + y**2 <= self.robot_size**2
        kernel[mask] = 1

        # Use convolution to inflate the occupied cells
        inflated_occupied = convolve2d(occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)
        
        # Update the inflated grid
        inflated_grid[inflated_occupied > 0] = 100

        if not np.allclose(inflated_grid, self.grid):
            self.grid = inflated_grid


if __name__ == "__main__":
    grid_plotter = InteractiveGridGenerator()
