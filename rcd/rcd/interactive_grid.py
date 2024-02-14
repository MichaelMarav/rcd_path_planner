from rcd.setup import *
from rcd.utilities import print_red, print_green


class InteractiveGridGenerator:
    # Initialization of parameters
    interactive_plot_params = "../config/interactive_plotter_params.yaml"
    offline_map_filename = "../Data/"
    offline_params_filename = "../Data/"
    workspace_size     = [None,None]
    grid_resolution    = None
    drawing_brush_size = None
    robot_size = None
    grid = np.zeros(())
    robot_pos   = None
    target_pos  = None
    grid_size   = None
    # Private Params 
    __im = 0           # Image object
    __drawing         = False
    __init_robot_pos  = False
    __init_target_pos = False
    
    
    
    def __init__(self, offline_mode, prefix):
        # Offline mode
        if offline_mode:
            if prefix is not None:
                self.offline_params_filename =  f"../Data/{prefix}/{prefix}.yaml"
                self.offline_map_filename += prefix +"/"+prefix+".ppm"
            else:
                print_red("Error: Please enter a prefix. \nExiting..")
                sys.exit()


            # Load Params YAML file
            with open(self.offline_params_filename, "r") as file:
                config_options = yaml.load(file, Loader=yaml.FullLoader)
                
                self.workspace_size  = (config_options["workspace_dimension_x"], config_options["workspace_dimension_y"])
                self.grid_resolution = config_options["grid_resolution"]
                self.grid_size       = [round(self.workspace_size[0]/self.grid_resolution), round(self.workspace_size[1]/self.grid_resolution)]
                self.robot_size      = 1
                self.robot_pos       = [config_options["robot_position_x"] , config_options["robot_position_y"]]
                self.target_pos      = [config_options["target_position_x"] , config_options["target_position_y"]]

            # Load Map
            with open(self.offline_map_filename, 'rb') as f:
                self.grid = np.zeros(self.grid_size, dtype=np.uint8)

                # Read the pixel data
                pixels = np.fromfile(f, dtype=np.uint8, count=self.grid_size[0]*self.grid_size[1])

                # Reshape the pixel data into a 2D array 
                self.grid = np.reshape(pixels, (self.grid_size[1], self.grid_size[0]))

                # Convert white pixels to 0 and black pixels to 100
                self.grid = np.where(self.grid == 255, 0, 100)

           
            # Inflate the occupancy grid
            self.inflate_occupancy_grid()

        # Online mode
        else:
            self.load_interactive_plot_params()

            self.grid_size = [int(self.workspace_size[0]/self.grid_resolution) , int(self.workspace_size[1]/self.grid_resolution)]
            wall_size = int(math.ceil((self.robot_size)/self.grid_resolution))

            # Initialize Occupancy grid
            self.grid = np.zeros(self.grid_size,dtype=np.uint32)

            print("Draw the map and enter the position of robot and target")
            # Add walls
            self.add_walls(wall_size)
        
            # Draw the occupancy grid 
            self.draw_grid()
            
            while self.robot_pos == None or self.target_pos == None:
                print_red("You need to enter both robot and target positions in order to continue. Point the cursor in the occupancy grid and press any key. One for the robot and one more for the target")
                plt.close('all')
                self.__init_target_pos = False
                self.__init_robot_pos  = False
                self.robot_pos   = None
                self.target_pos = None
                
                self.draw_grid()    


            # Inflate the occupancy grid
            self.inflate_occupancy_grid()

            # Save Map option
            save_prefix = str(input("Enter prefix to save the map at Data/prefix/prefix.ppm or enter to continue\n")) 
            if (save_prefix is not None):
                os.makedirs(f"../Data/{save_prefix}", exist_ok=True)
                test = self.grid.T[::-1]
                image_data = np.where(test == 0, 255, 0).astype(np.uint8)

                image = Image.fromarray(image_data)

                image.save(f"../Data/{save_prefix}/{save_prefix}.ppm")
                # imageio.imwrite(f"../Data/{save_prefix}/{save_prefix}.ppm", test, format='PPM-PIL')
                # plt.savefig("../Data/" + save_prefix + "/"+ save_prefix  + ".ppm", format='ppm')#, bbox_inches='tight', pad_inches=0)
                print_green("Map saved at: " + f"Data/{save_prefix}/{save_prefix}.ppm")
                with open("../Data/"+save_prefix+"/"+save_prefix+".yaml",'w') as file:
                    file.write("robot_position_x: " +  str(self.robot_pos[0] ) + '\n')
                    file.write("robot_position_y: " +  str(self.robot_pos[1] ) + '\n')
                    file.write("target_position_x: " + str(self.target_pos[0]) + '\n')
                    file.write("target_position_y: " + str(self.target_pos[1]) + '\n')
                    file.write("workspace_dimension_x: " + str(self.workspace_size[0]) + '\n')
                    file.write("workspace_dimension_y: " + str(self.workspace_size[1]) + '\n')
                    file.write("grid_resolution: "+str(self.grid_resolution) + '\n')      
                
    # Loads config Parameters from the .yaml file 
    def load_interactive_plot_params(self):
        # Read data from the YAML file
        with open(self.interactive_plot_params, "r") as file:
            config_options = yaml.load(file, Loader=yaml.FullLoader)
        
        self.workspace_size     = config_options["workspace_size"]
        self.grid_resolution    = config_options["grid_resolution"]
        self.drawing_brush_size = config_options["drawing_brush_size"]
        self.drawing_brush_size = int(self.drawing_brush_size/self.grid_resolution)
        self.robot_size         = config_options["robot_size"]
        
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
                self.target_pos = [round(event.xdata),round(event.ydata)] # [43,70] ICRA
                self.__init_target_pos = True
                plt.scatter([self.target_pos[0]], [self.target_pos[1]], color='green', marker='o', s=80, label='Target', zorder=2)
            
            if not self.__init_robot_pos:
                self.robot_pos = [round(event.xdata),round(event.ydata)] # ICRA [43,230]
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
