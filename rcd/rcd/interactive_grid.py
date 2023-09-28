from rcd.setup import *
from rcd.utilities import print_red 


class InteractiveGridGenerator:
    # Initialization of parameters
    file_path = "../config/grid_params.yaml"
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
    
    
    
    def __init__(self):
        self.load_grid_params()
        self.grid_size = [int(self.workspace_size[0]/self.grid_resolution) , int(self.workspace_size[1]/self.grid_resolution)]
        wall_size = int(math.ceil((self.robot_size)/self.grid_resolution))

        # Initialize Occupancy grid
        self.grid = np.zeros(self.grid_size)

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
        
        
    # Loads config Parameters from the .yaml file 
    def load_grid_params(self):
        # Read data from the YAML file
        with open(self.file_path, "r") as file:
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
        plt.title('Draw Occupancy Grid')
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

# offline_experiments = False

# # Hyperparams
# real_time_plotting = False
# draw_edge_split    = False
# random_source_dir  = True

# robot_size = 1 # (m) Robot's diameter
# grid_resolution = 0.1 # (m)
# workspace_size = (50,30) # (m) Size of the workspace/room where the robot needs to navigate
# num_beams = 8
# drawing_brush_size = 10#int(3/grid_resolution)  # Size of the brush (in grid cells)




# # Parameters
# grid_size = (int(workspace_size[0]/grid_resolution), int(workspace_size[1]/grid_resolution))     # Size of the occupancy grid in meters (rows, columns)
# wall_size = int(math.ceil((robot_size)/grid_resolution))

# # Initialize Global variables
# target_pos     = [0,0]
# robot_pos      = [0,0]
# visited_robot  = np.empty(0,dtype=object)
# visited_target = np.empty(0,dtype=object)
# coverage = 0


# grid         = np.zeros(grid_size)
# grid_edge_id = np.empty(grid_size,dtype=object) # Contains the edge id that passes through each cell


# init_robot_pos= False
# init_target_pos= False
# path_found = False

# robot_graph  = nx.Graph()
# target_graph = nx.Graph() 




# # Objects
# class Point:
#     def __init__(self,x,y):
#         self.x = x
#         self.y = y  

# class Edge:
#     def __init__(self,edge_id,start_node,end_node):
#         self.edge_id    = edge_id # Unique String for specifing which edge is this
#         self.start_node = start_node
#         self.end_node   = end_node




# # Adds obstacles by drawing at the plot
# def draw_obstacles(grid, position, drawing_brush_size):
#     cell_position = (round(position[0]),round(position[1]))
#     half_brush = drawing_brush_size // 2
#     for i in range(-half_brush, half_brush + 1):
#         for j in range(-half_brush, half_brush + 1):
#             x = int(cell_position[0] + i)
#             y = int(cell_position[1] + j)
#             if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
#                 grid[x, y] = 100

# '''
# Utility functions for drawing 
# ----------------------------------------------------------------------
# '''




# '''
# Initializes the occupancy grid
# '''
# def init_grid():
#     # Init empty occ grid
#     global grid, grid_edge_id, wall_size

#     # Every element in the grid_edge_id array is a Null object
#     for i in range(grid_edge_id.shape[0]):
#         for j in range(grid_edge_id.shape[1]):
#             grid_edge_id[i,j] = Edge(None,None,None)

#     if not offline_experiments:
#         # Add walls to the boundary of the grid
#         grid[0:wall_size, :] = 100
#         grid[-wall_size:, :] = 100
#         grid[:, 0:wall_size] = 100
#         grid[:, -wall_size:] = 100
    
#     return


# def on_press(event):
#     global drawing
#     drawing = True
#     if (event.xdata is not None and event.ydata is not None):
#         draw_obstacles(grid, (event.xdata, event.ydata), drawing_brush_size)
#         im.set_data(grid.T)
#         plt.draw()


# def on_release(event):
#     global drawing
#     drawing = False

# def on_motion(event):
#     if (drawing and event.xdata is not None and event.ydata is not None):
#         draw_obstacles(grid, (event.xdata, event.ydata), drawing_brush_size)
#         im.set_data(grid.T)
#         plt.draw()


# def set_goal_robot(event):
#     global init_robot_pos  # Declare init_robot_pos as global
#     global init_target_pos
#     global robot_pos,target_pos
#     if event.xdata is not None and event.ydata is not None:
#         if not init_target_pos and init_robot_pos:
#             target_pos = [round(event.xdata),round(event.ydata)] # [43,70] ICRA
#             init_target_pos = True
#             plt.scatter([target_pos[0]], [target_pos[1]], color='green', marker='o', s=80, label='Target', zorder=2)
#         if not init_robot_pos:
#             robot_pos = [round(event.xdata),round(event.ydata)] # ICRA [43,230]
#             init_robot_pos = True
#             plt.scatter([robot_pos[0]], [robot_pos[1]], color='blue', marker='o', s=80, label='Robot', zorder=2)

#         plt.draw()



# def draw_grid():
#     global im
#     plt.figure(figsize=(workspace_size[0], workspace_size[1]))

#     im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
#     plt.gca().invert_yaxis()
#     plt.title('Ray Casting and Diffusion Model')
#     plt.axis('off')  # Turn on axis for grid lines

#     # # Connect the mouse events
#     press   = plt.connect('button_press_event', on_press)
#     release = plt.connect('button_release_event', on_release)
#     motion  = plt.connect('motion_notify_event', on_motion)
#     goal    = plt.connect('key_press_event', set_goal_robot)

#     plt.show(block = False)

#     print("Draw and place goal and robot")
#     input("Press Enter when setup is completed...")


#     plt.disconnect(press)
#     plt.disconnect(release)
#     plt.disconnect(motion)
#     plt.disconnect(goal)
#     return
# #----------------------------------------------------------------------




# '''
# Uses the convolve2d to inflate the occupancy grid by robot_size//2
# '''
# def inflate_occupancy_grid(robot_size):
#     global grid, grid_resolution
#     inflated_grid = np.copy(grid)

#     # Create a binary mask of occupied cells
#     occupied_mask = (grid == 100).astype(np.int32)

#     # Create a circular kernel for inflation
#     kernel_size = int(robot_size / grid_resolution)
#     kernel = np.zeros((kernel_size * 2 + 1, kernel_size * 2 + 1), dtype=np.int32)
#     y, x = np.ogrid[-kernel_size:kernel_size + 1, -kernel_size:kernel_size + 1]
#     mask = x**2 + y**2 <= robot_size**2
#     kernel[mask] = 1

#     # Use convolution to inflate the occupied cells
#     inflated_occupied = convolve2d(occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)
    
#     # Update the inflated grid
#     inflated_grid[inflated_occupied > 0] = 100

#     if not np.allclose(inflated_grid, grid):
#         grid = inflated_grid

