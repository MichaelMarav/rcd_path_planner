from rcd.setup import *


class interactive_grid_generator:
    file_path = "../../config/grid_params.yaml"
    
    def __init__(self):
        # Read data from the YAML file
        with open(self.file_path, "r") as file:
            config_options = yaml.load(file, Loader=yaml.FullLoader)
        

        return None

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


# drawing = False
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

