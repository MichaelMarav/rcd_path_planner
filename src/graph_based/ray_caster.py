#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import math
import networkx as nx


# Hyperparams
real_time_plotting = True
robot_size = 1 # (m) Robot's diameter
grid_resolution = 0.1 # (m)
workspace_size = (50,30) # (m) Size of the workspace/room where the robot needs to navigate
num_beams = 6
drawing_brush_size = int(3/grid_resolution)  # Size of the brush (in grid cells)




# Objects
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Edge:
    def __init__(self,edge_id,start_node,end_node):
        self.edge_id    = edge_id
        self.start_node = start_node
        self.end_node   = end_node



# Parameters
# Arrays
grid_size = (int(workspace_size[0]/grid_resolution), int(workspace_size[1]/grid_resolution))     # Size of the occupancy grid in meters (rows, columns)
# Initialize Global variables
target_pos     = np.empty(0,dtype=object)
robot_pos      = np.empty(0,dtype=object)
visited_robot  = np.empty(0,dtype=object)
visited_target = np.empty(0,dtype=object)

grid = np.zeros(grid_size, dtype=int)
object_grid = np.empty(0,dtype=object)

drawing = False
init_robot_pos= False
init_target_pos= False
path_found = False

robot_graph = nx.Graph()





'''
Initializes the occupancy grid
'''
def init_grid():
    # Init empty occ grid
    global grid
    grid = np.zeros(grid_size, dtype=int)

    # Add walls to the boundary of the grid
    wall_size = math.ceil(robot_size/grid_resolution)
    grid[0:wall_size, :] = 100
    grid[-wall_size:, :] = 100
    grid[:, 0:wall_size] = 100
    grid[:, -wall_size:] = 100

    return


'''
Adds obstacles by drawing at the plot
'''
def draw_obstacles(grid, position, drawing_brush_size):
    cell_position = (round(position[0]),round(position[1]))
    half_brush = drawing_brush_size // 2
    for i in range(-half_brush, half_brush + 1):
        for j in range(-half_brush, half_brush + 1):
            x = cell_position[0] + i
            y = cell_position[1] + j
            if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                grid[x, y] = 100


'''
Utility functions for drawing 
----------------------------------------------------------------------
'''
def on_press(event):
    global drawing
    drawing = True
    if (event.xdata is not None and event.ydata is not None):
        draw_obstacles(grid, (event.xdata, event.ydata), drawing_brush_size)
        im.set_data(grid.T)
        plt.draw()

def on_release(event):
    global drawing
    drawing = False

def on_motion(event):
    if (drawing and event.xdata is not None and event.ydata is not None):
        draw_obstacles(grid, (event.xdata, event.ydata), drawing_brush_size)
        im.set_data(grid.T)
        plt.draw()


def set_goal_robot(event):
    global init_robot_pos  # Declare init_robot_pos as global
    global init_target_pos
    global robot_pos,target_pos
    if event.xdata is not None and event.ydata is not None:
        if not init_target_pos and init_robot_pos:
            target_pos = np.append(target_pos,Point(round(event.xdata),round(event.ydata)))
            init_target_pos = True
            plt.scatter([target_pos[0].x], [target_pos[0].y], color='green', marker='o', s=50, label='Target')

        if not init_robot_pos:
            robot_pos = np.append(robot_pos,Point(round(event.xdata),round(event.ydata)))
            init_robot_pos = True
            plt.scatter([robot_pos[0].x], [robot_pos[0].y], color='black', marker='o', s=50, label='Robot')

        plt.draw()

def draw_grid():
    # Initialize the plot
    plt.figure(figsize=(workspace_size[0], workspace_size[1]))
    global im

    im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
    plt.title('Occupancy Grid (Left Mouse Button: Draw Occupied Cells)')
    plt.axis('off')  # Turn on axis for grid lines

    # Connect the mouse events
    press   = plt.connect('button_press_event', on_press)
    release = plt.connect('button_release_event', on_release)
    motion  = plt.connect('motion_notify_event', on_motion)
    goal    = plt.connect('key_press_event', set_goal_robot)
    print("Draw and place goal and robot")
    plt.show(block=False)
    input("Press Enter when setup is completed...")

    plt.disconnect(press)
    plt.disconnect(release)
    plt.disconnect(motion)
    plt.disconnect(goal)

    return
#----------------------------------------------------------------------



if __name__ == "__main__":
    init_grid()

    draw_grid()

