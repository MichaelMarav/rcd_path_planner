#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math
import networkx as nx

# Custom imports
from occupancy_grid import Point,Grid
from visualizer import Visualizer

# Hyperparams
real_time_plotting = False
robot_size = 1 # (m) Robot's diameter
grid_resolution = 0.1 # (m) 
workspace_size = (50,30) # (m) Size of the workspace/room where the robot needs to navigate
num_beams = 6
drawing_brush_size = int(3/grid_resolution)  # Size of the brush (in grid cells)


# Parameters
target     = np.empty(0,dtype=object)
robot      = np.empty(0,dtype=object)
visited_robot  = np.empty(0,dtype=object)
visited_target = np.empty(0,dtype=object)
# Booleans
drawing = False
init_robot_pos= False
init_target_pos= False
path_found = False




class RayTracer:
    a = 3
    def __init__(self):
        pass




if __name__ == "__main__":
    map = Grid(workspace_size[0],workspace_size[1],grid_resolution,robot_size)
    vis = Visualizer(workspace_size[0],workspace_size[1],map.occ_grid,drawing_brush_size)
    print(map.occ_grid[0,0].occupied)
    # test = RayTracer.()