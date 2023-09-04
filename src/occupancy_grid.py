import numpy as np
import math
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y


class Grid:

    class grid_cell:

        def __init__(self):
            self.occupied = False
            self.robot_visited = False
            self.goal_visited  = False

    def __init__(self,size_x,size_y,grid_resolution,robot_size):
        # Initialize Grid Size in cells
        self.grid_size = (int(size_x/grid_resolution), int(size_y/grid_resolution))     
        # Initialize Grid limits (around occupancy grid)
        self.wall_size = math.ceil(robot_size/grid_resolution)

        # Initialize Occupancy Grid 
        self.occ_grid = np.empty(self.grid_size,dtype=object)
        self.occ_grid[:,:] = self.grid_cell()
        
        # Add walls to the limits
        self.occ_grid = self.add_walls()

        
    def add_walls(self):

        for i in range(self.wall_size):
            for j in range(self.grid_size[1]):
                self.occ_grid[i, j].occupied = True
                self.occ_grid[-i - 1, j].occupied = True

        for i in range(self.grid_size[0]):
            for j in range(self.wall_size):
                self.occ_grid[i, j].occupied = True
                self.occ_grid[i, -j - 1].occupied = True
       
       
    
       
       
       
        # self.occ_grid[0,0].occupied = True
        # self.wall_size = math.ceil(robot_size/grid_resolution)
        # self.occ_grid[-wall_size:, :].occupied = True
        # self.occ_grid[:, 0:wall_size].occupied = True
        # self.occ_grid[:, -wall_size:].occupied = True
        return self.occ_grid