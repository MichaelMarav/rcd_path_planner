#! /usr/bin/env python3
from re import T
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
matplotlib.rcParams['font.size'] = 16  # Choose your desired font size

import matplotlib.pyplot as plt
import math
import networkx as nx
import random
from scipy.interpolate import pchip_interpolate
import sys
from scipy.signal import convolve2d
import time
from copy import copy
import os 
import json

offline_experiments = False

# Hyperparams
real_time_plotting = False
draw_edge_split    = False
random_source_dir  = True

robot_size = 1 # (m) Robot's diameter
grid_resolution = 0.1 # (m)
workspace_size = (50,30) # (m) Size of the workspace/room where the robot needs to navigate
num_beams = 6
drawing_brush_size = int(3/grid_resolution)  # Size of the brush (in grid cells)




# Parameters
grid_size = (int(workspace_size[0]/grid_resolution), int(workspace_size[1]/grid_resolution))     # Size of the occupancy grid in meters (rows, columns)
wall_size = int(math.ceil((robot_size)/grid_resolution))

# Initialize Global variables
target_pos     = [0,0]
robot_pos      = [0,0]
visited_robot  = np.empty(0,dtype=object)
visited_target = np.empty(0,dtype=object)
coverage = 0


grid         = np.zeros(grid_size)
grid_edge_id = np.empty(grid_size,dtype=object) # Contains the edge id that passes through each cell


drawing = False
init_robot_pos= False
init_target_pos= False
path_found = False

robot_graph  = nx.Graph()
target_graph = nx.Graph() 



# Offline testing parameters 
os.chdir("../../Data/")
# print(os.getcwd())
RUN_ALL_DATASETS = True
SAVE_RESULTS = True
file_to_test = '4.png'  # Taken into account only if RUN_ALL_DATASETS = False

# Objects
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y  

class Edge:
    def __init__(self,edge_id,start_node,end_node):
        self.edge_id    = edge_id # Unique String for specifing which edge is this
        self.start_node = start_node
        self.end_node   = end_node



class Final_Node:
    def __init__(self,robot_found_path,x,y):
        self.robot_found_path    = False # Unique String for specifing which edge is this
        self.x = x
        self.y = y



'''
Initializes the occupancy grid
'''
def init_grid():
    # Init empty occ grid
    global grid, grid_edge_id, wall_size

    # Every element in the grid_edge_id array is a Null object
    for i in range(grid_edge_id.shape[0]):
        for j in range(grid_edge_id.shape[1]):
            grid_edge_id[i,j] = Edge(None,None,None)

    if not offline_experiments:
        # Add walls to the boundary of the grid
        grid[0:wall_size, :] = 100
        grid[-wall_size:, :] = 100
        grid[:, 0:wall_size] = 100
        grid[:, -wall_size:] = 100
    
    return


'''
Utility functions for drawing 
----------------------------------------------------------------------
'''

# Adds obstacles by drawing at the plot
def draw_obstacles(grid, position, drawing_brush_size):
    cell_position = (round(position[0]),round(position[1]))
    half_brush = drawing_brush_size // 2

    # for testing 
    
    for x in range(10,400):
        for y in range(140,160):
            for i in range(-half_brush, half_brush + 1):
                for j in range(-half_brush, half_brush + 1):
                    # x = int(cell_position[0] + i)
                    # y = int(cell_position[1] + j)
                    if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                        grid[x, y] = 100



    for x in range(80,100):
            for y in range(210,290):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100


    for x in range(160,180):
        for y in range(160,240):
            for i in range(-half_brush, half_brush + 1):
                for j in range(-half_brush, half_brush + 1):
                    # x = int(cell_position[0] + i)
                    # y = int(cell_position[1] + j)
                    if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                        grid[x, y] = 100

    for x in range(240,260):
            for y in range(210,290):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100

    
    for x in range(380,400):
            for y in range(160,240):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100


   # DOWN ----------------------------------------------------------
    for x in range(80,100):
            for y in range(60,140):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100


    for x in range(160,180):
        for y in range(10,90):
            for i in range(-half_brush, half_brush + 1):
                for j in range(-half_brush, half_brush + 1):
                    # x = int(cell_position[0] + i)
                    # y = int(cell_position[1] + j)
                    if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                        grid[x, y] = 100

    for x in range(240,260):
            for y in range(60,140):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100

    
    for x in range(380,400):
            for y in range(10,90):
                for i in range(-half_brush, half_brush + 1):
                    for j in range(-half_brush, half_brush + 1):
                        # x = int(cell_position[0] + i)
                        # y = int(cell_position[1] + j)
                        if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                            grid[x, y] = 100


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
            target_pos = [43,70]#[round(event.xdata),round(event.ydata)]
            init_target_pos = True
            plt.scatter([target_pos[0]], [target_pos[1]], color='green', marker='o', s=80, label='Target', zorder=2)
        if not init_robot_pos:
            robot_pos = [43,230]#[round(event.xdata),round(event.ydata)]
            init_robot_pos = True
            plt.scatter([robot_pos[0]], [robot_pos[1]], color='blue', marker='o', s=80, label='Robot', zorder=2)

        plt.draw()



def draw_grid():
    global im
    plt.figure(figsize=(workspace_size[0], workspace_size[1]))

    im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
    plt.gca().invert_yaxis()
    plt.title('Ray Casting and Diffusion Model')
    plt.axis('off')  # Turn on axis for grid lines

    # # Connect the mouse events
    press   = plt.connect('button_press_event', on_press)
    release = plt.connect('button_release_event', on_release)
    motion  = plt.connect('motion_notify_event', on_motion)
    goal    = plt.connect('key_press_event', set_goal_robot)

    plt.show(block = False)

    print("Draw and place goal and robot")
    input("Press Enter when setup is completed...")


    plt.disconnect(press)
    plt.disconnect(release)
    plt.disconnect(motion)
    plt.disconnect(goal)
    return
#----------------------------------------------------------------------



'''
Uses the convolve2d to inflate the occupancy grid by robot_size//2
'''
def inflate_occupancy_grid(robot_size):
    global grid, grid_resolution
    inflated_grid = np.copy(grid)

    # Create a binary mask of occupied cells
    occupied_mask = (grid == 100).astype(np.int32)

    # Create a circular kernel for inflation
    kernel_size = int(robot_size / grid_resolution)
    kernel = np.zeros((kernel_size * 2 + 1, kernel_size * 2 + 1), dtype=np.int32)
    y, x = np.ogrid[-kernel_size:kernel_size + 1, -kernel_size:kernel_size + 1]
    mask = x**2 + y**2 <= robot_size**2
    kernel[mask] = 1

    # Use convolution to inflate the occupied cells
    inflated_occupied = convolve2d(occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)
    
    # Update the inflated grid
    inflated_grid[inflated_occupied > 0] = 100

    if not np.allclose(inflated_grid, grid):
        grid = inflated_grid


# Returns true if the node has been casted else it returns false
def get_casted_flag(graph,node_name):
    return nx.get_node_attributes(graph,'ray_casted')[node_name]

# Returns the x,y coordinates of a node that belongs to graph
def get_node_position(graph,node_name):
    x_v = nx.get_node_attributes(graph,'x')[node_name]
    y_v = nx.get_node_attributes(graph,'y')[node_name]
    return x_v,y_v


# Checks if the node-to-add is really close to another node.
# Returns False if they are too close, else true
def check_closest_node_distance(x,y,visited_points,closest_threshold):
    for pos in visited_points:
        if math.sqrt((pos.x-x)**2 + (pos.y-y)**2) < closest_threshold:
            return False
    return True


'''
Checks if the beam intersects with another one
Condition--> Returns bool
'''
def check_possible_intersection(grid,beam_x,beam_y,value):
    if grid[beam_x,beam_y] == value:
        return beam_x,beam_y
    if grid[beam_x-1,beam_y] == value:
        return beam_x-1,beam_y
    if grid[beam_x+1,beam_y] == value:
        return beam_x+1,beam_y
    if  grid[beam_x,beam_y+1] == value:
        return beam_x,beam_y+1
    if grid[beam_x,beam_y-1] == value:
        return beam_x,beam_y-1
    return None, None



def split_edge(graph,new_node,x,y):
    global grid_edge_id
    edge_ = grid_edge_id[x,y]
    # I want to remove every edge_id that is the one intersected and I want to create 2 new edges one from the parent to the new point and one from the new node to the child
    node_1 = edge_.start_node
    node_2 = new_node
    node_3 = edge_.end_node

    x_1,y_1 = get_node_position(graph,node_1)
    x_2,y_2 = get_node_position(graph,node_2)
    x_3,y_3 = get_node_position(graph,node_3)

    dis_12 = math.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)
    dis_23 = math.sqrt((x_2-x_3)**2 + (y_2-y_3)**2)


    mask = np.vectorize(lambda obj: obj.edge_id == edge_.edge_id)(grid_edge_id)
    indices = np.argwhere(mask) # Stores the indices of grid_edge_id where the edge we are about to delete is located

    edge_name_1 = node_1 + "-" + node_2
    edge_name_2 = node_2 + "-" + node_3

    if x_1 < x_2: 
        for idx in indices:
            if idx[0] < x_2:
                grid_edge_id[idx[0],idx[1]] = Edge(edge_name_1,node_1,node_2)
            else:
                grid_edge_id[idx[0],idx[1]] = Edge(edge_name_2,node_2,node_3)
    else:
        for idx in indices:
            if idx[0] < x_2:
                grid_edge_id[idx[0],idx[1]] = Edge(edge_name_2,node_3,node_2)
            else:
                grid_edge_id[idx[0],idx[1]] = Edge(edge_name_1,node_2,node_1)

    graph.remove_edge(node_1,node_3)
    graph.add_edge(node_1,node_2,weight = dis_12)
    graph.add_edge(node_2,node_3,weight = dis_23)
    if draw_edge_split:
        mask = np.vectorize(lambda obj: obj.edge_id == edge_name_1)(grid_edge_id)
        indices = np.argwhere(mask) # Stores the indices of grid_edge_id where the edge we are about to delete is located
        plt.scatter(indices[:,0],indices[:,1],c = [np.random.rand(3)], marker='o', s=40) 

        mask = np.vectorize(lambda obj: obj.edge_id == edge_name_2)(grid_edge_id)
        indices = np.argwhere(mask) # Stores the indices of grid_edge_id where the edge we are about to delete is located
        plt.scatter(indices[:,0],indices[:,1],c = [np.random.rand(3)], marker='o', s=40) 




# Ray casting from parent node and create childs and directed edges
def ray_casting_robot(x,y,parent):

    global path_found, visited_robot, robot_graph, grid, grid_edge_id, target_graph,coverage
    child_id = 1
    edge_name = ""


    angle_list = np.arange(0,360,int(360/num_beams))
    if random_source_dir:
        random_rotation_bias = random.randint(0, 180) # Spin the orientation of the beams
        angle_list += random_rotation_bias


    for angle in angle_list: 

        # Saves the indices to change in the grid
        indexes_to_change = [] 

        angle_rad = np.radians(angle)

        stop_beam = False # Stop ray casting flag for this angle

        valid_ray = False # Flag that specifies if the ray is valid-> Not close enough to another node
        
        # Starting distance (radius) from ray casting 
        dis = 2 #(robot_size/2)/grid_resolution 
        prev_x = int(math.ceil(x))
        prev_y = int(math.ceil(y))

        # Propagates ray until 1. Ray hits wall 2. Ray hits another ray 3. Robot Ray and Goal ray are connected (path found)
        while not stop_beam and not path_found:
            dis += 1
            coverage += 1
            beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
            beam_y = int(math.ceil(y + dis * np.sin(angle_rad)))

            # Case 1: If beam hit wall
            if grid[beam_x,beam_y] == 100:
                stop_beam = True          


                if check_closest_node_distance(beam_x,beam_y,visited_robot,(robot_size)/grid_resolution):          
                    valid_ray = True

                    # Add node to graph
                    child_name = parent + str(child_id)
                    robot_graph.add_node(child_name, x = prev_x, y = prev_y, ray_casted = False)
                    robot_graph.add_edge(parent, child_name, weight = dis)

                    child_id += 1 # Update index for next child
                    
                    edge_name = parent + "-" + child_name

                    visited_robot  = np.append(visited_robot,Point(prev_x,prev_y)) # Save the places that the robot has visited
                    if not offline_experiments:
                        plt.plot([x,prev_x], [y,prev_y], c = 'b',zorder = 1)

                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Collision Points', zorder =2 )
                        # plt.plot([robot_pos[0],visited_robot[-1].x],[robot_pos[1],visited_robot[-1].y] , color='b', label = "Robot Rays",zorder = 1)

            # Case 2: If beam hits same kind of ray. (Checks in a cross-like manner)
            elif (grid[beam_x,beam_y] == 80 or grid[beam_x-1,beam_y] == 80 or grid[beam_x+1,beam_y] == 80 or grid[beam_x,beam_y+1] == 80 or grid[beam_x,beam_y-1] == 80):
                
                stop_beam = True

                intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,80) # where exactly the beams intersect
               
                if intersect_point_x is not None and intersect_point_y is not None and check_closest_node_distance(intersect_point_x, intersect_point_y, visited_robot, (robot_size)/grid_resolution):


                    valid_ray = True

                    # Add node
                    child_name = parent + str(child_id)
                    robot_graph.add_node(child_name, x = intersect_point_x, y = intersect_point_y, ray_casted = False)
                    robot_graph.add_edge(parent, child_name, weight = dis)

                    child_id += 1
                    edge_name = parent + "-" + child_name

                    # Break the edge that has the intersection in it
                    split_edge(robot_graph, child_name, intersect_point_x, intersect_point_y)


                    if not offline_experiments:
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'b', zorder  = 1)

                        plt.scatter(intersect_point_x,intersect_point_y,s = 50, color = 'purple', zorder = 3)

                    visited_robot  = np.append(visited_robot,Point(intersect_point_x,intersect_point_y))
            
            # Case 3: If beam hits the other kind of ray (path found)
            elif (grid[beam_x,beam_y] == 40 or grid[beam_x-1,beam_y] == 40 or grid[beam_x+1,beam_y] == 40 or grid[beam_x,beam_y+1] == 40 or grid[beam_x,beam_y-1] == 40):
                    path_found = True
                    valid_ray = True

                    print("FOUND PATH by robot")
                    intersect_point_x, intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,40) # where exactly the beams meet

                    # Add node
                    child_name = "F"
                    robot_graph.add_node(child_name, x= intersect_point_x, y = intersect_point_y, ray_casted = False)
                    robot_graph.add_edge(parent, child_name,weight = dis)
                    child_id += 1
                    edge_name = parent + "-" + child_name
                    


                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,40)

                    target_graph.add_node(child_name,x= beam_x,y=beam_y, ray_casted = False)

                    split_edge(target_graph, child_name, intersect_point_x, intersect_point_y)
                    if not offline_experiments:
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'b', zorder = 1)
                        plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'black',zorder = 2)
            else:
                indexes_to_change.append([beam_x,beam_y])
            
            prev_x = beam_x
            prev_y = beam_y
            if valid_ray:
                for row_idx, col_idx in indexes_to_change:
                    grid[row_idx,col_idx] = 80 # Robot ray has passed
                    grid_edge_id[row_idx,col_idx].edge_id = edge_name
                    grid_edge_id[row_idx,col_idx].start_node = parent
                    grid_edge_id[row_idx,col_idx].end_node    = child_name



# Ray casting from parent node and create childs and directed edges
def ray_casting_target(x,y,parent):

    global path_found, visited_target, target_graph, grid, grid_edge_id, robot_graph,coverage


    child_id = 1
    edge_name = ""

    angle_list = np.arange(0,360,int(360/num_beams))


    if random_source_dir:
        random_rotation_bias = random.randint(0, 180) # Spin the orientation of the beams
        angle_list += random_rotation_bias


    for angle in angle_list: 

        # Saves the indices to change in the grid
        indexes_to_change = [] 

        angle_rad = np.radians(angle)

        stop_beam = False # Stop ray casting flag for this angle

        valid_ray = False # Flag that specifies if the ray is valid-> Not close enough to another node
        
        # Starting distance (radius) from ray casting TODO: Make this start from robot_size
        dis = 2#(robot_size/2)/grid_resolution 
        prev_x = int(math.ceil(x))
        prev_y = int(math.ceil(y))

        # Propagates ray until 1. Ray hits wall 2. Ray hits another ray 3. Robot Ray and Goal ray are connected (path found)
        while not stop_beam and not path_found:
            dis += 1
            coverage += 1
            beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
            beam_y = int(math.ceil(y + dis * np.sin(angle_rad)))

            # Case 1: If beam hit wall
            if grid[beam_x,beam_y] == 100:
                stop_beam = True          
                if check_closest_node_distance(beam_x,beam_y,visited_target,(robot_size)/grid_resolution):          
                    valid_ray = True

                    # Add node to graph
                    child_name = parent + str(child_id)
                    target_graph.add_node(child_name, x = prev_x, y = prev_y, ray_casted = False)
                    target_graph.add_edge(parent, child_name, weight = dis)

                    child_id += 1 # Update index for next child
                    
                    edge_name = parent + "-" + child_name

                    visited_target  = np.append(visited_target,Point(prev_x,prev_y)) # Save the places that the robot has visited
                    if not offline_experiments:
                        plt.plot([x,prev_x], [y,prev_y], c = 'g', zorder = 1)

                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Collision Points', zorder = 3 )
                        # plt.plot([target_pos[0],visited_target[-1].x],[target_pos[1],visited_target[-1].y] , color='purple',label ="Target rays",zorder=1)



            # Case 2: If beam hits same kind of ray. (Checks in a cross-like manner)
            elif (grid[beam_x,beam_y] == 40 or grid[beam_x-1,beam_y] == 40 or grid[beam_x+1,beam_y] == 40 or grid[beam_x,beam_y+1] == 40 or grid[beam_x,beam_y-1] == 40):
                
                stop_beam = True

                intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,40) # where exactly the beams meet
               
                if intersect_point_x is not None and intersect_point_y is not None and check_closest_node_distance(intersect_point_x, intersect_point_y, visited_target,(robot_size)/grid_resolution):


                    valid_ray = True

                    # Add node
                    child_name = parent+str(child_id)
                    target_graph.add_node(child_name,x = intersect_point_x, y =intersect_point_y, ray_casted = False)
                    target_graph.add_edge(parent,child_name,weight = dis)

                    child_id += 1
                    edge_name = parent + "-" + child_name

                    # Break the edge that has the intersection in it
                    split_edge(target_graph, child_name, intersect_point_x, intersect_point_y)


                    if not offline_experiments:
                        plt.scatter(intersect_point_x,intersect_point_y,s = 50, color = 'purple')
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'g', zorder = 1)

                    visited_target = np.append(visited_target, Point(intersect_point_x,intersect_point_y))
            
            # Case 3: If beam hits the other kind of ray (path found)
            elif (grid[beam_x,beam_y] == 80 or grid[beam_x-1,beam_y] == 80 or grid[beam_x+1,beam_y] == 80 or grid[beam_x,beam_y+1] == 80 or grid[beam_x,beam_y-1] == 80):
                    path_found = True
                    valid_ray = True

                    print("FOUND PATH by Target")

                    # Add node
                    child_name = "F"
                    target_graph.add_node(child_name,x = beam_x, y = beam_y, ray_casted = False)
                    target_graph.add_edge(parent,child_name,weight = dis)
                    child_id += 1
                    edge_name = parent + "-" + child_name

                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,80)

                    robot_graph.add_node(child_name,x= beam_x,y=beam_y, ray_casted = False)

                    split_edge(robot_graph, child_name, intersect_point_x, intersect_point_y)
                    if not offline_experiments:
                        plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'black', zorder = 3)
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'g', zorder = 1)

            else:
                indexes_to_change.append([beam_x,beam_y])
            
            prev_x = beam_x
            prev_y = beam_y

            if valid_ray:
                for row_idx, col_idx in indexes_to_change:
                    grid[row_idx,col_idx] = 40 # Robot ray has passed
                    grid_edge_id[row_idx,col_idx].edge_id    = edge_name
                    grid_edge_id[row_idx,col_idx].start_node = parent
                    grid_edge_id[row_idx,col_idx].end_node   = child_name




'''
Finds the shortest path in both graphs (from R to F and from G to F) and constructs a way point path
'''
def find_shortest_path(robot_graph,target_graph):
    shortest_robot = nx.shortest_path(robot_graph,source = "R",target = "F", weight='weight')
    x_path = []
    y_path = []
    for name in shortest_robot:
        x_i, y_i = get_node_position(robot_graph,name)
        x_path.append(x_i)
        y_path.append(y_i)


    shortest_target = nx.shortest_path(target_graph,source = "G",target = "F",weight='weight')
    shortest_target = shortest_target[::-1]    # shortest_target.pop(0)
    shortest_target.pop(0) # Remove the common node from robot and target path
    

    for name in shortest_target:
        x_i, y_i = get_node_position(target_graph,name)
        x_path.append(x_i)
        y_path.append(y_i)

    path = [(x_path[i], y_path[i]) for i in range(len(x_path))]
    return path


def calc_dist_cell(x_list,y_list):
    d = 0
    for c1 in range(len(x_list)-1):
        d += math.sqrt( ( (x_list[c1+1]-x_list[c1]) )**2 + ( (y_list[c1+1]-y_list[c1]) )**2 )
    return d



'''
Generates samples on the initial path in order to perform LoS next
'''
def generate_samples(path):
    global robot_size, grid_resolution, grid
    generated_path = []
    for p in range(len(path)-1):
        new_point = list(path[p])
        generated_path.append(new_point)

        curr_point = path[p]
        next_point = path[p+1]
        distance_between_points = math.sqrt((curr_point[0] - next_point[0])**2  + (curr_point[1] - next_point[1])**2 )
        angle = math.atan2((next_point[1]-curr_point[1]),(next_point[0]-curr_point[0]))

        dis = robot_size/grid_resolution
        while dis < distance_between_points:
            x_p = int(math.ceil(curr_point[0] + dis * np.cos(angle)))
            y_p = int(math.ceil(curr_point[1] + dis * np.sin(angle)))
            dis += 2
            if (grid[x_p,y_p] == 100):
                continue
            generated_path.append([x_p,y_p])

    return generated_path


def check_collision(curr_point,next_point):
    global grid
    distance_between_points = math.sqrt((curr_point[0] - next_point[0])**2  + (curr_point[1] - next_point[1])**2 )
    dis = 3
    angle = math.atan2((next_point[1]-curr_point[1]),(next_point[0]-curr_point[0]))
    collision_flag = False
    while dis < distance_between_points and not collision_flag:
        x_p = int(math.ceil(curr_point[0] + dis * np.cos(angle)))
        y_p = int(math.ceil(curr_point[1] + dis * np.sin(angle)))
        dis += 1
        if grid[x_p,y_p] == 100:
            collision_flag = True
    if collision_flag:
        return True
    else:
        return False



def reduce_path_with_LoS(path):

    reduced_path = [list(path[0])]
    c = 0
    p = 1

    while  p < len(path):
        curr_point = path[c]
        next_point = path[p]

        collision_flag = check_collision(curr_point,next_point)
       
        if collision_flag:
            c = p - 1
            reduced_path.append(list(path[c]))
            p+=1 
        else:
            p += 1
     
        # if collision_flag:
        #     # GO BACK
        #     c = p - 1
        #     colision_flag = False
        #     while True:
        #         curr_point = path[c]
        #         collision_flag = check_collision(curr_point,next_point)
        #         if collision_flag:
        #             reduced_path.append(list(path[c+1]))
        #             c = c + 1
        #             p = c + 1
        #             break
        #         else: 
        #             c = c - 1

        # else:
        #     p += 1


    reduced_path.append(list(path[-1]))
    return np.array(reduced_path)


'''
@ Name calculate_path_distance: Finds the distance traveled at the produced path

'''
def calculate_path_distance(path):
    distance = 0 
    for p in range(len(path)-1):
        distance += math.sqrt((path[p][0]-path[p+1][0])**2 +(path[p][1]-path[p+1][1])**2) 
    return distance


def low_variance_resampling(graph):
    # Create a dictionary to store cumulative weights for each node
    cumulative_weights = {}
    
    # Calculate cumulative weights for all nodes
    for node in graph.nodes():
        neighbors = list(graph.neighbors(node))
        if not neighbors:
            cumulative_weights[node] = 0.0
        else:
            cumulative_weights[node] = sum(graph[node][neighbor]['weight'] for neighbor in neighbors)
    
    # Calculate the total weight of the entire graph
    total_weight = sum(cumulative_weights.values())
    
    # Generate a random number between 0 and the total weight
    random_value = random.uniform(0, total_weight)
    
    # Find the node whose cumulative weight encompasses the random value
    selected_node = None
    cumulative_weight_sum = 0.0
    for node, cumulative_weight in cumulative_weights.items():
        cumulative_weight_sum += cumulative_weight
        if random_value <= cumulative_weight_sum:
            selected_node = node
            break
    
    return selected_node



'''
--------------------------------------------------------------------------------------------
                                            ONLINE MAIN
--------------------------------------------------------------------------------------------

'''

def online_experiments_main():
    global grid, grid_resolution, grid_size, workspace_size, robot_size, wall_size, robot_pos, target_pos, path_found, drawing_brush_size


    init_grid()

    draw_grid()
   

    inflate_occupancy_grid(robot_size)
    


    # Add Robot Node to the graph
    robot_graph.add_node("R",x=robot_pos[0], y=robot_pos[1], ray_casted = False)
    target_graph.add_node("G",x=target_pos[0], y = target_pos[1], ray_casted = False)

    cant_cast_robot_count = 0
    cant_cast_target_count = 0
    
    start_time = time.time()

    while not path_found:

        # Do Low-Variance Resampling to figure out which node to cast next. After the path is found do low variance resampling but with diagonal - weight

        
        # Casting Robot Graph
        # for node in list(robot_graph.nodes):
        for i in range(robot_graph.number_of_nodes()):
            node = low_variance_resampling(robot_graph)

            if not get_casted_flag(robot_graph,node):
                casting_x,casting_y = get_node_position(robot_graph,node)
                ray_casting_robot(casting_x, casting_y, parent = node)
                robot_graph.nodes[node]['ray_casted'] = True
                cant_cast_robot_count = 0
                

            else:
                pass
                # cant_cast_robot_count += 1
                # if cant_cast_robot_count == robot_graph.number_of_nodes():
                #     sys.exit("Valid path from robot to goal does not exist")
                # else: 
                #    continue
            
            if real_time_plotting:
                plt.show(block = False)
                # target_beam_indices = np.where(grid == 40)

                # plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

                # robot_beam_indices = np.where(grid == 80)
                # plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')
                plt.pause(0.1)  # Pause to allow time for updates to be shown
                input("Press Enter to Continue...")

        # Casting Target Graph
        # for node in list(target_graph.nodes):   
        for i in range(target_graph.number_of_nodes()):
            node = low_variance_resampling(target_graph)

            if not get_casted_flag(target_graph,node):
                casting_x,casting_y = get_node_position(target_graph,node)
                ray_casting_target(casting_x, casting_y, parent = node)
                target_graph.nodes[node]['ray_casted'] = True
                cant_cast_target_count = 0
                
            else:
                pass
                # cant_cast_target_count += 1

                # if cant_cast_target_count == target_graph.number_of_nodes():
                #     sys.exit("Valid path from robot to goal does not exist")
                # else:
                #     continue


            if real_time_plotting:
                plt.show(block = False)
                # target_beam_indices = np.where(grid == 40)

                # plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

                # robot_beam_indices = np.where(grid == 80)
                # plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')
                plt.pause(0.1)  # Pause to allow time for updates to be shown
                input("Press Enter to Continue...")
    '''
    This part for finding the optimal path
    if path_found:
        # Change every weight of the edge to max_distance - current weight
        max_distance = math.sqrt(grid_size[0]**2 + grid_size[1]**2)
        
        # Invert the weights
        for u, v, data in robot_graph.edges(data=True):
            data['weight'] = max_distance - data['weight']

        for u, v, data in target_graph.edges(data=True):
            data['weight'] = max_distance - data['weight']

        path_found = False        
        while not path_found:

            for i in range(robot_graph.number_of_nodes()):
                node = low_variance_resampling(robot_graph)

                if not get_casted_flag(robot_graph,node):
                    casting_x,casting_y = get_node_position(robot_graph,node)
                    ray_casting_robot(casting_x, casting_y, parent = node)
                    robot_graph.nodes[node]['ray_casted'] = True
                    cant_cast_robot_count = 0
                    

            # Casting Target Graph
            # for node in list(target_graph.nodes):   
            for i in range(target_graph.number_of_nodes()):
                node = low_variance_resampling(target_graph)

                if not get_casted_flag(target_graph,node):
                    casting_x,casting_y = get_node_position(target_graph,node)
                    ray_casting_target(casting_x, casting_y, parent = node)
                    target_graph.nodes[node]['ray_casted'] = True
                    cant_cast_target_count = 0
    '''


    # At this point the path is found. TODO: add what to do when there is not a valid path

    # Find the shortest path (Node points) from robot and target to intersection then combine them and plot them
    path = find_shortest_path(robot_graph,target_graph)


    # Generate new samples
    path = generate_samples(path)
    path = np.array(path)
    # reduced_path = reduce_path_with_LoS(path)


    # new_path = generate_samples(reduced_path)
    prev_dist = 0
    curr_dis = calc_dist_cell(path[:,0],path[:,1])
    while True:
        print("optimized")
        reduced_path = reduce_path_with_LoS(path)
        curr_dis = calc_dist_cell(reduced_path[:,0],reduced_path[:,1])
        if abs(prev_dist - curr_dis) < 10 :
            break
        else:
            prev_dist = curr_dis
            path =  generate_samples(reduced_path)
    
    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Elapsed Time = ",elapsed_time," (s) | Coverage = ", coverage)


    reduced_path = reduce_path_with_LoS(path)

    # prev_distance = calculate_path_distance(reduced_path)

    # while True:
    #     new_path = generate_samples(reduced_path)

    #     reduced_path = reduce_path_with_LoS(new_path)

    #     curr_distance = calculate_path_distance(reduced_path)
    #     if abs(curr_distance-prev_distance) < 2:
    #         break
    #     else:
    #         prev_distance = curr_distance
    #         print("Optimized path")


    # --------------------- PLOTTING STAFF
    # Plots the ray casting result
    # if not real_time_plotting:
    #     target_beam_indices = np.where(grid == 40)
    #     plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

    #     robot_beam_indices = np.where(grid == 80)
    #     plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')

    #     plt.pause(0.01)  # Pause to allow time for updates to be shown
    #     input("Press Enter to Continue...")
    

    #----------------------------------------------------------
   
    # PLOT PATH WITH STRAIGHT LINES

    grid = np.where((grid != 0) & (grid != 100), 0, grid) # Remove rays
    # Create a figure
    fig2 = plt.figure(figsize=(workspace_size[0], workspace_size[1]))
    im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
    im.set_data(grid.T)
    plt.gca().invert_yaxis()
    plt.axis('off')  # Turn on axis for grid lines


    plt.title('Graph-based Shortest path')
    # Convert the path to a NumPy array for easier indexing
    path = np.array(path)

    # Plot the grid
    plt.plot(path[:, 0], path[:, 1], marker='o', color='red', markersize=5)  # Adjust color and marker size as needed
    plt.show(block = False)





    fig3 = plt.figure(figsize=(workspace_size[0], workspace_size[1]))
    im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
    im.set_data(grid.T)
    

    plt.title('Final path')
    # Convert the path to a NumPy array for easier indexing
    reduced_path = np.array(reduced_path)
    plt.axis('off')  # Turn on axis for grid lines

    # Plot the grid
    plt.plot(reduced_path[:, 0], reduced_path[:, 1], marker='o', color='red', markersize=5)  # Adjust color and marker size as needed
    plt.gca().invert_yaxis()

    plt.show(block = False)


    
    input("Press something to Exit")
    #----------------------------------------------------------



'''
--------------------------------------------------------------------------------------------
                                            OFFLINE MAIN
--------------------------------------------------------------------------------------------

'''

def offline_experiments_main():
    global grid, grid_resolution, grid_size, workspace_size, robot_size, wall_size, robot_pos, target_pos, robot_graph, target_graph, grid_edge_id, visited_robot, visited_target, path_found, coverage
    

    print("Running offline experiments")
    
    if RUN_ALL_DATASETS:
        name_list = os.listdir("./GRID/")
    else:
        name_list = [file_to_test]

    # Remove file extension from .png
    name_list = [f.split('.')[0] for f in name_list]


    for x in name_list:

        # Initialize graphs
        robot_graph.clear()
        target_graph.clear()
       
        # Load Grid
        grid = np.loadtxt("./CSV/" + x + ".csv", delimiter=",").astype(np.int64)
        print("Running experiment: ",os.getcwd() +"/CSV/" + x + ".json")

        with open("CSV/" + x +".json", 'r') as openfile:
            json_object = json.load(openfile)


        # Load Params
        grid_resolution = json_object['grid_resolution']
        grid_size = json_object['grid_size']
        workspace_size = json_object['workspace_size']
        robot_size = int(json_object['robot_size'])
        wall_size = int(json_object['wall_size'])
        robot_pos = [json_object['robot_x'], json_object['robot_y']]
        target_pos = [json_object['target_x'], json_object['target_y']]
        
        grid_edge_id = np.empty(grid_size,dtype=object) # Contains the edge id that passes through each cell

        visited_robot  = np.empty(0,dtype=object)
        visited_target = np.empty(0,dtype=object)
        
        init_grid()

        inflate_occupancy_grid(robot_size)



        coverage = 0
        

        # Add Robot Node to the graph
        robot_graph.add_node("R",x=robot_pos[0], y=robot_pos[1], ray_casted = False)
        target_graph.add_node("G",x=target_pos[0], y = target_pos[1], ray_casted = False)


        cant_cast_robot_count = 0
        cant_cast_target_count = 0
        start_time = time.time()
        path_found = False

        while not path_found:

            # Casting Robot Graph
            for i in range(robot_graph.number_of_nodes()):
                node = low_variance_resampling(robot_graph)

                if not get_casted_flag(robot_graph,node):
                    casting_x,casting_y = get_node_position(robot_graph,node)
                    ray_casting_robot(casting_x, casting_y, parent = node)
                    robot_graph.nodes[node]['ray_casted'] = True
                    cant_cast_robot_count = 0
                    

                else:
                    pass
                    # cant_cast_robot_count += 1
                    # if cant_cast_robot_count == robot_graph.number_of_nodes():
                    #     sys.exit("Valid path from robot to goal does not exist")
                    # else: 
                    #     continue

            # Casting Target Graph
            for i in range(target_graph.number_of_nodes()):
                node = low_variance_resampling(target_graph)

                if not get_casted_flag(target_graph,node):
                    casting_x,casting_y = get_node_position(target_graph,node)
                    ray_casting_target(casting_x, casting_y, parent = node)
                    target_graph.nodes[node]['ray_casted'] = True
                    cant_cast_target_count = 0
                    
                else:
                    pass
                    # cant_cast_target_count += 1

                    # if cant_cast_target_count == target_graph.number_of_nodes():
                    #     sys.exit("Valid path from robot to goal does not exist")
                    # else:
                    #     continue


        # Find the shortest path (Node points) from robot and target to intersection then combine them and plot them
        path = find_shortest_path(robot_graph,target_graph)

        # Generate new samples
        path = generate_samples(path)

        reduced_path = reduce_path_with_LoS(path)


        new_path = generate_samples(reduced_path)

        reduced_path = reduce_path_with_LoS(new_path)

        end_time = time.time()
        elapsed_time = end_time - start_time
        print("Elapsed Time = ",elapsed_time," (s)")


        # --------------------- SAVING STAFF --------------------
        #----------------------------------------------------------
        # Calculate traveled distance
        if SAVE_RESULTS:
            distance = calculate_path_distance(reduced_path)
            path_in_meters =[[element * grid_resolution for element in row] for row in reduced_path]
        
            np.savetxt("./Solutions/RayCaster_solution/"+ x +"_path.txt", path_in_meters, delimiter=",")
            solution = { 'duration' : elapsed_time, 'distance': distance,  'coverage' : coverage}
        
            with open("./Solutions/RayCaster_solution/"+ x + ".json", 'w') as convert_file:
                convert_file.write(json.dumps(solution))

        

        # --------------------- PLOTTING STAFF --------------------
        #----------------------------------------------------------
    
        # PLOT PATH WITH STRAIGHT LINES

        grid = np.where((grid != 0) & (grid != 100), 0, grid) # Remove rays
        # Create a figure
        fig2 = plt.figure(figsize=(workspace_size[0], workspace_size[1]))
        im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
        im.set_data(grid.T)
        plt.gca().invert_yaxis()


        plt.title('Smoothed Path')
        # Convert the path to a NumPy array for easier indexing
        plt.xlabel('X Coordinate')
        path = np.array(path)
        plt.ylabel('Y Coordinate')

        # Plot the grid
        plt.plot(path[:, 0], path[:, 1], marker='o', color='red', markersize=5)  # Adjust color and marker size as needed
        plt.show(block = False)





        fig3 = plt.figure(figsize=(workspace_size[0], workspace_size[1]))
        im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
        plt.gca().invert_yaxis()

        im.set_data(grid.T)
        

        plt.title('Final Path')
        # Convert the path to a NumPy array for easier indexing
        plt.xlabel('X Coordinate')
        reduced_path = np.array(reduced_path)
        plt.ylabel('Y Coordinate')

        # Plot the grid
        plt.plot(reduced_path[:, 0], reduced_path[:, 1], marker='o', color='red', markersize=5)  # Adjust color and marker size as needed
        plt.show(block = False)
        fig3.savefig('./Solutions/RayCaster_solution/'+ x + '.png')

        input("Press something to Exit")
        plt.close()

       
if __name__ == "__main__":

    if offline_experiments:
        offline_experiments_main()
    else:
        online_experiments_main()