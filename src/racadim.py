#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import math

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y


# Hyperparams
real_time_plotting = True
robot_size = 0.2 # (m) Robot's diameter
grid_resolution = 0.1 # (m) 
workspace_size = (50,50) # (m) Size of the workspace/room where the robot needs to navigate
num_beams = 36
drawing_brush_size = int(3/grid_resolution)  # Size of the brush (in grid cells)
bounces_allowed = 2 



# Parameters (Don't change)
# Arrays
grid_size = (int(workspace_size[0]/grid_resolution), int(workspace_size[1]/grid_resolution))     # Size of the occupancy grid in meters (rows, columns)
# Initialize Global variables
target_pos =  np.empty(0,dtype=object)
robot_pos  =  np.empty(0,dtype=object)

# Booleans
drawing = False
init_robot_pos= False
init_target_pos= False
path_found = False
#---------------------------------------------------------------------------------------


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


def break_casting():
    raise StopIteration

def update_grid(grid, position, drawing_brush_size):
    cell_position = (round(position[0]),round(position[1]))
    half_brush = drawing_brush_size // 2
    for i in range(-half_brush, half_brush + 1):
        for j in range(-half_brush, half_brush + 1):
            y = cell_position[1] + i
            x = cell_position[0] + j
            if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
                grid[y, x] = 100


def on_press(event):
    global drawing
    drawing = True
    if (event.xdata is not None and event.ydata is not None): 
        update_grid(grid, (event.xdata, event.ydata), drawing_brush_size)
        im.set_data(grid)
        plt.draw()

def on_release(event):
    global drawing
    drawing = False

def on_motion(event):
    if (drawing and event.xdata is not None and event.ydata is not None): 
        update_grid(grid, (event.xdata, event.ydata), drawing_brush_size)
        im.set_data(grid)
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

       
        # Update the plot to show the robot and target positions
        im.set_data(grid)
        plt.draw()

def draw_grid():
    # Initialize the plot
    plt.figure(figsize=(30, 30))
    global im

    im = plt.imshow(grid, cmap='binary', origin='upper', vmin=0, vmax=100)
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

'''
Ray cast two points 
'''
def ray_casting(x1,y1,x2,y2):
    global path_found,robot_pos,target_pos
    # Generate beams from the robot and the goal
    for angle in range(0, 360, int(360 / num_beams)):
        angle_rad = np.radians(angle)

        # Robot beam
        stop_robot_beam_flag  = False
        stop_target_beam_flag = False

        dis = robot_size/grid_resolution # Start range from robot

        while True and not path_found:
            dis += 1
            offset_lim = math.ceil(robot_size/grid_resolution)//2 # +- for the robot's size 

            if not stop_robot_beam_flag:

                # Beam center
                beam_x_robot = round(y1 + dis * np.cos(angle_rad))
                beam_y_robot = round(x1 + dis * np.sin(angle_rad))
                '''
                if grid[beam_x_robot,beam_y_robot] == 80:
                    found_Same_breakkkkk
                '''
                # Enlarge the beam to robot size 
                try:
                    for offset_x in range(-offset_lim,offset_lim+1):
                        for offset_y in range (-offset_lim,offset_lim+1):
                            x = beam_x_robot + offset_x                        
                            y = beam_y_robot + offset_y                
                            # Add break here to block robot's beam

                            if grid[x, y] == 40:
                                path_found = True
                                break_casting()
                            elif grid[x,y] == 100:
                                stop_robot_beam_flag = True
                                robot_pos = np.append(robot_pos, Point(beam_x_robot,beam_y_robot))
                                break_casting()
                            else: 
                                grid[x,y] = 80
                except StopIteration:
                    pass

            if not stop_target_beam_flag:

                # Target beam
                beam_x_target = round(y2 + dis * np.cos(angle_rad))
                beam_y_target = round(x2 + dis * np.sin(angle_rad))
                try:
                    for offset_x in range(-offset_lim,offset_lim+1):
                        for offset_y in range (-offset_lim,offset_lim+1):
                            x = beam_x_target + offset_x                        
                            y = beam_y_target + offset_y                
                            # Add break here to block target's beam
                            if grid[x,y] == 80:
                                path_found = True  # Stop when path is found
                                break_casting()
                            elif grid[x, y] == 100:
                                stop_target_beam_flag = True
                                target_pos = np.append(target_pos, Point(beam_x_target,beam_y_target))
                                break_casting()
                            else: 
                                grid[x, y] = 40

                except StopIteration:
                    pass
            
            if stop_robot_beam_flag and stop_target_beam_flag: 
                break # Breaks the while loop

        # REAL-TIME PLOTTING    
        if real_time_plotting:
            target_beam_indices = np.where(grid == 40)
            plt.scatter(target_beam_indices[1], target_beam_indices[0], color='red', s=2, label='TargetVirtual Beams')

            robot_beam_indices = np.where(grid == 80)
            plt.scatter(robot_beam_indices[1], robot_beam_indices[0], color='blue', s=2, label='RobotVirtual Beams')

            plt.pause(0.1)  # Pause to allow time for updates to be shown
               
        if path_found:
            break

        

    if not real_time_plotting:
        target_beam_indices = np.where(grid == 40)
        plt.scatter(target_beam_indices[1], target_beam_indices[0], color='red', s=2, label='TargetVirtual Beams')

        robot_beam_indices = np.where(grid == 80)
        plt.scatter(robot_beam_indices[1], robot_beam_indices[0], color='blue', s=2, label='RobotVirtual Beams')

        plt.pause(0.01)  # Pause to allow time for updates to be shown


if __name__ == "__main__":
    # Add condition to create or to read occupancy grid
    init_grid()

    draw_grid()  
    while not path_found:
            
        ray_casting(robot_pos[0].x,robot_pos[0].y,target_pos[0].x, target_pos[0].y)


    input("Press Enter to exit...")
