#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import math
import threading 
from matplotlib import colors
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y


# Hyperparams
real_time_plotting = False
robot_size = 1 # (m) Robot's diameter
grid_resolution = 0.1 # (m) 
workspace_size = (50,30) # (m) Size of the workspace/room where the robot needs to navigate
num_beams = 6
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
            x = cell_position[0] + i
            y = cell_position[1] + j
            if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                grid[x, y] = 100


def on_press(event):
    global drawing
    drawing = True
    if (event.xdata is not None and event.ydata is not None): 
        update_grid(grid, (event.xdata, event.ydata), drawing_brush_size)
        im.set_data(grid.T)
        plt.draw()

def on_release(event):
    global drawing
    drawing = False

def on_motion(event):
    if (drawing and event.xdata is not None and event.ydata is not None): 
        update_grid(grid, (event.xdata, event.ydata), drawing_brush_size)
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

       
        # Update the plot to show the robot and target positions
        # im.set_data(grid)
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




''' 
Test Multithreading for ray casting
Remove the size of  the robot but check around the beam
'''
# def ray_casting(x,y):
#     global path_found,robot_pos
#     indexes_to_change = []

#     for angle in range(0,360,int(360/num_beams)):
#         angle_rad = np.radians(angle)

#         stop_beam = False # Stop casting flag
        


#         dis = 0#robot_size/grid_resolution - 5# Starting distance (radius) from ray casting 
#         prev_x = int(math.ceil(x))
#         prev_y = int(math.ceil(y))
#         while True and not path_found:
#             dis += 1
#             if not stop_beam:
#                 beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
#                 beam_y = int(math.ceil(y - dis * np.sin(angle_rad)))

#                 # Found same ray
#                 if grid[beam_x,beam_y] == 80:
#                     plt.scatter([beam_x], [beam_y], color='purple', marker='o', s=50, label='Robot')
#                     stop_beam = True 
#                 elif(grid[beam_x,beam_y] == 100):
#                     if math.sqrt((beam_x-x)**2 + (beam_y-y)**2) > 5: # FIX HYPER
#                         robot_pos = np.append(robot_pos, Point(prev_x,prev_y))
#                         plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Robot')
#                     else:
#                         pass
#                     stop_beam = True


#                 else:
#                     indexes_to_change.append([prev_x, prev_y])
#                 prev_x = beam_x
#                 prev_y = beam_y
            
#             else: 
#                 break


      
                     
#     for row_idx, col_idx in indexes_to_change:
#         grid[row_idx,col_idx] = 80

'''
* Add line intersections
'''

def ray_casting(x,y):
    global path_found,robot_pos
    indexes_to_change = []

    for angle in range(0,360,int(360/num_beams)):
        angle_rad = np.radians(angle)

        stop_beam = False # Stop casting flag
        


        dis = robot_size/grid_resolution # Starting distance (radius) from ray casting 
        prev_x = int(math.ceil(x))
        prev_y = int(math.ceil(y))
        while True and not path_found:
            dis += 1
            beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
            beam_y = int(math.ceil(y - dis * np.sin(angle_rad)))
            if not stop_beam and beam_x > 0 and beam_x < grid_size[0] and beam_y > 0 and beam_y < grid_size[1]:
                # Found same ray
                if grid[beam_x,beam_y] == 80:
                    plt.scatter([beam_x], [beam_y], color='purple', marker='o', s=50, label='Robot')
                    stop_beam = True 
                elif(grid[beam_x,beam_y] == 100):
                    if math.sqrt((beam_x-x)**2 + (beam_y-y)**2) > 5: # FIX HYPER
                        robot_pos = np.append(robot_pos, Point(prev_x,prev_y))
                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Robot')
                    else:
                        pass
                    stop_beam = True


                else:
                    indexes_to_change.append([prev_x, prev_y])
                prev_x = beam_x
                prev_y = beam_y
            
            else: 
                break
            
    for row_idx, col_idx in indexes_to_change:
        grid[row_idx,col_idx] = 80



if __name__ == "__main__":
    # Add condition to create or to read occupancy grid
    init_grid()

    draw_grid()  

    it = 1
    max_it = 1
    while not path_found and it <= max_it:
        while True:
            if robot_pos.shape[0] >= 1:
                ray_casting(robot_pos[-1].x, robot_pos[-1].y)
                robot_pos = np.delete(robot_pos, -1)
                #input("Press Enter to continue...")
            else:
                break

            if real_time_plotting:
                target_beam_indices = np.where(grid == 40)
                plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

                robot_beam_indices = np.where(grid == 80)
                plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')

                plt.pause(0.1)  # Pause to allow time for updates to be shown
        it += 1
        

   
    if not real_time_plotting:
        target_beam_indices = np.where(grid == 40)
        plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

        robot_beam_indices = np.where(grid == 80)
        plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')

        plt.pause(0.01)  # Pause to allow time for updates to be shown
    
    input("Press Enter to exit...")


# Finds the intersection point

'''

import math
import matplotlib.pyplot as plt
import numpy as np
def find_intersection(x0, y0, x1, y1, x2, y2, r):
    # Calculate direction vectors
    segment_dir = (x2 - x1, y2 - y1)
    semi_infinite_dir = (math.cos(r), math.sin(r))

    # Solve for t and s
    t_numerator = (x0 - x1) * semi_infinite_dir[1] - (y0 - y1) * semi_infinite_dir[0]
    t_denominator = segment_dir[0] * semi_infinite_dir[1] - segment_dir[1] * semi_infinite_dir[0]
    
    if t_denominator == 0:
        # Lines are parallel
        return None

    t = t_numerator / t_denominator

    if t < 0 or t > 1:
        # Intersection point is outside the line segment
        return None

    s = ((x1 - x0) + t * segment_dir[0]) / semi_infinite_dir[0]

    if s < 0:
        # Intersection point is behind the semi-infinite line
        return None

    # Calculate intersection point
    intersection_x = x1 + t * segment_dir[0]
    intersection_y = y1 + t * segment_dir[1]

    if intersection_x >= 0 and intersection_x <= 10 and intersection_y >= 0 and intersection_y <= 10:
        return (intersection_x, intersection_y)
    else:
        return None

# Example inputs
x0, y0 = 2,5
x1, y1 = 1, 1
x2, y2 = 9, 9
r = math.radians(270)

intersection = find_intersection(x0, y0, x1, y1, x2, y2, r)

# Create a grid for visualization
grid_size = 10
grid = np.zeros((grid_size, grid_size))

# Mark endpoints and intersection on the grid
grid[int(y0), int(x0)] = 2  # Starting point
grid[int(y1), int(x1)] = 1  # Endpoint 1
grid[int(y2), int(x2)] = 1  # Endpoint 2

# Plot the lines and points
plt.figure(figsize=(8, 8))
# plt.imshow(grid, origin='lower', cmap='cool', extent=[0, grid_size, 0, grid_size], alpha=0.3)

# Plot the line segment
plt.plot([x1, x2], [y1, y2], color='blue', label='Line Segment')

# Plot the semi-infinite line
line_length = 10
line_end_x = x0 + line_length * math.cos(r)
line_end_y = y0 + line_length * math.sin(r)
plt.plot([x0, line_end_x], [y0, line_end_y], color='red', linestyle='dashed', label='Semi-Infinite Line')

# Highlight the intersection point if it exists
if intersection:
    intersection_x, intersection_y = intersection
    plt.scatter(intersection_x, intersection_y, color='green', label='Intersection Point')

plt.title("Line and Intersection Visualization")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid()

plt.show()


'''