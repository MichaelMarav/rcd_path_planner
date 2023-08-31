#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import math
import threading 
from matplotlib import colors

class Point:
    def __init__(self,x,y,angle = None):
        self.x = x
        self.y = y
        self.angle = angle


# Hyperparams
real_time_plotting = True
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
target_pos     = np.empty(0,dtype=object)
robot_pos      = np.empty(0,dtype=object)
visited_robot  = np.empty(0,dtype=object)
visited_target = np.empty(0,dtype=object)
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
Input: Source point x0,y0 + orientation
'''
def check_intersection(x0,y0,r,lines):
    print(lines.shape)
    if lines.shape[0] == 0:
        return None
    for point in lines:
        x2 = point[1].x
        y2 = grid_size[1] - point[1].y
        x1 = point[0].x
        y1 = grid_size[1] - point[0].y
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

        if intersection_x >= 0 and intersection_x <= grid_size[0] and intersection_y >= 0 and intersection_y < grid_size[1]:
            plt.scatter(intersection_x,intersection_y,c = 'purple', s= 90)
            return (intersection_x, intersection_y)
        else:
            return None


def find_angle_to_exclude(in_angle):
    if in_angle is None:
        return None 
    res = in_angle + 180
    if res >= 360:
        res -= 360
    elif (res < 0):
        res += 360
    print(res)
    return res

def check_closest_distance(x,y,visited_points,closest_threshold):
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


'''
Choose index to cast
'''
def choose_index(robot_pos):
    pass

def ray_casting_robot(x,y):
    global path_found,robot_pos,visited_robot,visited_target

    
    for angle in range(0,360,int(360/num_beams)): # TODO: Add condition to stop if the arc length R dtheta is greater than the robot's size
        indexes_to_change = []

        angle_rad = np.radians(angle)

        stop_beam = False # Stop casting flag
        
        valid_ray = False

        dis = 3#robot_size/grid_resolution # Starting distance (radius) from ray casting 
        prev_x = int(math.ceil(x))
        prev_y = int(math.ceil(y))
        while True and not path_found:
            dis += 1
            beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
            beam_y = int(math.ceil(y - dis * np.sin(angle_rad)))
           
            if not stop_beam and beam_x > 0 and beam_x < grid_size[0] and beam_y > 0 and beam_y < grid_size[1]:
                # Found same ray
                if(grid[beam_x,beam_y] == 100):
                    stop_beam = True

                    # Don't crash with wall next to the point
                    if check_closest_distance(beam_x,beam_y,visited_robot,10):
                        valid_ray = True
                        robot_pos  = np.append(robot_pos,Point(prev_x,prev_y))
                        visited_robot  = np.append(visited_robot,Point(prev_x,prev_y))

                        # Plot with red the hit point
                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=20, label='Robot')
        
                elif (grid[beam_x,beam_y] == 80 or grid[beam_x-1,beam_y] == 80 or grid[beam_x+1,beam_y] == 80 or grid[beam_x,beam_y+1] == 80 or grid[beam_x,beam_y-1] == 80):
                    stop_beam = True
                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,80)
                    if intersect_point_x is not None and intersect_point_y is not None and check_closest_distance(intersect_point_x,intersect_point_y,visited_robot,10):
                        valid_ray = True
                        plt.scatter(intersect_point_x,intersect_point_y,s = 20, color = 'purple')
                        robot_pos  = np.append(robot_pos, Point(intersect_point_x,intersect_point_y))
                        visited_robot  = np.append(visited_robot,Point(intersect_point_x,intersect_point_y))
                
                elif (grid[beam_x,beam_y] == 40 or grid[beam_x-1,beam_y] == 40 or grid[beam_x+1,beam_y] == 40 or grid[beam_x,beam_y+1] == 40 or grid[beam_x,beam_y-1] == 40):
                    path_found = True
                    print("FOUND PATH by robot")
                    valid_ray = True

                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,40)
                    plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'green')

                else:
                    # For plotting with blue color where the beam has passed
                    indexes_to_change.append([beam_x, beam_y])

                prev_x = beam_x
                prev_y = beam_y
            
            else: 
                break

        if valid_ray:
            for row_idx, col_idx in indexes_to_change:
                grid[row_idx,col_idx] = 80



def ray_casting_target(x,y):
    global path_found,robot_pos,target_pos,visited_robot,visited_target

    
    for angle in range(0,360,int(360/num_beams)): # TODO: Add condition to stop if the arc length R dtheta is greater than the robot's size
        indexes_to_change = []

        angle_rad = np.radians(angle)

        stop_beam = False # Stop casting flag
        
        valid_ray = False

        dis = 3#robot_size/grid_resolution # Starting distance (radius) from ray casting 
        prev_x = int(math.ceil(x))
        prev_y = int(math.ceil(y))
        while True and not path_found:
            dis += 1
            beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
            beam_y = int(math.ceil(y - dis * np.sin(angle_rad)))
           
            if not stop_beam and beam_x > 0 and beam_x < grid_size[0] and beam_y > 0 and beam_y < grid_size[1]:
                # Found same ray
                if(grid[beam_x,beam_y] == 100):
                    stop_beam = True

                    # Don't crash with wall next to the point
                    if check_closest_distance(beam_x,beam_y,visited_target,10):
                        valid_ray = True
                        target_pos     = np.append(target_pos,Point(prev_x,prev_y))
                        visited_target = np.append(visited_target,Point(prev_x,prev_y))

                        # Plot with red the hit point
                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=20, label='Robot')
        
                elif (grid[beam_x,beam_y] == 40 or grid[beam_x-1,beam_y] == 40 or grid[beam_x+1,beam_y] == 40 or grid[beam_x,beam_y+1] == 40 or grid[beam_x,beam_y-1] == 40):
                    stop_beam = True
                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,40)
                    if intersect_point_x is not None and intersect_point_y is not None and check_closest_distance(intersect_point_x,intersect_point_y,visited_target,10):
                        valid_ray = True
                        plt.scatter(intersect_point_x,intersect_point_y,s = 20, color = 'purple')
                        target_pos  = np.append(target_pos, Point(intersect_point_x,intersect_point_y))
                        visited_target  = np.append(visited_target,Point(intersect_point_x,intersect_point_y))
                elif (grid[beam_x,beam_y] == 80 or grid[beam_x-1,beam_y] == 80 or grid[beam_x+1,beam_y] == 80 or grid[beam_x,beam_y+1] == 80 or grid[beam_x,beam_y-1] == 80):
                    valid_ray = True

                    path_found = True
                    intersect_point_x,intersect_point_y = check_possible_intersection(grid,beam_x,beam_y,80)
                    print("FOUND PATH by target")
                    plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'green')

                else:
                    # For plotting with blue color where the beam has passed
                    indexes_to_change.append([beam_x, beam_y])

                prev_x = beam_x
                prev_y = beam_y
            
            else: 
                break

        if valid_ray:
            for row_idx, col_idx in indexes_to_change:
                grid[row_idx,col_idx] = 40


if __name__ == "__main__":
    # Add condition to create or to read occupancy grid
    init_grid()

    draw_grid()  
    cur_it = 1
    do_robot = True
    while True:
        if do_robot  and robot_pos.shape[0] >= 1:
                ray_casting_robot(robot_pos[0].x,robot_pos[0].y)
                # Add sophisticated condition for choosing casting index
                robot_pos = np.delete(robot_pos,0)
                do_robot = False

        elif not do_robot and target_pos.shape[0] >= 1:
                ray_casting_target(target_pos[0].x,target_pos[0].y)
                target_pos = np.delete(target_pos,0)
                do_robot = True
        elif target_pos.shape[0] < 1 and robot_pos.shape[0] < 1 and not path_found: 
            print("Could not find path") 
            print("Total Iterations = ", cur_it)
            break
        else:
            do_robot = not do_robot


        if real_time_plotting:
            target_beam_indices = np.where(grid == 40)
            plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

            robot_beam_indices = np.where(grid == 80)
            plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')

            plt.pause(0.1)  # Pause to allow time for updates to be shown
            input("Press Enter to Continue...")

            # break # Remove this to check if it works

        if path_found:
            print("Found path")
            print("Total Iterations = ", cur_it)
            break
        cur_it += 1

        

   
    if not real_time_plotting:
        target_beam_indices = np.where(grid == 40)
        plt.scatter(target_beam_indices[0], target_beam_indices[1], color='red', s=2, label='TargetVirtual Beams')

        robot_beam_indices = np.where(grid == 80)
        plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')

        plt.pause(0.01)  # Pause to allow time for updates to be shown

    # Plot The path (grid values = 50 for path)

    # fig2 = plt.figure()
    # grid = np.where((grid != 0) & (grid != 100), 0, grid) # Removes casts

    # im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
    # plt.scatter(robot_beam_indices[0], robot_beam_indices[1], color='blue', s=2, label='RobotVirtual Beams')
    # plt.title('Figure 2')
    # plt.xlabel('x')
    # plt.ylabel('y')
    plt.show()
    input("Press Enter to exit...")
