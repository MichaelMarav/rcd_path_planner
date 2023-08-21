import numpy as np
import matplotlib.pyplot as plt


# TODO: 
'''
IDEAS: 
* Once a beam detects collision from target and robot compare the distances from other intersections and select the smallest
1) Change the beam size to the robot's size 
2) Add diffusion 
3) Add condition to stop(when robot_beam finds the goal beam)
4) Add saving occupancy grid option
'''


# Parameters
# Hyper
real_time_plotting = True
robot_size = 0.5 # m (robot's diameter)

# Arrays
grid_size = (500, 500)     # Size of the occupancy grid in meters (rows, columns)
target_pos =  [0,0]  
robot_pos  =  [0,0] 

# Ints
num_beams = 36
drawing_brush_size =   30        # Size of the brush
bounces_allowed = 2
# Booleans
drawing = False
init_robot_pos= False
init_target_pos= False

#---------------------------------------------------------------------------------------


# Create an empty grid
def create_empty_grid(grid_size):
    return np.zeros(grid_size, dtype=int)


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
    if event.xdata is not None and event.ydata is not None:
        if not init_target_pos and init_robot_pos:
            target_pos[0] = round(event.xdata)
            target_pos[1] = round(event.ydata)
            init_target_pos = True
            plt.scatter([target_pos[0]], [target_pos[1]], color='red', marker='o', s=40, label='Target')

        if not init_robot_pos:
            robot_pos[0] = round(event.xdata)
            robot_pos[1] = round(event.ydata)
            init_robot_pos = True
            plt.scatter([robot_pos[0]], [robot_pos[1]], color='blue', marker='o', s=robot_size/0.001, label='Robot')

       
        # Update the plot to show the robot and target positions
        im.set_data(grid)
        plt.draw()

# Find coverage percentage
def find_coverage():
    count = 0
    found_indices = np.where(grid == 100)
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row, col] == 40 or grid[row,col] == 80:
                count += 1
    print("Coverage Percentage: ", 100*count/(grid_size[0]*grid_size[1]-len(found_indices[0])), " %" )

grid = create_empty_grid(grid_size)


# Add walls at the limits of the gridreal_time_plotting = True

wall_size = 1
grid[0:wall_size, :] = 100
grid[-wall_size:, :] = 100
grid[:, 0:wall_size] = 100
grid[:, -wall_size:] = 100

# Initialize the plot
plt.figure(figsize=(20, 20))


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


# Generate beams from the robot and the goal
for angle in range(0, 360, int(360 / num_beams)):
    angle_rad = np.radians(angle)

    # Robot beam
    dis = 5 # Start range from robot
    while True:
        dis += 1

        beam_x_robot = round(robot_pos[1] + dis * np.cos(angle_rad))
        beam_y_robot = round(robot_pos[0] + dis * np.sin(angle_rad))

        # if grid[beam_x_robot, beam_y_robot] != 100:
        #     grid[beam_x_robot, beam_y_robot] = 80
        # else:
        #     break
        
        if grid[beam_x_robot, beam_y_robot] == 100:
            break
        else:
            grid[beam_x_robot, beam_y_robot] = 80
        
        
    # Target beam
    dis = 5 # Start range from target
    while True:
        dis += 1
        # Target beam
        beam_x_target = round(target_pos[1] + dis * np.cos(angle_rad))
        beam_y_target = round(target_pos[0] + dis * np.sin(angle_rad))

        if grid[beam_x_target, beam_y_target] == 100:
            break
        elif grid[beam_x_target,beam_y_target] == 80 : # TODO: This does not work properly the beam passes throught the other beam sometimes
            plt.scatter(beam_y_target,beam_x_target,color ="purple",s=40,label="Intersections")
            break
        else:
            grid[beam_x_target, beam_y_target] = 40

    if real_time_plotting:
        target_beam_indices = np.where(grid == 40)
        plt.scatter(target_beam_indices[1], target_beam_indices[0], color='red', s=2, label='TargetVirtual Beams')

        robot_beam_indices = np.where(grid == 80)
        plt.scatter(robot_beam_indices[1], robot_beam_indices[0], color='blue', s=2, label='RobotVirtual Beams')

        plt.pause(0.1)  # Pause to allow time for updates to be shown

if not real_time_plotting:
    target_beam_indices = np.where(grid == 40)
    plt.scatter(target_beam_indices[1], target_beam_indices[0], color='red', s=2, label='TargetVirtual Beams')

    robot_beam_indices = np.where(grid == 80)
    plt.scatter(robot_beam_indices[1], robot_beam_indices[0], color='blue', s=2, label='RobotVirtual Beams')

    plt.pause(0.01)  # Pause to allow time for updates to be shown





find_coverage()

input("Press Enter to exit...")

