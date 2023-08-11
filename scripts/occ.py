import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation



# Parameters
grid_resolution = 0.1
grid_size = (int(50/grid_resolution), int(50/grid_resolution))   # Size of the occupancy grid (rows, columns)
robot_pos = (4/grid_resolution, 4/grid_resolution)     # Robot's initial position (row, column)
target_pos = (46/grid_resolution, 46/grid_resolution)  # Target's position (row, column)
num_beams = 36       # Number of beams generated by the robot

# Create an occupancy grid
occupancy_grid = np.ones(grid_size, dtype=int)

# Set unoccupied cells (open up the rooms)
unoccupied_cells = [
    (i, j) for i in range(int(2/grid_resolution), int(grid_size[0] - 2/grid_resolution)) for j in range(int(2/grid_resolution), int(grid_size[1] - 2/grid_resolution))
]
for row, col in unoccupied_cells:
    occupancy_grid[row, col] = 0

# Add walls to create rooms
rooms = [
    ((int(8/grid_resolution), int(8/grid_resolution) ), (int(18/grid_resolution), int(18/grid_resolution))),
    ((int(2/grid_resolution), int(18/grid_resolution)), (int(10/grid_resolution), int(28/grid_resolution)))
]
for (start_row, start_col), (end_row, end_col) in rooms:
    occupancy_grid[start_row:end_row + 1, start_col:end_col + 1] = 1

# Generate beams from the robot and the goal
beam_grid = np.zeros_like(occupancy_grid)
robot_x, robot_y = robot_pos
target_x, target_y = target_pos


for angle in range(0,360,int(360 / num_beams)):
    plt.figure(figsize=(20, 20))
    plt.axis('off')
    plt.xticks([])
    plt.yticks([])
    angle_rad = np.radians(angle)
    dis = 0
    while (True):
        dis += 1

        # Robot beam
        beam_x_robot = round(robot_x + dis*np.cos(angle))
        beam_y_robot = round(robot_y + dis*np.sin(angle))

        if 0 <= beam_x_robot < grid_size[0] and 0 <= beam_y_robot < grid_size[1] and occupancy_grid[beam_x_robot,beam_y_robot] != 1:
            occupancy_grid[beam_x_robot,beam_y_robot] = 2
        else:
            break
    dis=0
    while (True):
        dis += 1
        # Target beam
        beam_x_target = round(target_x + dis*np.cos(angle))
        beam_y_target = round(target_y + dis*np.sin(angle))

        if 0 <= beam_x_target < grid_size[0] and 0 <= beam_y_target < grid_size[1] and occupancy_grid[beam_x_target,beam_y_target] != 1:
            occupancy_grid[beam_x_target,beam_y_target] = 3
        else:
            break


    # Visualize the occupancy grid and beams for each iteration
    plt.clf()
    plt.imshow(occupancy_grid, cmap='binary', origin='upper', vmin=0, vmax=3)

    plt.scatter([robot_pos[1]], [robot_pos[0]], color='blue', marker='o', label='Robot')
    plt.scatter([target_pos[1]], [target_pos[0]], color='red', marker='o', label='Target')

    beam_indices = np.where(beam_grid == 1)
    plt.scatter(beam_indices[1], beam_indices[0], color='black', s=2, label='Virtual Beams')

    # plt.legend()
   
    plt.draw()
    plt.pause(0.01)
    # plt.show()  
    # Wait for user input to proceed to the next iteration
    key = plt.waitforbuttonpress()
    # if key:
    #     break

