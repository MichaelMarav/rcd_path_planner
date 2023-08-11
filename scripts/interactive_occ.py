import numpy as np
import matplotlib.pyplot as plt



# Parameters
grid_size = (20, 20)     # Size of the occupancy grid in meters (rows, columns)
resolution = 1          # Resolution of the grid in meters
brush_size = 1          # Size of the brush
drawing = False

# Create an empty grid


def create_empty_grid(grid_size):
    return np.zeros(grid_size, dtype=int)

def convert_meters_to_cells(position, resolution):
    return tuple(round(p / resolution) for p in position)

def update_grid(grid, position, resolution, brush_size):
    cell_position = convert_meters_to_cells(position, resolution)
    half_brush = brush_size // 2
    for i in range(-half_brush, half_brush + 1):
        for j in range(-half_brush, half_brush + 1):
            y = cell_position[1] + i
            x = cell_position[0] + j
            if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
                grid[y, x] = 1

def on_press(event):
    global drawing
    drawing = True
    update_grid(grid, (event.xdata, event.ydata), resolution, brush_size)
    im.set_data(grid)
    plt.draw()

def on_release(event):
    global drawing
    drawing = False

def on_motion(event):
    if drawing:
        update_grid(grid, (event.xdata, event.ydata), resolution, brush_size)
        im.set_data(grid)
        plt.draw()




grid = create_empty_grid(grid_size)


# Add walls at the limits of the grid
wall_size = 4
grid[0:wall_size, :] = 1
grid[-wall_size:, :] = 1
grid[:, 0:wall_size] = 1
grid[:, -wall_size:] = 1

# Initialize the plot
plt.figure(figsize=(20, 20))
im = plt.imshow(grid, cmap='binary', origin='upper', vmin=0, vmax=1)
plt.title('Occupancy Grid (Left Mouse Button: Draw Occupied Cells)')
plt.axis('on')  # Turn on axis for grid lines

# Draw grid lines
plt.gca().xaxis.set_major_locator(plt.NullLocator())
plt.gca().yaxis.set_major_locator(plt.NullLocator())
plt.grid(True, which='both', color='black', linewidth=1)

# Connect the mouse events
plt.connect('button_press_event', on_press)
plt.connect('button_release_event', on_release)
plt.connect('motion_notify_event', on_motion)

plt.show()


print("HIMARK")

plt.figure(figsize=(20, 20))
im.set_data(grid)
im = plt.imshow(grid, cmap='binary', origin='upper', vmin=0, vmax=1)
plt.show()
