# import numpy as np
# import matplotlib.pyplot as plt

# class OccupancyGrid:
#     def __init__(self, size):
#         self.size = size
#         self.grid = np.zeros(size, dtype=np.uint8)
        
#         self.fig, self.ax = plt.subplots()
#         self.ax.imshow(self.grid, cmap='binary', interpolation='none')
#         self.ax.set_xticks(np.arange(0, size[1]+1, 0.5), minor=True)
#         self.ax.set_yticks(np.arange(0, size[0]+1, 0.5), minor=True)
#         self.ax.grid(which="major", color="black", linewidth=1)
#         self.fig.canvas.mpl_connect('button_release_event', self.on_click)
        
#         plt.show()
    
#     def on_click(self, event):
#         if event.button == 1:  # Left mouse button
#             row, col = int(round(event.ydata)), int(round(event.xdata))
#             if 0 <= row < self.size[0] and 0 <= col < self.size[1]:
#                 self.grid[row, col] = 1 - self.grid[row, col]  # Toggle cell state
#                 self.ax.imshow(self.grid, cmap='binary', interpolation='nearest')
#                 plt.draw()

# def main():
#     grid_size = (20, 20)
#     occupancy_grid = OccupancyGrid(grid_size)

# if __name__ == "__main__":
#     main()
import numpy as np
import matplotlib.pyplot as plt

def create_empty_grid(grid_size):
    return np.zeros(grid_size, dtype=int)

def convert_meters_to_cells(position, resolution):
    return tuple(round(p / resolution) for p in position)

def update_grid(grid, position, resolution):
    cell_position = convert_meters_to_cells(position, resolution)
    if 0 <= cell_position[1] < grid.shape[0] and 0 <= cell_position[0] < grid.shape[1]:
        grid[cell_position[1], cell_position[0]] = 1

def on_press(event):
    global drawing
    drawing = True
    update_grid(grid, (event.xdata, event.ydata), resolution)
    im.set_data(grid)
    plt.draw()

def on_release(event):
    global drawing
    drawing = False

def on_motion(event):
    if drawing:
        update_grid(grid, (event.xdata, event.ydata), resolution)
        im.set_data(grid)
        plt.draw()

# Parameters
grid_size = (20, 20)     # Size of the occupancy grid in meters (rows, columns)
resolution = 1        # Resolution of the grid in meters
drawing = False

# Create an empty grid
grid = create_empty_grid(grid_size)

# Initialize the plot
plt.figure(figsize=(10, 10))
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