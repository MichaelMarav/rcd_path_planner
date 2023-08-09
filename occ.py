import numpy as np
import matplotlib.pyplot as plt

class OccupancyGrid:
    def __init__(self, size):
        self.size = size
        self.grid = np.zeros(size, dtype=np.uint8)
        
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.grid, cmap='binary', interpolation='none')
        self.ax.set_xticks(np.arange(0, size[1]+1, 0.5), minor=True)
        self.ax.set_yticks(np.arange(0, size[0]+1, 0.5), minor=True)
        self.ax.grid(which="major", color="black", linewidth=1)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        plt.show()
    
    def on_click(self, event):
        if event.button == 1:  # Left mouse button
            row, col = int(round(event.ydata)), int(round(event.xdata))
            if 0 <= row < self.size[0] and 0 <= col < self.size[1]:
                self.grid[row, col] = 1 - self.grid[row, col]  # Toggle cell state
                self.ax.imshow(self.grid, cmap='binary', interpolation='nearest')
                plt.draw()

def main():
    grid_size = (20, 20)
    occupancy_grid = OccupancyGrid(grid_size)

if __name__ == "__main__":
    main()
