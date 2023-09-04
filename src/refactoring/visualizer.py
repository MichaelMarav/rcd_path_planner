import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self,size_x,size_y,grid,drawing_brush_size):
        
        self.drawing_brush_size = drawing_brush_size 

        self.grid = grid
        # Initialize the plot
        plt.figure(figsize=(size_x, size_y))
        global im
        im = plt.imshow(grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
        plt.title('Occupancy Grid (Left Mouse Button: Draw Occupied Cells)')
        plt.axis('off')  # Turn on axis for grid lines

        # Connect the mouse events
        press   = plt.connect('button_press_event', self.on_press)
        release = plt.connect('button_release_event', self.on_release)
        motion  = plt.connect('motion_notify_event', self.on_motion)
        goal    = plt.connect('key_press_event', self.set_goal_robot)
        print("Draw and place goal and robot")
        plt.show(block=False)
        input("Press Enter when setup is completed...")

        plt.disconnect(press)
        plt.disconnect(release)
        plt.disconnect(motion)
        plt.disconnect(goal)


    def update_grid(grid, position, drawing_brush_size):
        cell_position = (round(position[0]),round(position[1]))
        half_brush = drawing_brush_size // 2
        for i in range(-half_brush, half_brush + 1):
            for j in range(-half_brush, half_brush + 1):
                x = cell_position[0] + i
                y = cell_position[1] + j
                if 0 <= y < grid.shape[1] and 0 <= x < grid.shape[0]:
                    grid[x, y].occupied = True

    def on_press(self,event):
        global drawing
        drawing = True
        if (event.xdata is not None and event.ydata is not None): 
            self.update_grid(self.grid, (event.xdata, event.ydata), self.drawing_brush_size)
            im.set_data(self.grid.T)
            plt.draw()

    def on_release(self,event):
        global drawing
        drawing = False

    def on_motion(self,event):
        if (drawing and event.xdata is not None and event.ydata is not None): 
            self.update_grid(self.grid, (event.xdata, event.ydata), self.drawing_brush_size)
            im.set_data(self.grid.T)
            plt.draw()


    def set_goal_robot(self,event):
        global init_robot_pos  # Declare init_robot_pos as global
        global init_target_pos
        global robot_pos,target_pos
        if event.xdata is not None and event.ydata is not None:
            if not init_target_pos and init_robot_pos:
                # target_pos = np.append(target_pos,Point(round(event.xdata),round(event.ydata)))
                init_target_pos = True
                plt.scatter([target_pos[0].x], [target_pos[0].y], color='green', marker='o', s=50, label='Target')

            if not init_robot_pos:
                # robot_pos = np.append(robot_pos,Point(round(event.xdata),round(event.ydata)))
                init_robot_pos = True
                plt.scatter([robot_pos[0].x], [robot_pos[0].y], color='black', marker='o', s=50, label='Robot')

            plt.draw()
