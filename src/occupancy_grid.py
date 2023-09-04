class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y



class Grid:
    
    def __init__(self,size_x,size_y,grid_resolution):
        grid_size = (int(size_x/grid_resolution), int(size_y/grid_resolution))     
    
