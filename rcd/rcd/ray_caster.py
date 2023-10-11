from rcd.interactive_grid import InteractiveGridGenerator
from rcd.utilities import *
from rcd.setup import *
import sys


class RCD:
    
    random_source_dir  = True    
    
    file_path = "../config/rcd_params.yaml"

    grid_generator = None
    robot_graph  = nx.Graph()
    target_graph = nx.Graph() 
    
    
    visited_robot  = np.empty(0,dtype=object)
    visited_target = np.empty(0,dtype=object)
    
    grid         = None 
    grid_edge_id = None
    grid_resolution = None

    grid_size  = None
    robot_pos  = None 
    target_pos = None
    robot_size = None
    cant_cast_robot_count = None 
    
    path_found = False
    
    num_beams = None
    real_time_plotting = None
    
    path = None
    def __init__(self):
        # Load rcd parameters
        self.load_grid_params()

        grid_generator = InteractiveGridGenerator()
        self.grid       = grid_generator.grid
        self.robot_size = grid_generator.robot_size
        self.robot_pos  = grid_generator.robot_pos
        self.target_pos = grid_generator.target_pos
        self.grid_size  = grid_generator.grid_size 
        self.grid_edge_id = np.empty(self.grid_size,dtype=object) # Contains the edge id that passes through each cell
        self.grid_resolution  = grid_generator.grid_resolution 
        self.cant_cast_robot_count = 0
        self.cant_cast_target_count = 0
        # Add Robot Node to the graph
        self.robot_graph.add_node( "R", x=self.robot_pos[0] , y = self.robot_pos[1] , ray_casted = False)
        self.target_graph.add_node("G", x=self.target_pos[0], y = self.target_pos[1], ray_casted = False)
        
        self.initialize_edges()
        
        while not self.path_found:
            
            # Robot Cast
            for i in range(self.robot_graph.number_of_nodes()):
                node = self.low_variance_resampling(self.robot_graph)

                if not self.get_casted_flag(self.robot_graph,node):
                    casting_x,casting_y = self.get_node_position(self.robot_graph,node)
                    self.ray_casting_robot(casting_x, casting_y, parent = node)
                    self.robot_graph.nodes[node]['ray_casted'] = True
                    if self.path_found:
                        break

                    self.cant_cast_robot_count = 0
                    if self.real_time_plotting:
                        input("Press something to continue")
                    
                else:
                    self.cant_cast_robot_count += 1
                
                    
            for i in range(self.target_graph.number_of_nodes()):
                node = self.low_variance_resampling(self.target_graph)

                if not self.get_casted_flag(self.target_graph,node):
                    casting_x,casting_y = self.get_node_position(self.target_graph,node)
                    self.ray_casting_target(casting_x, casting_y, parent = node)
                    self.target_graph.nodes[node]['ray_casted'] = True
                    self.cant_cast_target_count = 0
                    if self.path_found:
                        break
                    if self.real_time_plotting:
                        input("Press something to continue")
                else:
                    self.cant_cast_target_count += 1
            # print(self.cant_cast_target_count,self.cant_cast_robot_count)
            print("Number of nodes : ", self.robot_graph.number_of_nodes())
            if (self.cant_cast_robot_count > self.robot_graph.number_of_nodes()-1 or self.cant_cast_target_count > self.target_graph.number_of_nodes()-1):
                sys.exit("Path does not exist")
                


        # Find the shortest path (Node points) from robot and target to intersection then combine them and plot them
        init_path = self.find_shortest_path(self.robot_graph,self.target_graph)

        # Generate new samples
        enriched_path = self.generate_samples(init_path)

        self.path = self.reduce_path_with_LoS(enriched_path)
        
        self.plot_path()
                   
                   
                   
                   
                   
    def plot_path(self):
        # PLOT PATH WITH STRAIGHT LINES

        self.grid = np.where((self.grid != 0) & (self.grid != 100), 0, self.grid) # Remove rays

        fig2 = plt.figure(figsize=(self.grid_size[0]*self.grid_resolution, self.grid_size[1]*self.grid_resolution))
        im = plt.imshow(self.grid.T, cmap='binary', origin='upper', vmin=0, vmax=100)
        im.set_data(self.grid.T)
        

        plt.title('Path')
        # Convert the path to a NumPy array for easier indexing
        self.path = np.array(self.path)
        plt.axis('off')  # Turn on axis for grid lines

        # Plot the grid
        plt.plot(self.path[:, 0], self.path[:, 1], marker='o', color='red', markersize=5,zorder = 1)  # Adjust color and marker size as needed
        plt.scatter(self.path[0, 0], self.path[0, 1], s = 50,  color='blue',zorder=2)  # ROBOT
        plt.scatter(self.path[-1, 0], self.path[-1, 1], s = 50,  color='green',zorder=2)  # Target

        
        plt.gca().invert_yaxis()

        plt.show(block = False)


    
    # Loads config Parameters from the .yaml file 
    def load_grid_params(self):
        # Read data from the YAML file
        with open(self.file_path, "r") as file:
            config_options = yaml.load(file, Loader=yaml.FullLoader)
        
        self.num_beams          = config_options["num_beams"]
        self.real_time_plotting = config_options["real_time_plotting"]
    
    
    
    def initialize_edges(self):
        # Every element in the grid_edge_id array is a Null object
        for i in range(self.grid_edge_id.shape[0]):
            for j in range(self.grid_edge_id.shape[1]):
                self.grid_edge_id[i,j] = Edge(None,None,None)
                
                
                
         
    # Returns true if the node has been casted else it returns false
    def get_casted_flag(self,graph,node_name):
        return nx.get_node_attributes(graph,'ray_casted')[node_name]   
    
    
    # Returns the x,y coordinates of a node that belongs to graph
    def get_node_position(self,graph,node_name):
        x_v = nx.get_node_attributes(graph,'x')[node_name]
        y_v = nx.get_node_attributes(graph,'y')[node_name]
        return x_v,y_v

    def check_closest_node_distance(self,x,y,visited_points,closest_threshold):
        for pos in visited_points:
            if math.sqrt((pos.x-x)**2 + (pos.y-y)**2) < closest_threshold:
                return False
        return True


    # Ray casting from parent node and create childs and directed edges
    def ray_casting_robot(self,x,y,parent):

        child_id = 1
        edge_name = ""


        angle_list = np.arange(0,360,int(360/self.num_beams))
        if self.random_source_dir:
            random_rotation_bias = random.randint(0, 180) # Spin the orientation of the beams
            angle_list += random_rotation_bias


        for angle in angle_list: 

            # Saves the indices to change in the grid
            indexes_to_change = [] 

            angle_rad = np.radians(angle)

            stop_beam = False # Stop ray casting flag for this angle

            valid_ray = False # Flag that specifies if the ray is valid-> Not close enough to another node
            
            # Starting distance (radius) from ray casting 
            dis = 2 #(robot_size/2)/grid_resolution 
            prev_x = int(math.ceil(x))
            prev_y = int(math.ceil(y))

            # Propagates ray until 1. Ray hits wall 2. Ray hits another ray 3. Robot Ray and Goal ray are connected (path found)
            while not stop_beam and not self.path_found:
                dis += 1
                beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
                beam_y = int(math.ceil(y + dis * np.sin(angle_rad)))

                # Case 1: If beam hit wall
                if self.grid[beam_x,beam_y] == 100:
                    stop_beam = True          


                    if self.check_closest_node_distance(beam_x,beam_y,self.visited_robot,(self.robot_size)/self.grid_resolution):          
                        valid_ray = True

                        # Add node to graph
                        child_name = parent + str(child_id)
                        self.robot_graph.add_node(child_name, x = prev_x, y = prev_y, ray_casted = False)
                        self.robot_graph.add_edge(parent, child_name, weight = dis)

                        child_id += 1 # Update index for next child
                        
                        edge_name = parent + "-" + child_name

                        self.visited_robot  = np.append(self.visited_robot,Point(prev_x,prev_y)) # Save the places that the robot has visited
                        plt.plot([x,prev_x], [y,prev_y], c = 'b',zorder = 1)

                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Collision Points', zorder =2 )
                        plt.show(block = False)
                            # plt.plot([robot_pos[0],self.visited_robot[-1].x],[robot_pos[1],self.visited_robot[-1].y] , color='b', label = "Robot Rays",zorder = 1)
                # Case 2: If beam hits same kind of ray. (Checks in a cross-like manner)
                elif (self.grid[beam_x,beam_y] == 80 or self.grid[beam_x-1,beam_y] == 80 or self.grid[beam_x+1,beam_y] == 80 or self.grid[beam_x,beam_y+1] == 80 or self.grid[beam_x,beam_y-1] == 80):
                    
                    stop_beam = True

                    intersect_point_x,intersect_point_y = self.check_possible_intersection(self.grid,beam_x,beam_y,80) # where exactly the beams intersect
                
                    if intersect_point_x is not None and intersect_point_y is not None and self.check_closest_node_distance(intersect_point_x, intersect_point_y, self.visited_robot, (self.robot_size)/self.grid_resolution):


                        valid_ray = True

                        # Add node
                        child_name = parent + str(child_id)
                        self.robot_graph.add_node(child_name, x = intersect_point_x, y = intersect_point_y, ray_casted = False)
                        self.robot_graph.add_edge(parent, child_name, weight = dis)

                        child_id += 1
                        edge_name = parent + "-" + child_name

                        # Break the edge that has the intersection in it
                        self.split_edge(self.robot_graph, child_name, intersect_point_x, intersect_point_y)


                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'b', zorder  = 1)

                        plt.scatter(intersect_point_x,intersect_point_y,s = 50, color = 'purple', zorder = 3)
                        plt.show(block = False)
                        self.visited_robot  = np.append(self.visited_robot,Point(intersect_point_x,intersect_point_y))
                
                # Case 3: If beam hits the other kind of ray (path found)
                elif (self.grid[beam_x,beam_y] == 40 or self.grid[beam_x-1,beam_y] == 40 or self.grid[beam_x+1,beam_y] == 40 or self.grid[beam_x,beam_y+1] == 40 or self.grid[beam_x,beam_y-1] == 40):
                        self.path_found = True
                        valid_ray = True

                        print("FOUND PATH by robot")
                        intersect_point_x, intersect_point_y = self.check_possible_intersection(self.grid,beam_x,beam_y,40) # where exactly the beams meet

                        # Add node
                        child_name = "F"
                        self.robot_graph.add_node(child_name, x= intersect_point_x, y = intersect_point_y, ray_casted = False)
                        self.robot_graph.add_edge(parent, child_name,weight = dis)
                        child_id += 1
                        edge_name = parent + "-" + child_name
                        


                        intersect_point_x,intersect_point_y = self.check_possible_intersection(self.grid,beam_x,beam_y,40)

                        self.target_graph.add_node(child_name,x= beam_x,y=beam_y, ray_casted = False)

                        self.split_edge(self.target_graph, child_name, intersect_point_x, intersect_point_y)
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'b', zorder = 1)
                        plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'black',zorder = 2)
                        plt.show(block = False)
                else:
                    indexes_to_change.append([beam_x,beam_y])
                
                prev_x = beam_x
                prev_y = beam_y
                if valid_ray:
                    for row_idx, col_idx in indexes_to_change:
                        self.grid[row_idx,col_idx] = 80 # Robot ray has passed
                        self.grid_edge_id[row_idx,col_idx].edge_id = edge_name
                        self.grid_edge_id[row_idx,col_idx].start_node = parent
                        self.grid_edge_id[row_idx,col_idx].end_node    = child_name

        plt.show(block = False)
        
    '''
    Checks if the beam intersects with another one
    Condition--> Returns bool
    '''
    def check_possible_intersection(self,grid,beam_x,beam_y,value):
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


    def split_edge(self,graph,new_node,x,y):
        edge_ = self.grid_edge_id[x,y]
        # I want to remove every edge_id that is the one intersected and I want to create 2 new edges one from the parent to the new point and one from the new node to the child
        node_1 = edge_.start_node
        node_2 = new_node
        node_3 = edge_.end_node

        x_1,y_1 = self.get_node_position(graph,node_1)
        x_2,y_2 = self.get_node_position(graph,node_2)
        x_3,y_3 = self.get_node_position(graph,node_3)

        dis_12 = math.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)
        dis_23 = math.sqrt((x_2-x_3)**2 + (y_2-y_3)**2)


        mask = np.vectorize(lambda obj: obj.edge_id == edge_.edge_id)(self.grid_edge_id)
        indices = np.argwhere(mask) # Stores the indices of grid_edge_id where the edge we are about to delete is located

        edge_name_1 = node_1 + "-" + node_2
        edge_name_2 = node_2 + "-" + node_3

        if x_1 < x_2: 
            for idx in indices:
                if idx[0] < x_2:
                    self.grid_edge_id[idx[0],idx[1]] = Edge(edge_name_1,node_1,node_2)
                else:
                    self.grid_edge_id[idx[0],idx[1]] = Edge(edge_name_2,node_2,node_3)
        else:
            for idx in indices:
                if idx[0] < x_2:
                    self.grid_edge_id[idx[0],idx[1]] = Edge(edge_name_2,node_3,node_2)
                else:
                    self.grid_edge_id[idx[0],idx[1]] = Edge(edge_name_1,node_2,node_1)

        graph.remove_edge(node_1,node_3)
        graph.add_edge(node_1,node_2,weight = dis_12)
        graph.add_edge(node_2,node_3,weight = dis_23)
       


    def low_variance_resampling(self,graph):
        # Create a dictionary to store cumulative weights for each node
        cumulative_weights = {}
        
        # Calculate cumulative weights for all nodes
        for node in graph.nodes():
            neighbors = list(graph.neighbors(node))
            if not neighbors:
                cumulative_weights[node] = 0.0
            else:
                cumulative_weights[node] = sum(graph[node][neighbor]['weight'] for neighbor in neighbors)
        
        # Calculate the total weight of the entire graph
        total_weight = sum(cumulative_weights.values())
        
        # Generate a random number between 0 and the total weight
        random_value = random.uniform(0, total_weight)
        
        # Find the node whose cumulative weight encompasses the random value
        selected_node = None
        cumulative_weight_sum = 0.0
        for node, cumulative_weight in cumulative_weights.items():
            cumulative_weight_sum += cumulative_weight
            if random_value <= cumulative_weight_sum:
                selected_node = node
                break
        
        return selected_node
    
    '''
    Finds the shortest path in both graphs (from R to F and from G to F) and constructs a way point path
    '''
    def find_shortest_path(self,robot_graph,target_graph):
        shortest_robot = nx.shortest_path(robot_graph,source = "R",target = "F", weight='weight')
        x_path = []
        y_path = []
        for name in shortest_robot:
            x_i, y_i = self.get_node_position(robot_graph,name)
            x_path.append(x_i)
            y_path.append(y_i)


        shortest_target = nx.shortest_path(target_graph,source = "G",target = "F",weight='weight')
        shortest_target = shortest_target[::-1]    # shortest_target.pop(0)
        shortest_target.pop(0) # Remove the common node from robot and target path
        

        for name in shortest_target:
            x_i, y_i = self.get_node_position(target_graph,name)
            x_path.append(x_i)
            y_path.append(y_i)

        path = [(x_path[i], y_path[i]) for i in range(len(x_path))]
        return path
        
    '''
    Generates samples on the initial path in order to perform LoS next
    '''
    def generate_samples(self,path):
        generated_path = []
        for p in range(len(path)-1):
            new_point = list(path[p])
            generated_path.append(new_point)

            curr_point = path[p]
            next_point = path[p+1]
            distance_between_points = math.sqrt((curr_point[0] - next_point[0])**2  + (curr_point[1] - next_point[1])**2 )
            angle = math.atan2((next_point[1]-curr_point[1]),(next_point[0]-curr_point[0]))

            dis = self.robot_size/self.grid_resolution
            while dis < distance_between_points:
                x_p = int(math.ceil(curr_point[0] + dis * np.cos(angle)))
                y_p = int(math.ceil(curr_point[1] + dis * np.sin(angle)))
                dis += 2
                if (self.grid[x_p,y_p] == 100):
                    continue
                generated_path.append([x_p,y_p])

        return generated_path
    
    
    
    def reduce_path_with_LoS(self,path):

        reduced_path = [list(path[0])]
        c = 0
        p = 1

        while  p < len(path):
            curr_point = path[c]
            next_point = path[p]

            collision_flag = self.check_collision(curr_point,next_point)
        
            if collision_flag:
                c = p - 1
                reduced_path.append(list(path[c]))
                p+=1 
            else:
                p += 1
        
            # if collision_flag:
            #     # GO BACK
            #     c = p - 1
            #     colision_flag = False
            #     while True:
            #         curr_point = path[c]
            #         collision_flag = check_collision(curr_point,next_point)
            #         if collision_flag:
            #             reduced_path.append(list(path[c+1]))
            #             c = c + 1
            #             p = c + 1
            #             break
            #         else: 
            #             c = c - 1

            # else:
            #     p += 1


        reduced_path.append(list(path[-1]))
        return reduced_path
    
    
    
    
    
    
    
    
    def check_collision(self,curr_point,next_point):
        distance_between_points = math.sqrt((curr_point[0] - next_point[0])**2  + (curr_point[1] - next_point[1])**2 )
        dis = 3
        angle = math.atan2((next_point[1]-curr_point[1]),(next_point[0]-curr_point[0]))
        collision_flag = False
        while dis < distance_between_points and not collision_flag:
            x_p = int(math.ceil(curr_point[0] + dis * np.cos(angle)))
            y_p = int(math.ceil(curr_point[1] + dis * np.sin(angle)))
            dis += 1
            if self.grid[x_p,y_p] == 100:
                collision_flag = True
        if collision_flag:
            return True
        else:
            return False


    # Ray casting from parent node and create childs and directed edges
    def ray_casting_target(self,x,y,parent):
        child_id = 1
        edge_name = ""

        angle_list = np.arange(0,360,int(360/self.num_beams))


        if self.random_source_dir:
            random_rotation_bias = random.randint(0, 180) # Spin the orientation of the beams
            angle_list += random_rotation_bias


        for angle in angle_list: 

            # Saves the indices to change in the grid
            indexes_to_change = [] 

            angle_rad = np.radians(angle)

            stop_beam = False # Stop ray casting flag for this angle

            valid_ray = False # Flag that specifies if the ray is valid-> Not close enough to another node
            
            # Starting distance (radius) from ray casting TODO: Make this start from robot_size
            dis = 2#(robot_size/2)/grid_resolution 
            prev_x = int(math.ceil(x))
            prev_y = int(math.ceil(y))

            # Propagates ray until 1. Ray hits wall 2. Ray hits another ray 3. Robot Ray and Goal ray are connected (path found)
            while not stop_beam and not self.path_found:
                dis += 1
                beam_x = int(math.ceil(x + dis * np.cos(angle_rad)))
                beam_y = int(math.ceil(y + dis * np.sin(angle_rad)))

                # Case 1: If beam hit wall
                if self.grid[beam_x,beam_y] == 100:
                    stop_beam = True          
                    if self.check_closest_node_distance(beam_x,beam_y,self.visited_target,(self.robot_size)/self.grid_resolution):          
                        valid_ray = True

                        # Add node to graph
                        child_name = parent + str(child_id)
                        self.target_graph.add_node(child_name, x = prev_x, y = prev_y, ray_casted = False)
                        self.target_graph.add_edge(parent, child_name, weight = dis)

                        child_id += 1 # Update index for next child
                        
                        edge_name = parent + "-" + child_name

                        self.visited_target  = np.append(self.visited_target,Point(prev_x,prev_y)) # Save the places that the robot has visited
                        plt.plot([x,prev_x], [y,prev_y], c = 'g', zorder = 1)


                        plt.scatter([prev_x], [prev_y], color='red', marker='o', s=50, label='Collision Points', zorder = 3 )
                        # plt.plot([target_pos[0],visited_target[-1].x],[target_pos[1],visited_target[-1].y] , color='purple',label ="Target rays",zorder=1)
                        plt.show(block = False)


                # Case 2: If beam hits same kind of ray. (Checks in a cross-like manner)
                elif (self.grid[beam_x,beam_y] == 40 or self.grid[beam_x-1,beam_y] == 40 or self.grid[beam_x+1,beam_y] == 40 or self.grid[beam_x,beam_y+1] == 40 or self.grid[beam_x,beam_y-1] == 40):
                    
                    stop_beam = True

                    intersect_point_x,intersect_point_y = self.check_possible_intersection(self.grid,beam_x,beam_y,40) # where exactly the beams meet
                
                    if intersect_point_x is not None and intersect_point_y is not None and self.check_closest_node_distance(intersect_point_x, intersect_point_y, self.visited_target,(self.robot_size)/self.grid_resolution):


                        valid_ray = True

                        # Add node
                        child_name = parent+str(child_id)
                        self.target_graph.add_node(child_name,x = intersect_point_x, y =intersect_point_y, ray_casted = False)
                        self.target_graph.add_edge(parent,child_name,weight = dis)

                        child_id += 1
                        edge_name = parent + "-" + child_name

                        # Break the edge that has the intersection in it
                        self.split_edge(self.target_graph, child_name, intersect_point_x, intersect_point_y)


                        plt.scatter(intersect_point_x,intersect_point_y,s = 50, color = 'purple')
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'g', zorder = 1)
                        plt.show(block = False)
                        self.visited_target = np.append(self.visited_target, Point(intersect_point_x,intersect_point_y))
                
                # Case 3: If beam hits the other kind of ray (path found)
                elif (self.grid[beam_x,beam_y] == 80 or self.grid[beam_x-1,beam_y] == 80 or self.grid[beam_x+1,beam_y] == 80 or self.grid[beam_x,beam_y+1] == 80 or self.grid[beam_x,beam_y-1] == 80):
                        self.path_found = True
                        valid_ray = True

                        print("FOUND PATH by Target")

                        # Add node
                        child_name = "F"
                        self.target_graph.add_node(child_name,x = beam_x, y = beam_y, ray_casted = False)
                        self.target_graph.add_edge(parent,child_name,weight = dis)
                        child_id += 1
                        edge_name = parent + "-" + child_name

                        intersect_point_x,intersect_point_y = self.check_possible_intersection(self.grid,beam_x,beam_y,80)

                        self.robot_graph.add_node(child_name,x= beam_x,y=beam_y, ray_casted = False)

                        self.split_edge(self.robot_graph, child_name, intersect_point_x, intersect_point_y)
                        plt.scatter(intersect_point_x,intersect_point_y,s = 120, color = 'black', zorder = 3)
                        plt.plot([x,intersect_point_x], [y,intersect_point_y], c = 'g', zorder = 1)
                        plt.show(block = False)

                else:
                    indexes_to_change.append([beam_x,beam_y])
                
                prev_x = beam_x
                prev_y = beam_y

                if valid_ray:
                    for row_idx, col_idx in indexes_to_change:
                        self.grid[row_idx,col_idx] = 40 # Robot ray has passed
                        self.grid_edge_id[row_idx,col_idx].edge_id    = edge_name
                        self.grid_edge_id[row_idx,col_idx].start_node = parent
                        self.grid_edge_id[row_idx,col_idx].end_node   = child_name
        plt.show(block = False)
        # input("Enter to continue")
