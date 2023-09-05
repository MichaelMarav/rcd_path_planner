import networkx as nx
import matplotlib.pyplot as plt



class WeightedGraph:
    def __init__(self):
        self.graph = nx.DiGraph()

    def add_vertex(self, vertex_name, x=0, y=0):
        self.graph.add_node(vertex_name, x=x, y=y)

    def add_edge(self, source, target, weight):
        if source in self.graph.nodes() and target in self.graph.nodes():
            self.graph.add_edge(source, target, weight=weight)
        else:
            print("Source or target node not found!")

    def update_vertex(self, vertex_name, x=None, y=None):
        if vertex_name in self.graph.nodes():
            if x is not None:
                self.graph.nodes[vertex_name]["x"] = x
            if y is not None:
                self.graph.nodes[vertex_name]["y"] = y
        else:
            print("Vertex not found!")

    def traverse_graph(self):
        print("Graph Nodes and Attributes:")
        for node, attrs in self.graph.nodes(data=True):
            print(f"Node {node}: x={attrs['x']}, y={attrs['y']}")

        print("\nGraph Edges and Weights:")
        for source, target, attrs in self.graph.edges(data=True):
            print(f"Edge {source} -> {target}: weight={attrs['weight']}")


    # def find_shortest_path(self, source, target):
    #     try:
    #         shortest_path = nx.shortest_path(self.graph, source=source, target=target, weight="weight", method="dijkstra")
    #         shortest_distance = nx.shortest_path_length(self.graph, source=source, target=target, weight="weight", method="dijkstra")
    #         return shortest_path, shortest_distance
    #     except nx.NetworkXNoPath:
    #         return None, float("inf")
    def find_shortest_path(self, source, target):
        if source in self.graph.nodes() and target in self.graph.nodes():
            shortest_path = nx.shortest_path(self.graph, source=source, target=target, weight='weight')
            return shortest_path
        else:
            print("Source or target node not found!")
            return None
        

    def find_vertex(self,vertex_name):

        if vertex_name in self.graph.nodes():
            x_coord = self.graph.nodes[vertex_name]['x']
            y_coord = self.graph.nodes[vertex_name]['y']
            return x_coord,y_coord
        else:
            print("Vertex not found!")
            return None, None
        

        
    def visualize_shortest_path(self, source, target):
            shortest_path = self.find_shortest_path(source, target)

            if shortest_path:
                x_coords = [self.graph.nodes[node]['x'] for node in shortest_path]
                y_coords = [self.graph.nodes[node]['y'] for node in shortest_path]

                fig2  = plt.figure(figsize=(8, 6))
                plt.scatter(x_coords, y_coords, c='b', label='Shortest Path', marker='o', s=100)

                for i in range(len(shortest_path) - 1):
                    source_node = shortest_path[i]
                    target_node = shortest_path[i + 1]
                    source_x = self.graph.nodes[source_node]['x']
                    source_y = self.graph.nodes[source_node]['y']
                    target_x = self.graph.nodes[target_node]['x']
                    target_y = self.graph.nodes[target_node]['y']
                    plt.plot([source_x, target_x], [source_y, target_y], 'r--', linewidth=2)

                plt.scatter(x_coords[0], y_coords[0], c='g', label='Start', marker='s', s=100)
                plt.scatter(x_coords[-1], y_coords[-1], c='r', label='End', marker='s', s=100)

                plt.xlabel('X')
                plt.ylabel('Y')
                plt.title('Shortest Path Visualization')
                plt.legend()
                plt.grid(True)
                plt.show()
# Create an instance of the WeightedGraph class
# my_graph = WeightedGraph()

# # Add vertices and edges
# my_graph.add_vertex("A", x=0, y=0)
# my_graph.add_vertex("B", x=1, y=2)
# my_graph.add_vertex("C", x=3, y=1)
# my_graph.add_vertex("D", x=2, y=3)

# my_graph.add_edge("A", "B", 2.5)
# my_graph.add_edge("B", "C", 1.0)
# my_graph.add_edge("C", "D", 3.0)
# my_graph.add_edge("D", "A", 1.5)

# # Print the initial graph
# my_graph.traverse_graph()

# # Update vertex attributes
# my_graph.update_vertex("A", x=5, y=5)
# my_graph.update_vertex("B", x=10)

# # Print the updated graph
# my_graph.traverse_graph()
