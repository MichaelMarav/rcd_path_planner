import networkx as nx

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

# Create an instance of the WeightedGraph class
my_graph = WeightedGraph()

# Add vertices and edges
my_graph.add_vertex("A", x=0, y=0)
my_graph.add_vertex("B", x=1, y=2)
my_graph.add_vertex("C", x=3, y=1)
my_graph.add_vertex("D", x=2, y=3)

my_graph.add_edge("A", "B", 2.5)
my_graph.add_edge("B", "C", 1.0)
my_graph.add_edge("C", "D", 3.0)
my_graph.add_edge("D", "A", 1.5)

# Print the initial graph
my_graph.traverse_graph()

# Update vertex attributes
my_graph.update_vertex("A", x=5, y=5)
my_graph.update_vertex("B", x=10)

# Print the updated graph
my_graph.traverse_graph()
