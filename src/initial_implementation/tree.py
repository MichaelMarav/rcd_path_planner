
class TreeNode:
    def __init__(self, key, parent=None):
        self.key = key
        self.parent = parent
        self.children = []

    def add_child(self, child):
        child.parent = self
        self.children.append(child)

    def __str__(self, level=0):
        ret = "\t" * level + repr(self.key) + "\n"
        for child in self.children:
            ret += child.__str__(level + 1)
        return ret

class TreeEdge:
    def __init__(self, source, destination, weight):
        self.source = source
        self.destination = destination
        self.weight = weight

    def __str__(self):
        return f"{self.source} -> {self.destination}: {self.weight}"

class Tree:
    def __init__(self):
        self.root = None
        self.edges = []

    def add_node(self, key, parent_key=None):
        new_node = TreeNode(key)
        if parent_key is None:
            if self.root is None:
                self.root = new_node
            else:
                raise ValueError("Root node already exists.")
        else:
            parent = self.find_node(parent_key)
            if parent is None:
                raise ValueError(f"Parent node ({parent_key}) not found.")
            parent.add_child(new_node)

    def add_edge(self, source_key, destination_key, weight):
        self.edges.append(TreeEdge(source_key, destination_key, weight))

    def find_node(self, key):
        return self._find_node_recursive(self.root, key)

    def _find_node_recursive(self, current_node, key):
        if current_node is None:
            return None
        if current_node.key == key:
            return current_node
        for child in current_node.children:
            result = self._find_node_recursive(child, key)
            if result is not None:
                return result
        return None

    def find_shortest_path(self, leaf_key):
        leaf_node = self.find_node(leaf_key)
        if leaf_node is None:
            raise ValueError(f"Leaf node ({leaf_key}) not found.")
        
        path = []
        self._find_shortest_path_recursive(leaf_node, path)
        return path

    def _find_shortest_path_recursive(self, current_node, path):
        if current_node is None:
            return

        path.append(current_node.key)

        if current_node.parent is not None:
            parent_edge = None
            for edge in self.edges:
                if edge.source == current_node.parent.key and edge.destination == current_node.key:
                    parent_edge = edge
                    break

            if parent_edge is not None:
                path[-1] += f" ({parent_edge.weight})"
                
        self._find_shortest_path_recursive(current_node.parent, path)

    def __str__(self):
        return str(self.root)


# if __name__ == "__main__":
#     tree = Tree()

#     # Add nodes to the tree
#     tree.add_node("A")
#     tree.add_node("B", parent_key="A")
#     tree.add_node("C", parent_key="A")
#     tree.add_node("D", parent_key="B")
#     tree.add_node("E", parent_key="B")
#     tree.add_node("F", parent_key="C")

#     # Add weighted edges
#     tree.add_edge("A", "B", 2)
#     tree.add_edge("A", "C", 3)
#     tree.add_edge("B", "D", 1)
#     tree.add_edge("B", "E", 2)
#     tree.add_edge("C", "F", 4)

#     # Find the shortest path from a leaf to the root
#     leaf_key = "D"
#     shortest_path = tree.find_shortest_path(leaf_key)
#     shortest_path.reverse()
#     print(f"Shortest Path from Leaf '{leaf_key}' to Root: {' -> '.join(shortest_path)}")
