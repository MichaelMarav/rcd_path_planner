
def print_green(text):
    print("\033[32m" + text)
    print("\033[0m")
    
    
    
def print_red(text):
    print("\033[31m" + text)
    print("\033[0m")
    

def user_input_mode():
    
    print(" \n\n -- \033[31mR\033[0may \033[31mC\033[0masting and \033[31mD\033[0miffusion Path Planning Algorithm (RCD --\n\n")    
    
    while True:
        print("Choose the RCD mode:\n 1. Interactive mode \n 2. Offline Datasets")
        mode_input = int(input("Enter the number for the option and press enter\nInput:"))
        if mode_input == 1:
            print_green("Interactive Mode")
            return 1
        elif mode_input == 2:
            print_green("Offline Mode")
            return 2
        else:
            print_red("Invalid input, try again")



# Utility objects Objects
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y  

class Edge:
    def __init__(self,edge_id,start_node,end_node):
        self.edge_id    = edge_id # Unique String for specifing which edge is this
        self.start_node = start_node
        self.end_node   = end_node


