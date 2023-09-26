
def print_green(text):
    print("\033[32m" + text)
    print("\033[0m")
    
    
    
def print_red(text):
    print("\033[31m" + text)
    print("\033[0m")
    

def user_input_mode():
    print_green(" \n\n - Ray Casting and Diffusion Path Planning (RCD) -\n\n")
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
