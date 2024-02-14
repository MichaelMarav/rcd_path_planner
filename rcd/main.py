from rcd.ray_caster import RCD
from rcd.utilities import user_input_mode, print_red, print_green
import sys




def main():
    user_input = user_input_mode()
    if  user_input == 1: # Online Mode
        ray_caster = RCD(False, None)
        input("Press something to Exit")     

    elif (user_input == 2): 
        print("Usage: provide the prefix to the sample you want to run RCD. e.g. enter my_sample1 for running Data/my_sample1/my_sample1.ppm with parameters specified at Data/my_sample1/my_sample1.yaml")
        prefix = str(input("Enter the prefix: \n"))
        ray_caster = RCD(True, prefix)
    else:
        print_red("Please enter a valid mode number. Exiting.")
        sys.exit()
        
    print("Would you like to save the map?")
    print_green("Exiting successfuly")

    return
    
if __name__ == "__main__":
    main()
    