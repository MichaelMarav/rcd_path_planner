from rcd.ray_caster import RCD
from rcd.utilities import user_input_mode, print_red, print_green
import sys




def main():
    user_input = user_input_mode()
    if  user_input == 1: # Online Mode
        ray_caster = RCD()
        input("Press something to Exit")     

    elif (user_input == 2): 
        pass # Under construction
    else:
        print_red("Please enter a valid mode number. Exiting.")
        sys.exit()
        
    print_green("Exiting successfuly")
    return
    
if __name__ == "__main__":
    main()
    